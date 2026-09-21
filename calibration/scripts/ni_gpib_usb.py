#!/usr/bin/env python3
"""User-space driver for the NI GPIB-USB-HS (3923:709b) over libusb/pyusb.

A pure-Python GPIB controller that talks to the adapter's USB brain (Cypress
FX2) + GPIB controller (NI TNT4882) directly, with NO kernel driver. It works
on macOS/Apple Silicon (where NI ships no driver and linux-gpib can't run) and
is portable anywhere pyusb+libusb run.

Why this exists: on the bench Raspberry Pi, linux-gpib's ni_usb kernel driver
could not complete addressed transfers with this (genuine) adapter — its
interrupt-endpoint status path mis-handles the unit. This driver uses plain
SYNCHRONOUS bulk request->response transfers (no interrupt monitoring) and works.

Protocol ported from linux-gpib ni_usb_gpib.[ch] (GPL-2.0). Requires:
    brew install libusb   &&   pip install pyusb

Minimal by design: single controller, one instrument at a time, no serial poll
or async SRQ. Sufficient for DMM-referenced calibration. See dmm.py for the
34401A wrapper.
"""
from __future__ import annotations

import usb.core
import usb.util

NI_VID, NI_HS_PID = 0x3923, 0x709b

# bulk endpoints (NI-USB-HS)
_EP_OUT, _EP_IN = 0x02, 0x84
_BMREQ_VENDOR_IN = 0xC0

# bulk instruction / block ids (from ni_usb_gpib.h)
_REG_WRITE_ID = 0x09
_TERM_ID = 0x04
_IBCAC_ID = 0x01          # take control
_CMD_ID = 0x0c            # send command bytes (ATN)
_WRITE_ID = 0x0d          # send data bytes
_READ_ID = 0x0a           # receive data bytes
_IBRD_DATA_ID = 0x36
_IBRD_EXT_ID = 0x37
_IBRD_STATUS_ID = 0x38

# subdevices
_TNT, _UNK2, _UNK3 = 1, 2, 3

_ERR_NAMES = {0: "NO_ERROR", 1: "ABORTED", 2: "ATN_STATE", 3: "ADDRESSING",
              4: "NO_LISTENER", 5: "TIMEOUT", 6: "EOSMODE", 7: "NO_BUS"}

# GPIB command bytes
_UNL, _UNT = 0x3f, 0x5f


def _lad(addr: int) -> int:  # listen address
    return 0x20 + addr


def _tad(addr: int) -> int:  # talk address
    return 0x40 + addr


class GpibError(RuntimeError):
    pass


def _timeout_code(usec: int) -> int:
    """ni_usb_timeout_code: map a timeout in microseconds to the TNT code byte."""
    table = [(0, 0xf0), (10, 0xf1), (30, 0xf2), (100, 0xf3), (300, 0xf4),
             (1000, 0xf5), (3000, 0xf6), (10000, 0xf7), (30000, 0xf8),
             (100000, 0xf9), (300000, 0xfa), (1000000, 0xfb), (3000000, 0xfc),
             (10000000, 0xfd)]
    if usec == 0:
        return 0xf0
    for limit, code in table[1:]:
        if usec <= limit:
            return code
    return 0xfe


def _build_init_writes(pad: int, master: bool):
    """The ni_usb_setup_init 26-register TNT4882 bring-up (t1=2000ns, sad off, no BIN)."""
    A = 0x0a  # nec7210_to_tnt4882_offset(AUXMR) = 2*5
    return [
        (_UNK3, 0x10, 0x00),
        (_TNT, 0x1c, 0x22),                     # CMDR SOFT_RESET
        (_TNT, A, 0x81),                        # AUXMR AUXRA|HR_HLDA
        (_TNT, 0x06, 0x81),                     # AUXCR
        (_TNT, 0x0d, 0x01),                     # HSSEL TNT_ONE_CHIP_BIT
        (_TNT, A, 0x02),                        # AUXMR AUX_CR
        (_TNT, 0x1d, 0x80),                     # IMR0 ALWAYS
        (_TNT, 0x02, 0x00),                     # IMR1 (2*1)
        (_TNT, 0x04, 0x00),                     # IMR2 (2*2)
        (_TNT, 0x12, 0x00),                     # IMR3
        (_TNT, A, 0x51),                        # AUXMR AUX_HLDI
        (_TNT, A, 0xe1),                        # t1: AUXRI|SISB
        (_TNT, A, 0xa0),                        # t1: AUXRB
        (_TNT, 0x17, 0x00),                     # t1: KEYREG 0
        (_TNT, A, 0x48),                        # AUXMR AUXRG|NTNL_BIT
        (_TNT, 0x1c, 0x03 if master else 0x02),  # CMDR SETSC/CLRSC
        (_TNT, A, 0x16),                        # AUXMR AUX_CIFC
        (_TNT, 0x0c, pad),                      # ADR (2*6) = pad
        (_UNK2, 0x00, pad),                     # UNKNOWN2 addr0
        (_TNT, 0x0c, 0xe0),                     # ADR HR_ARS|HR_DT|HR_DL  (sad disabled)
        (_TNT, 0x08, 0x31),                     # ADMR (2*4) HR_TRM0|HR_TRM1|HR_ADM0
        (_UNK2, 0x01, 0x00),                    # UNKNOWN2 addr1
        (_UNK2, 0x02, 0xfd),                    # UNKNOWN2
        (_TNT, 0x0f, 0x11),                     # TNT undocumented 0xf
        (_TNT, A, 0x00),                        # AUXMR AUX_PON
        (_TNT, A, 0x01),                        # AUXMR AUX_CPPF
    ]


class NIUSBGPIB:
    """A minimal GPIB controller over an NI GPIB-USB-HS. Use as a context manager.

        with NIUSBGPIB(my_pad=0) as gpib:
            print(gpib.query(22, "*IDN?"))
    """

    def __init__(self, my_pad: int = 0, master: bool = True,
                 timeout_usec: int = 3_000_000, usb_timeout_ms: int = 5000):
        self.my_pad = my_pad
        self.master = master
        self._tcode = _timeout_code(timeout_usec)
        self._usb_ms = usb_timeout_ms
        self.dev = usb.core.find(idVendor=NI_VID, idProduct=NI_HS_PID)
        if self.dev is None:
            raise GpibError("NI GPIB-USB-HS (3923:709b) not found (is it plugged into this host?)")
        try:
            self.dev.set_configuration()
        except Exception:  # already configured on macOS
            pass
        self._ready()
        self._init_chip()

    # -- context manager --
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        if self.dev is not None:
            usb.util.dispose_resources(self.dev)
            self.dev = None

    # -- low level usb --
    def _out(self, data):
        n = self.dev.write(_EP_OUT, bytes(data), timeout=self._usb_ms)
        if n != len(data):
            raise GpibError(f"short bulk OUT {n}/{len(data)}")

    def _in(self, length):
        return bytes(self.dev.read(_EP_IN, length, timeout=self._usb_ms))

    @staticmethod
    def _pad_term(buf):
        while len(buf) % 4:
            buf.append(0x00)
        buf += [_TERM_ID, 0, 0, 0]
        return buf

    def _status12(self, tag):
        resp = self._in(0x10)
        if len(resp) != 12:
            raise GpibError(f"{tag}: expected 12-byte status, got {len(resp)}")
        return resp[0], (resp[1] << 8) | resp[2], resp[3]  # id, ibsta, error_code

    # -- init --
    def _ready(self):
        ser = self.dev.ctrl_transfer(_BMREQ_VENDOR_IN, 0x41, 0, 0, 16, timeout=self._usb_ms)
        rdy = self.dev.ctrl_transfer(_BMREQ_VENDOR_IN, 0x40, 0, 0, 16, timeout=self._usb_ms)
        if ser[0] != 0x41 or rdy[0] != 0x40:
            raise GpibError("adapter did not pass the ready handshake")

    def _write_registers(self, writes):
        buf = [_REG_WRITE_ID, len(writes), 0x00]
        for dev, addr, val in writes:
            buf += [dev, addr & 0xff, val & 0xff]
        self._pad_term(buf)
        self._out(buf)
        resp = self._in(0x20)
        sid, err, completed = resp[0], resp[3], (resp[8] if len(resp) > 8 else -1)
        if sid != _REG_WRITE_ID or err or completed != len(writes):
            raise GpibError(f"register write failed (id=0x{sid:02x} "
                            f"err={_ERR_NAMES.get(err, err)} completed={completed})")

    def _init_chip(self):
        self._write_registers(_build_init_writes(self.my_pad, self.master))
        # take control (ATN); NO_BUS here is benign with nothing driving the bus
        self._out(self._pad_term([_IBCAC_ID, 1, 0, 0]))
        self._status12("take_control")

    # -- gpib primitives --
    def command(self, cmd_bytes):
        """Send up to 16 command bytes (ATN asserted)."""
        if len(cmd_bytes) > 16:
            raise GpibError("command chunk > 16 bytes")
        cc = (~(len(cmd_bytes) - 1)) & 0xff
        buf = [_CMD_ID, cc, 0x00, self._tcode] + list(cmd_bytes)
        self._pad_term(buf)
        self._out(buf)
        _, _, err = self._status12("command")
        if err:
            raise GpibError(f"command error {_ERR_NAMES.get(err, err)}")

    def write(self, addr: int, data: bytes, eoi: bool = True):
        """Address instrument `addr` as listener and send data (EOI on last byte)."""
        self.command([_UNL, _tad(self.my_pad), _lad(addr)])
        cc = (~(len(data) - 1)) & 0xffff
        buf = [_WRITE_ID, cc & 0xff, (cc >> 8) & 0xff, self._tcode, 0, 0,
               0x8 if eoi else 0, 0] + list(data)
        self._pad_term(buf)
        self._out(buf)
        _, _, err = self._status12("write")
        if err:
            raise GpibError(f"write error {_ERR_NAMES.get(err, err)}")

    def read(self, addr: int, length: int = 256) -> bytes:
        """Address instrument `addr` as talker and read up to `length` bytes."""
        self.command([_UNL, _lad(self.my_pad), _tad(addr)])
        buf = [_READ_ID, 0x00, 0x00, self._tcode,
               (~(length - 1)) & 0xff, ((~(length - 1)) >> 8) & 0xff, 0, 0]
        buf += [_REG_WRITE_ID, 2, 0x00, _TNT, 0x0a, 0x51, _TNT, 0x0a, 0x55]  # AUX_HLDI, AUX_CLEAR_END
        self._pad_term(buf)
        self._out(buf)
        resp = self._in((length // 30 + 2) * 0x20)
        return self._parse_read(resp)

    def query(self, addr: int, cmd, length: int = 256) -> str:
        """Write a command then read the reply, returned as a stripped string."""
        self.write(addr, cmd if isinstance(cmd, bytes) else cmd.encode())
        return self.read(addr, length).decode(errors="replace").strip()

    @staticmethod
    def _parse_read(resp) -> bytes:
        i, data, n_blocks, blocklen = 0, bytearray(), 0, 15
        while i < len(resp) and resp[i] in (_IBRD_DATA_ID, _IBRD_EXT_ID):
            if resp[i] == _IBRD_DATA_ID:
                blocklen, i = 15, i + 1
            else:
                blocklen, i = 30, i + 2  # 0x37 then a pad byte
            for _ in range(blocklen):
                if i < len(resp):
                    data.append(resp[i])
                    i += 1
            n_blocks += 1
        err = resp[i + 3] if i + 3 < len(resp) else 0
        i += 8 + 1  # status block (8) + separator
        if n_blocks:
            last = resp[i] if i < len(resp) else 0
            actual = (n_blocks - 1) * blocklen + last
        else:
            actual = 0
        if err:
            raise GpibError(f"read error {_ERR_NAMES.get(err, err)}")
        return bytes(data[:actual])


if __name__ == "__main__":
    import sys
    addr = int(sys.argv[1]) if len(sys.argv) > 1 else 22
    with NIUSBGPIB() as g:
        print(f"pad {addr} *IDN? -> {g.query(addr, '*IDN?')!r}")
