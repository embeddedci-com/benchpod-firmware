# Raspberry Pi GPIB setup (NI GPIB-USB-HS + 34401A)

One-time setup to get the Agilent/Keysight **34401A** talking over an **NI
GPIB-USB-HS** on a Raspberry Pi, then run the bench server. The NI HS is not a
USBTMC device — it needs **linux-gpib** (plus a firmware upload on plug-in).

> Targets Raspberry Pi OS (Debian-based). Commands are a guide — exact package
> names/versions drift; cross-check the linux-gpib README.

## 1. Build & install linux-gpib

```bash
sudo apt update
sudo apt install -y build-essential autoconf libtool flex bison \
                    python3-dev python3-setuptools tk-dev libusb-1.0-0-dev \
                    raspberrypi-kernel-headers   # or: linux-headers-$(uname -r)

# from https://sourceforge.net/projects/linux-gpib/
tar xf linux-gpib-user-*.tar.gz && tar xf linux-gpib-kernel-*.tar.gz
cd linux-gpib-kernel-* && make && sudo make install && cd ..
cd linux-gpib-user-*   && ./configure && make && sudo make install && cd ..
sudo ldconfig
```

The user package also builds the **python bindings** (`import Gpib`) used by
`dmm.py`. Confirm: `python3 -c "import Gpib; print('ok')"`.

## 2. NI GPIB-USB-HS firmware (uploaded on every plug-in)

The HS loads firmware over USB. linux-gpib ships the firmware + an `fxload`
udev rule; install them so the device's **READY** LED goes solid on connect:

```bash
sudo apt install -y fxload
# copy the gpib_firmware / ni_usb_gpib firmware where the udev rule expects it,
# then reload rules (paths per the linux-gpib firmware README):
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Plug in the HS; `lsusb` should show `3923:709b` (NI GPIB-USB-HS), READY LED solid.

## 3. Configure the board + address

`/etc/gpib.conf` — one `interface` (the HS) and the 34401A at its address:

```
interface {
    minor      = 0
    board_type = "ni_usb_b"
    name       = "gpib0"
    pad        = 0
    master     = yes
}
device { name = "dmm" pad = 22 }   # 34401A default GPIB address is 22
```

Initialise + smoke-test:

```bash
sudo gpib_config --minor 0
ibtest                     # try: w *IDN?\n  then  r  -> 34401A IDN string
python3 -c "import Gpib; d=Gpib.Gpib(0, pad=22); d.write(b'*IDN?'); print(d.read(128))"
```

> Set the 34401A GPIB address on its front panel: **Shift → I/O → GPIB ADDR = 22**,
> interface = GPIB.

## 4. Bench server deps + run

```bash
pip3 install -r scripts/requirements-pi.txt    # Flask, pyserial

# move the BOARD console USB-serial onto the Pi; find it:
ls /dev/ttyUSB*            # e.g. /dev/ttyUSB0

python3 scripts/bench_server.py --board-port /dev/ttyUSB0 --gpib-addr 22
# pipeline check with no hardware:
python3 scripts/bench_server.py --mock
```

Confirm from any machine on the network:

```bash
curl http://<pi-ip>:8080/health        # {"ok":true,"board":true,"dmm":"...34401A..."}
curl http://<pi-ip>:8080/dmm/dc?nplc=10
```

## 5. Run a calibration (from the Mac)

```bash
python3 scripts/dac_cal.py --host <pi-ip>          # DAC + lower-range ADC
python3 scripts/adc_cal.py --host <pi-ip>          # full 0-5 V ADC (external source)
```

## Troubleshooting

- `import Gpib` fails → linux-gpib user package/python bindings not installed (step 1).
- `ibtest` can't reach the device → firmware not loaded (READY LED off, step 2) or
  wrong `board_type`/address (step 3).
- DMM `*IDN?` times out → 34401A not in GPIB mode or wrong address (front panel).
- Counterfeit HS → may not load firmware / bind under linux-gpib at all; if it
  won't `*IDN?`, return it (see the buying notes).
