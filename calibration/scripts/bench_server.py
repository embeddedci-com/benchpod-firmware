#!/usr/bin/env python3
"""Bench instrument HTTP server — runs on the Raspberry Pi.

Owns the bench hardware (the GPIB DMM and the RP2350B board console) and exposes
it as a small JSON API so a cal client on any machine (e.g. the Mac) can
orchestrate a calibration over the network:

  GET  /health                     -> {ok, board, dmm}
  GET  /dmm/idn                    -> {idn}
  GET  /dmm/dc?nplc=10&samples=1   -> {volts}
  POST /board/dac  {code,channel}  -> {ok, raw}
  GET  /board/adc?avg=32           -> {count, n, spread}
  POST /board/raw  {line}          -> {out}

Run on the Pi (board console + GPIB-DMM attached):
  python3 bench_server.py --board-port /dev/ttyUSB0 --gpib-addr 22
Validate the client/server pipeline with NO hardware (synthetic board + DMM):
  python3 bench_server.py --mock

The server is single-threaded on purpose — the serial console and GPIB bus are
not safe to drive concurrently, and the cal is strictly sequential anyway.
"""
import argparse

from flask import Flask, jsonify, request


def build_app(board, dmm):
    app = Flask(__name__)

    @app.get("/health")
    def health():
        try:
            pong, _ = board.ping()
        except Exception:  # noqa: BLE001
            pong = False
        try:
            idn = dmm.idn()
        except Exception as e:  # noqa: BLE001
            idn = f"error: {e}"
        return jsonify(ok=bool(pong), board=pong, dmm=idn)

    @app.get("/dmm/idn")
    def dmm_idn():
        return jsonify(idn=dmm.idn())

    @app.get("/dmm/dc")
    def dmm_dc():
        nplc = request.args.get("nplc", type=float)
        samples = request.args.get("samples", default=1, type=int)
        return jsonify(volts=dmm.read_dc(nplc=nplc, samples=samples))

    @app.post("/board/dac")
    def board_dac():
        body = request.get_json(force=True, silent=True) or {}
        ok, raw = board.dac_set(int(body["code"]), int(body.get("channel", 0)))
        return jsonify(ok=ok, raw=raw.strip()[-300:])

    @app.get("/board/adc")
    def board_adc():
        avg = request.args.get("avg", default=32, type=int)
        mean, samples = board.adc_avg(avg)
        spread = (max(samples) - min(samples)) if samples else None
        return jsonify(count=mean, n=len(samples), spread=spread)

    @app.post("/board/raw")
    def board_raw():
        body = request.get_json(force=True, silent=True) or {}
        return jsonify(out=board.raw(str(body.get("line", ""))))

    return app


def main():
    ap = argparse.ArgumentParser(description="bench-pod calibration HTTP server (Pi)")
    ap.add_argument("--http-host", default="0.0.0.0")
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--board-port", default="/dev/ttyUSB0")
    ap.add_argument("--board-baud", type=int, default=115200)
    ap.add_argument("--gpib-addr", type=int, default=22)
    ap.add_argument("--gpib-board", type=int, default=0)
    ap.add_argument("--nplc", type=float, default=10.0)
    ap.add_argument("--mock", action="store_true", help="no hardware; synthetic board + DMM")
    args = ap.parse_args()

    if args.mock:
        from board_console import MockBoard
        from dmm import MockDMM
        board = MockBoard()
        dmm = MockDMM(board=board)
        print("[bench] MOCK mode — synthetic board + DMM")
    else:
        from board_console import BoardConsole
        from dmm import Agilent34401A
        board = BoardConsole(args.board_port, args.board_baud)
        dmm = Agilent34401A(gpib_addr=args.gpib_addr, board=args.gpib_board, nplc=args.nplc)
        print(f"[bench] board={args.board_port} dmm=GPIB{args.gpib_board}:{args.gpib_addr}")

    app = build_app(board, dmm)
    print(f"[bench] serving on http://{args.http_host}:{args.http_port}")
    app.run(host=args.http_host, port=args.http_port, threaded=False)


if __name__ == "__main__":
    main()
