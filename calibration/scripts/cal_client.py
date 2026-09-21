#!/usr/bin/env python3
"""HTTP client for bench_server.py + a tiny linear-fit helper.

Runs on the host (Mac). Standard library only — no pip installs needed.
"""
import json
import math
import urllib.parse
import urllib.request


class BenchClient:
    def __init__(self, host, port=8080, timeout=30):
        self.base = f"http://{host}:{port}"
        self.timeout = timeout

    def _get(self, path, **params):
        url = self.base + path
        params = {k: v for k, v in params.items() if v is not None}
        if params:
            url += "?" + urllib.parse.urlencode(params)
        with urllib.request.urlopen(url, timeout=self.timeout) as r:
            return json.load(r)

    def _post(self, path, payload):
        data = json.dumps(payload).encode()
        req = urllib.request.Request(
            self.base + path, data=data, headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return json.load(r)

    def health(self):
        return self._get("/health")

    def dmm_idn(self):
        return self._get("/dmm/idn")["idn"]

    def dmm_dc(self, nplc=None, samples=1):
        return self._get("/dmm/dc", nplc=nplc, samples=samples)["volts"]

    def board_dac(self, code, channel=0):
        return self._post("/board/dac", {"code": int(code), "channel": int(channel)})

    def board_adc(self, avg=32):
        return self._get("/board/adc", avg=avg)

    def board_raw(self, line):
        return self._post("/board/raw", {"line": line})["out"]


def linfit(xs, ys):
    """Least-squares y = a + b*x. Returns (a, b, r2, max_abs_residual)."""
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx == 0:
        raise ValueError("degenerate fit: all x equal")
    b = sxy / sxx
    a = my - b * mx
    resid = [y - (a + b * x) for x, y in zip(xs, ys)]
    ss_res = sum(r * r for r in resid)
    ss_tot = sum((y - my) ** 2 for y in ys)
    r2 = 1 - ss_res / ss_tot if ss_tot else math.nan
    return a, b, r2, max(abs(r) for r in resid)
