// sensor_def.mjs: a small wrapper over sensor_def.wasm (the BenchPod sensor definition engine)
// for browsers, Electron and Node.  No dependencies.  README.md documents the ABI and the JSON
// each call returns.
//
//   import { loadSensorDef } from "./sensor_def.mjs";
//   const sd = await loadSensorDef(new URL("./sensor_def.wasm", import.meta.url));
//   const info = sd.validate(definitionJsonText);      // {ok: true, kind, name, params, ...}
//   const img = sd.preview(definitionJsonText, { temperature_c: 30 }, { tMs: 1000 });
//
// Every call is stateless and synchronous; one module instance runs one call at a time.

export const ABI_VERSION = 1;

const enc = new TextEncoder();
const dec = new TextDecoder();

/** @param {string | object} v */
function jsonText(v) {
  return typeof v === "string" ? v : JSON.stringify(v);
}

/** Base64 to bytes (RFC 4648 with padding). */
export function base64ToBytes(b64) {
  const bin = atob(b64);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}

/** Bytes to base64. */
export function bytesToBase64(bytes) {
  let s = "";
  for (let i = 0; i < bytes.length; i++) s += String.fromCharCode(bytes[i]);
  return btoa(s);
}

/**
 * Load the module.
 * @param {URL | string | Response | ArrayBuffer | Uint8Array | WebAssembly.Module} source
 *   a URL (fetched), a fetch Response, the bytes, or a compiled module.
 */
export async function loadSensorDef(source) {
  let instance;
  if (source instanceof WebAssembly.Module) {
    instance = await WebAssembly.instantiate(source, {});
  } else {
    let bytes = source;
    if (typeof source === "string" || source instanceof URL) source = await fetch(source);
    if (typeof Response !== "undefined" && source instanceof Response) {
      if (!source.ok) throw new Error(`sensor_def.wasm: HTTP ${source.status}`);
      bytes = await source.arrayBuffer();
    }
    ({ instance } = await WebAssembly.instantiate(bytes, {}));
  }
  return new SensorDef(instance);
}

export class SensorDef {
  /** @param {WebAssembly.Instance} instance */
  constructor(instance) {
    this.x = instance.exports;
    this.abiVersion = this.x.abi_version();
    if (this.abiVersion !== ABI_VERSION) {
      throw new Error(`sensor_def.wasm ABI ${this.abiVersion}, this wrapper speaks ${ABI_VERSION}`);
    }
  }

  // Copy byte arrays into the module, run fn(...pointers/lengths, ...rest), return the JSON text.
  _call(fn, buffers, rest) {
    const x = this.x;
    const ptrs = [];
    const args = [];
    try {
      for (const b of buffers) {
        if (b == null) { args.push(0, 0); continue; }
        const p = x.sd_alloc(b.length);
        if (!p) throw new Error("sensor_def.wasm: out of memory");
        ptrs.push(p);
        new Uint8Array(x.memory.buffer, p, b.length).set(b);
        args.push(p, b.length);
      }
      const n = fn(...args, ...rest);
      return dec.decode(new Uint8Array(x.memory.buffer, x.out_ptr(), n));
    } finally {
      for (const p of ptrs) x.sd_free(p);
    }
  }

  // ---- raw calls: the JSON text exactly as the engine wrote it ----
  validateRaw(def) {
    return this._call(this.x.validate, [enc.encode(jsonText(def))], []);
  }
  previewRaw(def, values = null, { liveRegs = null, tMs = 0, input = 0 } = {}) {
    let live = liveRegs;
    if (typeof live === "string") live = base64ToBytes(live);
    const vals = values == null ? null : enc.encode(jsonText(values));
    return this._call(this.x.preview, [enc.encode(jsonText(def)), vals, live], [tMs, input]);
  }
  uartAnswerRaw(def, values = null, request, { tMs = 0 } = {}) {
    const req = typeof request === "string" ? enc.encode(request) : request;
    const vals = values == null ? null : enc.encode(jsonText(values));
    return this._call(this.x.uart_answer, [enc.encode(jsonText(def)), vals, req], [tMs]);
  }
  stimulusValidateRaw(stim) {
    return this._call(this.x.stimulus_validate, [enc.encode(jsonText(stim))], []);
  }
  stimulusSampleRaw(stim, fromS, toS, stepS) {
    return this._call(this.x.stimulus_sample, [enc.encode(jsonText(stim))], [fromS, toS, stepS]);
  }

  // ---- parsed calls ----
  /**
   * Compile a definition (JSON text or object).
   * @returns {{ok: true, kind: "i2c"|"spi"|"uart"|"analog", name: string, label: string,
   *   program_bytes: number, params: {key: string, unit: string, label: string, min: number,
   *   max: number, default: number}[], bus?: object, dynamic?: number[], read_only?: number[],
   *   uart?: object, analog?: object} | {ok: false, error: string, path: string, message: string,
   *   code: number, json_off: number, expr_off: number}}
   */
  validate(def) {
    return JSON.parse(this.validateRaw(def));
  }

  /**
   * What the sensor outputs for these param values at tMs.
   * I2C/SPI: {image_b64, image (Uint8Array, added here), changed, dynamic, read_only, power_on}.
   * UART: {at_ms, text, bytes_b64, epochs, next_ms}.  Analog: {value, unit}.
   * @param {string|object} def
   * @param {Record<string, number> | null} values
   * @param {{liveRegs?: Uint8Array | string | null, tMs?: number, input?: number}} [opts]
   */
  preview(def, values = null, opts = {}) {
    const r = JSON.parse(this.previewRaw(def, values, opts));
    if (r.ok && r.image_b64) r.image = base64ToBytes(r.image_b64);
    if (r.ok && r.bytes_b64 !== undefined) r.bytes = base64ToBytes(r.bytes_b64);
    return r;
  }

  /**
   * Feed request bytes (Uint8Array, or a string sent as UTF-8) to a UART sensor.
   * @returns {{ok: true, answers: {after_byte: number, delay_ms: number, text: string,
   *   bytes_b64: string, bytes: Uint8Array}[]} | {ok: false, error: string}}
   */
  uartAnswer(def, values, request, opts = {}) {
    const r = JSON.parse(this.uartAnswerRaw(def, values, request, opts));
    if (r.ok) for (const a of r.answers) a.bytes = base64ToBytes(a.bytes_b64);
    return r;
  }

  /** Compile a stimulus file: {ok, name, rate_hz, duration_s, repeat, signals, faults}. */
  stimulusValidate(stim) {
    return JSON.parse(this.stimulusValidateRaw(stim));
  }

  /**
   * Sample a stimulus for a chart: {ok, t: number[], signals: {name, instance, param,
   * values: number[]}[], faults: {index, type, ..., windows: [on_s, off_s | null][]}[]}.
   * At most 10000 points per signal.
   */
  stimulusSample(stim, fromS, toS, stepS) {
    return JSON.parse(this.stimulusSampleRaw(stim, fromS, toS, stepS));
  }
}
