// test_wasm.mjs: replays every call golden_host recorded (build/golden.jsonl, the native build)
// against sensor_def.wasm through sensor_def.mjs and checks the JSON is byte for byte the same.
// Also checks that every number the module prints is the shortest form JavaScript prints, and a
// few facts about the five library sensors.
//
//   node test_wasm.mjs build/sensor_def.wasm build/golden.jsonl ../test/fixtures/sensor_defs
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { loadSensorDef } from "./sensor_def.mjs";

const [wasmPath, goldenPath, fixtures] = process.argv.slice(2);
const sd = await loadSensorDef(readFileSync(wasmPath));
const lines = readFileSync(goldenPath, "utf8").split("\n").filter(Boolean);

let fails = 0;
const fail = (msg) => { fails++; if (fails <= 20) console.log("FAIL " + msg); };

// Number tokens outside strings.
function numbers(text) {
  const out = [];
  for (let i = 0; i < text.length; i++) {
    const c = text[i];
    if (c === '"') {
      for (i++; i < text.length && text[i] !== '"'; i++) if (text[i] === "\\") i++;
    } else if (c === "-" || (c >= "0" && c <= "9")) {
      let j = i + 1;
      while (j < text.length && /[0-9.eE+-]/.test(text[j])) j++;
      out.push(text.slice(i, j));
      i = j - 1;
    }
  }
  return out;
}

const defCache = new Map();
const defText = (c) => {
  if (c.text !== undefined) return c.text;
  if (!defCache.has(c.fixture)) defCache.set(c.fixture, readFileSync(join(fixtures, c.fixture + ".json"), "utf8"));
  return defCache.get(c.fixture);
};

let calls = 0;
for (const line of lines) {
  const c = JSON.parse(line);
  let got;
  switch (c.fn) {
    case "validate": got = sd.validateRaw(defText(c)); break;
    case "preview":
      got = sd.previewRaw(defText(c), c.values, {
        liveRegs: c.live ? Uint8Array.from(c.live) : null, tMs: c.t_ms, input: c.input,
      });
      break;
    case "uart_answer": got = sd.uartAnswerRaw(defText(c), c.values, Uint8Array.from(c.request), { tMs: c.t_ms }); break;
    case "stimulus_validate": got = sd.stimulusValidateRaw(c.text); break;
    case "stimulus_sample": got = sd.stimulusSampleRaw(c.text, c.from_s, c.to_s, c.step_s); break;
    default: fail("unknown call " + c.fn); continue;
  }
  calls++;
  const what = `${c.fn} ${c.fixture ?? (c.text ?? "").slice(0, 40)}${c.t_ms !== undefined ? " t=" + c.t_ms : ""}`;
  if (got !== c.out) {
    let k = 0;
    while (k < got.length && got[k] === c.out[k]) k++;
    fail(`${what}: differs from the host at ${k}:\n  wasm: ${got.slice(Math.max(0, k - 40), k + 60)}\n  host: ${c.out.slice(Math.max(0, k - 40), k + 60)}`);
    continue;
  }
  try { JSON.parse(got); } catch (e) { fail(`${what}: not JSON: ${e.message}`); }
  for (const n of numbers(got)) if (String(Number(n)) !== n) fail(`${what}: number ${n} prints as ${Number(n)} in JS`);
}

// The five library sensors.
const want = { bme280: "i2c", bmp280: "i2c", sht4x: "i2c", mpu6050: "i2c", gps_nmea: "uart" };
for (const [name, kind] of Object.entries(want)) {
  const v = sd.validate(defText({ fixture: name }));
  if (!v.ok || v.kind !== kind || v.name !== name || !v.params.length) fail(`${name}: ${JSON.stringify(v).slice(0, 200)}`);
  const p = sd.preview(defText({ fixture: name }), null, { tMs: 1000 });
  if (!p.ok) fail(`${name} preview: ${p.error}`);
  else if (kind === "i2c" && p.image.length !== 256) fail(`${name}: image is ${p.image.length} bytes`);
  else if (kind === "uart" && !p.text.startsWith("$GP")) fail(`${name}: epoch ${p.text.slice(0, 40)}`);
}
// The wrapper's own conveniences.
const bme = defText({ fixture: "bme280" });
const typo = bme.replace('"registers"', '"registerz"');
const bad = sd.validate(typo);
if (bad.ok || !bad.error.startsWith("registerz:")) fail("typo error: " + JSON.stringify(bad));
const obj = sd.validate(JSON.parse(bme));
if (!obj.ok) fail("validate(object): " + obj.error);

console.log(`${fails ? "FAIL" : "PASS"}: sensor_def.wasm, ${calls} calls replayed against the host build, ${fails} failure(s)`);
process.exit(fails ? 1 : 0);
