// Presentation-only decoding of the Rust oracle's fixed-stride observation projection.
export const STRIDE = 9;
export const KINDS = Object.freeze({
  1: "Ray", 2: "Penetrates", 3: "Position", 4: "Rotation", 5: "Grounded",
  6: "Depenetration", 7: "Contact", 8: "Blocked", 9: "Support", 10: "Contact event",
  11: "Velocity", 12: "Tension", 13: "Hit", 14: "Overlaps", 15: "Inertia", 16: "Joint",
});

const decoder = new TextDecoder();
const utf8 = (e, ptr, len) => decoder.decode(new Uint8Array(e.memory.buffer, ptr, len));

export function readScenarios(e) {
  return Array.from({ length: e.oracle_scenarios() }, (_, index) => ({
    index,
    name: utf8(e, e.oracle_name_ptr(index), e.oracle_name_len(index)),
    capabilities: utf8(e, e.oracle_capabilities_ptr(index), e.oracle_capabilities_len(index))
      .split(/\s+/).filter((id) => /^C\d+$/.test(id)),
    ticks: e.oracle_ticks(index),
  }));
}

export function buildCatalogue(capabilities, scenarios) {
  const catalogue = capabilities.map((capability) => ({
    ...capability,
    scenarios: scenarios.filter((scenario) => scenario.capabilities.includes(capability.id)),
  }));
  const missing = catalogue.filter((entry) => !entry.scenarios.length);
  if (missing.length) throw new Error(`No oracle scenario for ${missing.map((entry) => entry.id).join(", ")}`);
  return catalogue;
}

export function decodeRows(raw, ticks) {
  if (raw.length % STRIDE) throw new Error("Invalid oracle trace stride");
  const frames = Array.from({ length: ticks }, () => []);
  const rows = [];
  let previousTick = -1, previousOrdinal = -1;
  for (let i = 0; i < raw.length; i += STRIDE) {
    const [tick, ordinal, kind] = raw.subarray(i, i + 3);
    if (!Number.isInteger(tick) || tick < 0 || tick >= ticks ||
        !Number.isInteger(ordinal) || ordinal < 0 || !KINDS[kind] ||
        Array.from(raw.subarray(i, i + STRIDE)).some((number) => !Number.isFinite(number)) ||
        tick < previousTick || (tick === previousTick && ordinal <= previousOrdinal)) {
      throw new Error(`Invalid oracle observation at row ${i / STRIDE}`);
    }
    const row = { tick, ordinal, kind, values: Array.from(raw.subarray(i + 3, i + STRIDE)) };
    frames[tick].push(row);
    rows.push(row);
    previousTick = tick;
    previousOrdinal = ordinal;
  }
  return { rows, frames };
}

export function readTrace(e, scenario, side) {
  if (e.oracle_trace(scenario.index, side) !== 0) throw new Error("Oracle replay rejected this scenario");
  if (e.oracle_trace_stride() !== STRIDE) throw new Error("Incompatible oracle display contract");
  const length = e.oracle_trace_len();
  // Copy before running the second side: the WASM Vec and even the memory buffer may move.
  const raw = new Float64Array(new Float64Array(e.memory.buffer, e.oracle_trace_ptr(), length));
  const data = decodeRows(raw, scenario.ticks);
  return { ...data, displayed: data.rows.length, total: e.oracle_trace_total() };
}

export function pairedFrame(engine, rapier, tick) {
  const a = new Map(engine.frames[tick].map((row) => [row.ordinal, row]));
  const b = new Map(rapier.frames[tick].map((row) => [row.ordinal, row]));
  return [...new Set([...a.keys(), ...b.keys()])].sort((x, y) => x - y)
    .map((ordinal) => ({ ordinal, engine: a.get(ordinal), rapier: b.get(ordinal) }));
}

export function visualMode(engine, rapier) {
  const kinds = new Set([...engine.rows, ...rapier.rows].map((row) => row.kind));
  if (kinds.has(3)) return "position";
  if (kinds.has(4)) return "rotation";
  if (kinds.has(10)) return "events";
  return "probes";
}

const fmt = (value) => Number(value.toFixed(3)).toString();
const vector = (values, n) => `[${values.slice(0, n).map(fmt).join(", ")}]`;

export function observationText(row) {
  if (!row) return "—";
  const v = row.values;
  switch (row.kind) {
    case 1: return v[0] ? `Ray · hit at t=${fmt(v[1])}, normal ${vector(v.slice(2), 3)}` : "Ray · clear";
    case 2: return `Penetrates · ${v[0] ? "yes" : "no"}`;
    case 3: return `Position · ${vector(v, 3)} m`;
    case 4: return `Rotation · quaternion ${vector(v, 4)}`;
    case 5: return `Grounded · ${v[0] ? "yes" : "no"}`;
    case 6: return `Depenetration · ${vector(v, 3)} m`;
    case 7: return v[0] ? `Contact · distance ${fmt(v[1])} m, normal ${vector(v.slice(2), 3)}` : "Contact · none";
    case 8: return `Blocked · ${v[0] ? "yes" : "no"}`;
    case 9: return v[0] ? `Support · gap ${fmt(v[1])} m` : "Support · none";
    case 10: return `Contact ${v[0] ? "started" : "stopped"} · bodies ${v[1]} / ${v[2]}`;
    case 11: return `Velocity · ${vector(v, 3)} m/s`;
    case 12: return `Tension · ${fmt(v[0])} N`;
    case 13: return v[0] ? `Hit · body ${v[1]} at t=${fmt(v[2])}` : "Hit · none";
    case 15: return `Inertia · ${vector(v, 3)} kg·m²`;
    case 16: return `Joint · angle ${fmt(v[0])} rad, rate ${fmt(v[1])} rad/s, motor ${fmt(v[2])} N·m`;
    case 14: {
      const labels = [];
      for (let bit = 0; bit < 64; bit += 1) {
        const word = bit < 32 ? v[0] : v[1];
        if (Math.floor(word / 2 ** (bit % 32)) % 2) labels.push(bit);
      }
      return `Overlaps · bodies ${labels.length ? labels.join(", ") : "none"}`;
    }
    default: throw new Error(`Unknown observation kind ${row.kind}`);
  }
}
