/** Middle-ellipsis: "IRB1300_7_12KG-1400-IP40-338203" → "IRB1300_…338203". */
export function shortName(name: string, max = 24): string {
  if (name.length <= max) return name;
  const keep = max - 1;
  const head = Math.ceil(keep / 2);
  const tail = Math.floor(keep / 2);
  return `${name.slice(0, head)}…${name.slice(name.length - tail)}`;
}

export const deg = (r: number) => (r * 180) / Math.PI;
export const rad = (d: number) => (d * Math.PI) / 180;
export const fmtLen = (m: number) => (Math.abs(m) >= 1 ? `${m.toFixed(2)} m` : `${(m * 1000).toFixed(0)} mm`);
