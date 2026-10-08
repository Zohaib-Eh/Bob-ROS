export function splitExportError(message: string) {
  const body = message.replace(/^Cannot export URDF --\s*/, "").replace(/^structural validation failed:\s*/, "");
  const pieces = body.split(/; (?=[A-Z])/).map((p) => p.trim()).filter(Boolean);
  const byLink: Record<string, string[]> = {};
  const byJoint: Record<string, string[]> = {};
  const general: string[] = [];
  for (const p of pieces) {
    const link = /[Ll]ink '([^']+)'/.exec(p);
    const joint = /^Joint (\S+)/.exec(p);
    if (joint) (byJoint[joint[1]] ??= []).push(p);
    else if (link) (byLink[link[1]] ??= []).push(p);
    else general.push(p);
  }
  return { byLink, byJoint, general };
}
