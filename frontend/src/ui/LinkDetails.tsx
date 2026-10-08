import { useEffect, useState } from "react";
import { linkMass } from "../api/client";
import type { LinkMass } from "../api/types";
import type { LinkProps } from "../model/reducer";
import { DENSITY_PRESETS } from "../model/types";
import type { DetailsProps } from "./DetailsPanel";

export default function LinkDetails({ model, dispatch, linkId }: DetailsProps & { linkId: string }) {
  const link = model.links.find((l) => l.id === linkId);
  const [mass, setMass] = useState<LinkMass | string | null>(null);
  const bodies = link ? link.body_ids.map((id) => model.bodies[id]) : [];

  useEffect(() => {
    if (!link) return;
    let alive = true;
    const t = setTimeout(() => {
      linkMass(link, bodies)
        .then((m) => alive && setMass(m))
        .catch((e) => alive && setMass((e as Error).message));
    }, 300);
    return () => { alive = false; clearTimeout(t); };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [link, ...bodies]);

  if (!link) return null;
  const set = (props: LinkProps) => dispatch({ type: "setLinkProps", linkId, props });
  const preset = Object.entries(DENSITY_PRESETS).find(([, v]) => v === link.density_kg_m3)?.[0] ?? "";

  return (
    <div>
      <h3>{link.name}</h3>
      <label>Material{" "}
        <select data-testid="density-preset" value={preset}
          onChange={(e) => { if (e.target.value) set({ density_kg_m3: DENSITY_PRESETS[e.target.value] }); }}>
          <option value="">Custom</option>
          {Object.keys(DENSITY_PRESETS).map((k) => <option key={k} value={k}>{k}</option>)}
        </select>
      </label>
      <label>Density (kg/m³){" "}
        <input data-testid="density" type="number" step="any" value={link.density_kg_m3}
          onChange={(e) => set({ density_kg_m3: Number(e.target.value) })} />
      </label>
      <label>Mass override (kg){" "}
        <input data-testid="mass-override" type="number" step="any" value={link.mass_override_kg ?? ""}
          onChange={(e) => set({ mass_override_kg: e.target.value === "" ? null : Number(e.target.value) })} />
      </label>
      <label>
        <input data-testid="accept-approx" type="checkbox" checked={link.accept_approximate_mass}
          onChange={(e) => set({ accept_approximate_mass: e.target.checked })} />
        Accept approximate mass
      </label>
      <div data-testid="link-mass">
        {mass === null ? "Computing mass…"
          : typeof mass === "string" ? <span className="error">{mass}</span>
          : `Mass: ${mass.mass_kg.toFixed(3)} kg · centre of mass ${mass.center_of_mass.map((x) => x.toFixed(3)).join(", ")}`}
      </div>
    </div>
  );
}
