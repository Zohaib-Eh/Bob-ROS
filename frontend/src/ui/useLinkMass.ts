import { useEffect, useRef, useState } from "react";
import { linkMass } from "../api/client";
import type { Body, Link, LinkMass } from "../api/types";

/** Backend messages name links by id; show their names instead. */
export function withNames(message: string, links: Link[]): string {
  return message.replace(/[Ll]ink '([^']+)'/g, (all, id: string) => {
    const l = links.find((x) => x.id === id);
    return l ? `Link "${l.name}"` : all;
  });
}

export function useLinkMass(link: Link | undefined, bodies: Record<string, Body>, links: Link[]) {
  const [state, setState] = useState<{ mass: LinkMass | null; error: string | null }>({ mass: null, error: null });
  const names = useRef(links);
  names.current = links;
  useEffect(() => {
    setState({ mass: null, error: null });
    if (!link) return;
    let alive = true;
    const t = setTimeout(() => {
      linkMass(link, link.body_ids.map((id) => bodies[id]))
        .then((mass) => { if (alive) setState({ mass, error: null }); })
        .catch((e) => { if (alive) setState({ mass: null, error: withNames((e as Error).message, names.current) }); });
    }, 300);
    return () => { alive = false; clearTimeout(t); };
  }, [link, bodies]);
  return state;
}
