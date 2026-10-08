export type Selection =
  | { kind: "bodies"; ids: string[] }
  | { kind: "link"; id: string }
  | { kind: "joint"; id: string }
  | { kind: "suggestion"; key: string }
  | null;
