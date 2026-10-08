import { Anchor } from "lucide-react";
import type { ReactNode } from "react";
import type { Joint } from "../api/types";
import { chainDepth } from "../model/nextJoint";
import { directJoints } from "../model/tree";
import type { Ctx } from "./ctx";
import { shortName } from "./format";
import { JointIcon } from "./icons";

export default function ChainView({ ctx }: { ctx: Ctx }) {
  const { model, setSelection, viewer } = ctx;
  const root = model.rootLinkId;
  if (!root) return null;
  const { joints } = directJoints(model.links, model.joints, root);
  const depth = chainDepth(model);
  const loose = model.links.filter((l) => !depth.has(l.id));
  const name = (id: string) => model.links.find((l) => l.id === id)?.name ?? id;
  const focus = (id: string) => {
    setSelection({ kind: "link", id });
    viewer?.focus(model.links.find((l) => l.id === id)?.body_ids ?? []);
  };
  const node = (id: string, via?: Joint): ReactNode => {
    const kids = joints.filter((j) => j.parent_link_id === id);
    return (
      <li key={id}>
        <button className="chain-node" data-testid={`chain-${id}`} onClick={() => focus(id)} title={name(id)}>
          {via ? <JointIcon type={via.type} /> : <Anchor size={12} />} {shortName(name(id), 22)}
        </button>
        {kids.length > 0 && <ul>{kids.map((j) => node(j.child_link_id, j))}</ul>}
      </li>
    );
  };
  return (
    <div className="chain" data-testid="chain">
      <h3>Chain</h3>
      <ul className="chain-root">{node(root)}</ul>
      {loose.length > 0 && (
        <>
          <div className="muted small">Not connected yet</div>
          <div className="row">
            {loose.map((l) => (
              <button key={l.id} className="chain-node chain-loose" data-testid={`chain-${l.id}`} title={l.name}
                onClick={() => focus(l.id)}>{shortName(l.name, 18)}</button>
            ))}
          </div>
        </>
      )}
    </div>
  );
}
