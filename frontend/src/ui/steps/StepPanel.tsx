import type { Ctx } from "../ctx";
import Info from "../Info";
import ConnectStep from "./ConnectStep";
import ExportStep from "./ExportStep";
import GroupStep from "./GroupStep";
import PartsStep from "./PartsStep";
import ProgramStep from "./ProgramStep";

export default function StepPanel({ ctx }: { ctx: Ctx }) {
  if (!Object.keys(ctx.model.bodies).length) {
    return (
      <div className="empty" data-testid="empty">
        <p>Import a STEP, IGES, BREP, STL or OBJ file to start.</p>
        <p className="muted">The app turns your assembly into a <b>URDF</b> <Info term="urdf" /> robot description.</p>
      </div>
    );
  }
  switch (ctx.step) {
    case "parts": return <PartsStep ctx={ctx} />;
    case "group": return <GroupStep ctx={ctx} />;
    case "connect": return <ConnectStep ctx={ctx} />;
    case "export": return <ExportStep ctx={ctx} />;
    case "program": return <ProgramStep ctx={ctx} />;
  }
}
