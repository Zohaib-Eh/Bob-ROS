# Guided UI — Design

**Date:** 2026-10-08
**Builds on:** `docs/superpowers/specs/2026-10-08-frontend-design.md` (model layer, API, viewer, export) and the `frontend` branch as of `75925fe`.
**Scope:** replace the frontend's UI layer with a guided, graphical, Fusion-style workflow. The model layer (`src/model/*`), API client and backend stay; the viewer is extended. Local use only, as before.

## 1. Purpose and success

The first frontend works but people can't follow it: joints appear as an unordered pile of "part ↔ part: planar" rows, every link shows a red "Not connected" error before anything has been reviewed, the base lands on a mid-arm part, the arm is drawn orange, and part fixes are hidden. Users think about a robot as a **chain built out from the ground**, so the UI must walk them along that chain, show everything in the 3D view, and explain technical terms without hiding them.

Audience: everyone — robotics people and hobbyists alike. Technical terms (link, joint, revolute, inertia, URDF…) stay visible; hard ones carry an ⓘ with a plain-language explanation.

Success criteria:

- A first-time user takes the ABB IRB1300 STEP (`corpus/irb1300_asm.stp`) from import to a downloaded URDF zip without outside explanation: fixes or accepts the approximate parts, groups the hardware, sets the pedestal as base, connects all links out from the base with motion previewed, exports.
- Every problem (failed part, approximate part, unconnected link, missing mass/axis/limits, invalid name) is visible **in the 3D view** at the part it concerns, and clicking it opens a card with its fix.
- Clicking any part or joint in 3D shows its details next to it.
- No red errors are shown for work the user simply hasn't done yet; blocking problems appear only in the Export checklist.
- Everything from the first frontend still holds: undo/redo of every edit, save/open project, background detection, export error mapping.

## 2. Decisions (from the design conversation)

| Topic | Decision |
|---|---|
| Workflow | Guided steps: **1 Parts · 2 Group · 3 Connect · 4 Export**. Every step stays clickable at any time; nothing is locked. |
| Connecting | Fusion 360 **As-built Joint** pattern: pick fixed part, moving part, joint origin by snap point, motion type; animate; OK. Detected joints pre-fill the tool. |
| Order | Connect walks out from the base: the next joint offered is always one whose fixed side is already connected to the base. |
| Base | Asked at the start of Connect ("Click the part that sits on the ground"), pre-filled with the **lowest** link (min z of its box), not the largest. |
| Terms | Shown as-is with ⓘ hover cards from one shared glossary. Motion types use Fusion's names; ⓘ gives the URDF type. |
| Problems | Drawn in 3D (pins, wireframe boxes, outlines). Non-blocking until Export. |
| Look | Professional neutral, after Fusion / Onshape / Shapr3D patterns (not their branding): neutral grey chrome, one blue accent, colour only for meaning, light theme default, dark theme follows the system. |
| Shortcuts | `J` joint · `G` group · `Space` animate · `F` focus · `Esc` back · `Ctrl+Z`/`Ctrl+Y` undo/redo; each shown in its button's tooltip. |
| Later (not MVP) | Command line (AutoCAD-style), marking menus, timeline. |

## 3. Architecture

```
frontend/src/
  model/      unchanged except: nextJoint.ts (new), baseGuess() added to geometry.ts
  api/        unchanged
  viewer/     Viewer.ts extended: overlays, snap picking, render-on-demand, view cube
              overlays.ts (new): pins, wireframe boxes, link tints, joint glyphs, limit handles, snap glyphs
  ui/         rewritten:
    App.tsx           layout, step state, keyboard shortcuts
    theme.css         design tokens (light + dark), base styles
    Stepper.tsx       top bar: steps with badges, project actions (import, undo/redo, save/open)
    Browser.tsx       left panel: Parts / Links / Joints sections + ChainView
    ChainView.tsx     ⚓ Base → link → link …, dashed gaps, click = fly to
    steps/PartsStep.tsx    size check + problem cards
    steps/GroupStep.tsx    auto-group notice + link cards
    steps/ConnectStep.tsx  base pick, next-joint flow, launches JointTool
    steps/ExportStep.tsx   checklist, play all joints, summary, export
    JointTool.tsx     floating As-built Joint dialog
    InfoCard.tsx      card anchored to a part/joint in 3D
    Info.tsx          ⓘ icon + hover card; glossary.ts holds the texts
```

Dependencies added: `lucide-react` (icons), `@fontsource-variable/inter` (font, served locally). Nothing else.

UI state that is **not** history: current step, selection, hovered item, Joint tool draft, open info card, theme. All edits still go through the existing `historyReducer` actions (`merge`, `moveBody`, `setRoot`, `addJoint`, `updateJoint`, `acceptSuggestion`, `rejectSuggestion`, `removeJoint`, `setLinkProps`, `replaceBody`, `ignoreBody`, `renameLink`…). No new model actions are required.

## 4. Layout and visual style

- **Top:** stepper `1 Parts · 2 Group · 3 Connect · 4 Export`; each step shows ✓ (nothing left), or a count badge (⚠ 2). Right side: Import, Undo/Redo, Save, Open.
- **Left — Browser** (Fusion/Onshape pattern): collapsible sections Parts, Links, Joints; selecting a row selects in 3D and vice versa. **ChainView** pinned at the bottom.
- **Centre — viewport** (~70% width): soft gradient background, grid floor, **view cube** top-right (front/top/right/iso, home).
- **Right — step panel:** the current step's task as cards, one focus at a time.
- **Floating:** Joint tool (docked top-left of the viewport), info cards (anchored to the clicked object).

Tokens (CSS custom properties in `theme.css`):

| Token | Light | Dark |
|---|---|---|
| chrome | `#F4F4F5` | `#18181B` |
| panel | `#FFFFFF` | `#232326` |
| border | `#E4E4E7` | `#2E2E33` |
| text / text-muted | `#18181B` / `#71717A` | `#F4F4F5` / `#A1A1AA` |
| viewport gradient | `#ECEEF1 → #CDD2D8` | `#1F2023 → #2C2E33` |
| part | `#B9BDC3` satin, edge outlines | same |
| accent (selection, primary, fixed side) | `#2563EB` | `#3B82F6` |
| moving side | `#F59E0B` | same |
| danger / success | `#DC2626` / `#16A34A` | same |
| link tints (Group step only) | 8 soft hues at low saturation | same |

Type: Inter 13 px body, 600 headings, tabular numbers. Lucide line icons. 6 px radius, subtle shadow on floating panels. Dark theme follows `prefers-color-scheme`.

## 5. 3D view

- **Parts:** neutral satin grey with edge outlines. Hover brightens; selection gets an accent outline. Status never colours the whole part.
- **Problem pins:** red ⚠ marker floating above a part with a problem (failed, approximate-unresolved, unconnected at Export). Click → its problem card.
- **Failed part:** red wireframe box at the original bounding box (the part has no geometry).
- **Approximate part:** hatched overlay on the part until resolved (estimate accepted or mass set).
- **Group step:** each link tinted with its soft hue.
- **Joints:** axis arrow through the origin; revolute → arc with the limit wedge; prismatic → track with end stops; fixed → small lock glyph. The joint being edited animates on loop; fixed side tinted accent, moving side amber.
- **Unconnected link (Connect step):** pulsing outline + dashed line to its nearest touching neighbour, labelled "Not connected — click to connect".
- **Snap glyphs (Joint tool):** hovering a cylinder face shows a glyph at its centre with the axis drawn through it; a plane face shows a glyph at its centre with its normal. Built from `Body.faces` (`kind`, `center`, `axis`, `normal`) — no backend change. True edge snapping is out of scope (no B-rep edges in mesh data).
- **Limit handles:** two draggable handles on the revolute arc / prismatic track set lower/upper; number fields stay in sync.
- **Rendering on demand:** render only when something changed or an animation runs (replaces the continuous `requestAnimationFrame` loop, which starved the backend on CPU-only machines).
- **Info card anchoring:** the card is positioned at the object's projected screen point and follows the camera.

## 6. Steps

### 6.1 Parts — "check what came in"

- **Size check:** after import, a dimension box around the model: "1.42 m tall × 0.60 m × 0.55 m". Beside it the scale field and "Re-import at this scale" (STL/OBJ/BREP only).
- **Problem cards**, one per problem, each: plain sentence, technical term with ⓘ, one primary button, "Other options ▾":
  - Failed (`status: failed`): "This part has no usable shape." Primary **Replace with a file…**; other: Ignore it, How to re-export ⓘ.
  - Approximate: "This part's surface has gaps, so its weight is estimated ⓘ." Primary **Use the estimate** (sets `accept_approximate_mass` on its link); other: Enter the real mass, Replace.
  - Any other backend warning: its message with its listed fixes.
- Footer: "✓ N parts look fine."
- Replace uses the import scale (current behaviour) and resets the file input.

### 6.2 Group — "which parts move together"

- Title ⓘ: "A **link** is a set of parts bolted together that move as one piece."
- If small parts exist on entering the step the first time, the existing `mergeSmall` runs automatically and a notice says "We grouped N small parts (bolts, covers) into the parts they touch. Undo · Review". One undo step.
- Link cards: colour swatch, name (inline rename, validated live with the URDF name rule, ⓘ), part count, mass (live from `/link-mass`, with source "estimated from volume ⓘ" / "set by you").
- Regrouping: shift-click parts + **Group (G)**; or info card "Move to link…"; or drag a part row onto a link card.

### 6.3 Connect — "how the robot moves"

1. **Base:** "Click the part that sits on the ground ⚓", pre-filled with the lowest link; confirm or click another. (`setRoot`)
2. **Next joint:** `nextJoint(model)` returns the next pair to connect: a touching link pair (from `touchingPairs`) with exactly one side already connected to the base, preferring pairs with a detected suggestion, nearest to the base first. The Joint tool opens pre-filled for it.
3. **Joint tool (As-built Joint):**
   ```
   ① Fixed part   ② Moving part   ③ Joint origin (snap)
   Motion  [Rigid] [Revolute] [Slider] [Planar]   (Cylindrical, Ball greyed, ⓘ)
   Limits  lower ━●━━━━●━ upper   ☐ No limits (revolute → continuous)
   [▶ Animate] [⇄ Flip]  [Cancel] [OK]
   ```
   - Parts are picked in 3D (blue fixed, amber moving); fields fill as you pick; `Esc` steps back one field.
   - Mapping (ⓘ on each): Rigid → `fixed`; Revolute → `revolute`, or `continuous` with "No limits"; Slider → `prismatic`; Planar → `planar`. Cylindrical/Ball: greyed, ⓘ "URDF can't express this — use two joints".
   - Pre-filled from a suggestion: parts, origin, axis and motion, labelled "✨ Suggested from contact faces ⓘ" plus "Detector thinks: <type>". A detected `cylindrical` pre-selects Revolute; a detected `planar` pre-selects Planar but the animation makes it obvious to change.
   - **OK** is enabled only when: two parts picked, a motion type chosen, an axis set for non-Rigid types, and limits set (lower ≤ upper) for Revolute-with-limits and Slider. Accepting a suggestion uses `acceptSuggestion`; a new joint uses `addJoint` (`source: manual`); editing uses `updateJoint`.
   - After OK, the joint appears in 3D and in the ChainView; the panel offers the next joint. "Not connected" on a pair rejects its suggestion (`rejectSuggestion`).
4. **Manual connect:** **Joint (J)** opens an empty tool at any time; clicking a dashed "not connected" line opens it with both parts pre-filled.
5. Clicking an existing joint glyph or ChainView node reopens the tool on that joint (Remove available there). The tool's draft resyncs when the stored joint changes (undo/redo).

### 6.4 Export — "ready check"

- Checklist, live, each item clickable (jumps to the step and flies the camera):
  - every link connected to the base;
  - every link has a usable mass (no failed parts, approximate resolved);
  - link and robot names valid ⓘ;
  - every moving joint has an axis and, where needed, limits.
- **▶ Play all joints:** animates each joint through its range in chain order.
- Summary "7 links · 6 joints · 7 meshes", robot name field, **Export URDF (.zip)** (disabled with the first unmet item as its reason; disabled while a request is running).
- On a 422: pieces mapped to links/joints become checklist items and 3D pins, as in the first frontend.

## 7. Glossary (ⓘ texts, `glossary.ts`)

link, base link, joint, joint origin, axis, Rigid/fixed, Revolute/revolute, continuous, Slider/prismatic, Planar/planar, Cylindrical, Ball, limits, mass, inertia, estimated mass, approximate shape, failed part, URDF, mesh, scale. Each: one or two plain sentences; the URDF term named where it differs.

## 8. Names

Part names display with a middle ellipsis (`IRB1300…338203`) and the full name on hover. Link names stay the sanitized defaults until renamed; Group and the Joint tool offer inline rename ("Name this link: shoulder").

## 9. Carried over from the final review of the first frontend

Folded into the rewrite rather than fixed in the old components: link mass resets when the selected link changes and shows link names in errors; part info resets between parts; OK/Accept requires an axis for non-fixed types; Export disabled on invalid names and while running; joint draft resyncs after undo/redo.

## 10. Testing

- Unit (Vitest): `nextJoint` ordering (base-outward, prefers suggestions, skips connected); base guess (lowest link); Joint tool enable rules and type mapping; Export checklist items; glossary coverage (every ⓘ key exists).
- Component (Testing Library): each step's main path with a fake viewer; Joint tool pick → snap → motion → OK dispatches the right action; problem card buttons dispatch the right action.
- Viewer: pure helpers (snap point from face, limit handle ↔ value, card anchor projection) unit-tested; WebGL itself exercised by e2e.
- e2e (Playwright): peg-in-hole import → Connect (pre-filled suggestion, Revolute, limits, OK) → Export downloads zip.
- Manual: the ABB run in §1 by someone who hasn't used the app.

## 11. Out of scope

Command line, marking menus, timeline, true edge snapping, Cylindrical/Ball joints, moving parts into position (non-as-built joints), hosting, accounts.
