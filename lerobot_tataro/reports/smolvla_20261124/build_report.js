// Builds SmolVLA_TATARO_Performance_Report.docx from figs/ (run make_figures.py first).
//   NODE_PATH=<dir with node_modules/docx> node build_report.js
const fs = require("fs");
const path = require("path");
const {
  Document, Packer, Paragraph, TextRun, ImageRun, Table, TableRow, TableCell, WidthType, ShadingType,
  AlignmentType, HeadingLevel, LevelFormat, BorderStyle, Footer, PageNumber,
} = require("docx");

const REP = __dirname;
const FIG = path.join(REP, "figs");
const sizes = JSON.parse(fs.readFileSync(path.join(FIG, "sizes.json")));
const attempts = JSON.parse(fs.readFileSync(path.join(FIG, "arm_attempts.json")));
const FONT = "Arial";

const p = (text, opts = {}) => new Paragraph({ spacing: { after: 120 }, ...opts, children: runs(text) });
function runs(text) {
  return String(text).split(/(\*\*[^*]+\*\*)/).filter(Boolean).map((t) =>
    t.startsWith("**") ? new TextRun({ text: t.slice(2, -2), bold: true }) : new TextRun(t));
}
const h1 = (t) => new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun(t)] });
const h2 = (t) => new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun(t)] });
const bullet = (t) => new Paragraph({ numbering: { reference: "bullets", level: 0 }, spacing: { after: 60 }, children: runs(t) });
const num = (t) => new Paragraph({ numbering: { reference: "numbers", level: 0 }, spacing: { after: 60 }, children: runs(t) });

function img(file, widthPx, caption) {
  const [w, h] = sizes[file];
  const width = Math.min(widthPx, 600);
  return [new Paragraph({
    alignment: AlignmentType.CENTER, spacing: { before: 120, after: 60 }, keepNext: true,
    children: [new ImageRun({ type: file.endsWith(".png") ? "png" : "jpg", data: fs.readFileSync(path.join(FIG, file)),
      transformation: { width, height: Math.round(width * h / w) },
      altText: { title: caption, description: caption, name: file } })],
  }), new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 200 },
    children: [new TextRun({ text: caption, italics: true, size: 18, color: "555555" })] })];
}

const border = { style: BorderStyle.SINGLE, size: 4, color: "C9C8C2" };
const borders = { top: border, bottom: border, left: border, right: border };
function table(header, rows, widths, opts = {}) {
  const total = widths.reduce((a, b) => a + b, 0);
  const cell = (t, i, head, fill) => new TableCell({
    borders, width: { size: widths[i], type: WidthType.DXA },
    shading: head ? { fill: "E8EEF7", type: ShadingType.CLEAR } : fill ? { fill, type: ShadingType.CLEAR } : undefined,
    margins: { top: 50, bottom: 50, left: 90, right: 90 },
    children: [new Paragraph({ alignment: i > 0 && opts.right && opts.right.includes(i) ? AlignmentType.RIGHT : AlignmentType.LEFT,
      children: [new TextRun({ text: String(t), bold: head, size: opts.size || 19 })] })],
  });
  return new Table({
    width: { size: total, type: WidthType.DXA }, columnWidths: widths,
    rows: [new TableRow({ tableHeader: true, children: header.map((t, i) => cell(t, i, true)) }),
      ...rows.map((r) => new TableRow({ children: r.cells.map((t, i) => cell(t, i, false, r.fill)) }))],
  });
}
const T = (header, rows, widths, opts) => table(header, rows.map((r) => ({ cells: r })), widths, opts);
const gap = () => new Paragraph({ spacing: { after: 120 }, children: [] });

const OUTCOME = { success: "Success: cube in bin", missed: "Missed", c31: "Collision stop (C31)",
  guard: "Guard stop (would miss)", knocked: "Knocked cube away" };
const FILL = { success: "E3F4E3" };
const fmtTime = (run) => run.slice(9, 11) + ":" + run.slice(11, 13);
const attemptRows = attempts.map((r, i) => ({
  fill: FILL[r.outcome],
  cells: [i + 1, `${fmtTime(r.run)} ep${r.ep}`, r.colour, `(${Math.round(r.cube[0])}, ${Math.round(r.cube[1])})`,
    `(${Math.round(r.close[0])}, ${Math.round(r.close[1])}, ${Math.round(r.close[2])})`, r.err.toFixed(0),
    OUTCOME[r.outcome]],
}));
const nSucc = attempts.filter((r) => r.outcome === "success").length;
const errs = attempts.map((r) => r.err).sort((a, b) => a - b);
const median = (a) => (a.length % 2 ? a[(a.length - 1) / 2] : (a[a.length / 2 - 1] + a[a.length / 2]) / 2);
const t1 = attempts.filter((r) => Math.abs(r.cube[0] - 267) < 8 && Math.abs(r.cube[1] + 101) < 8);
const t1Succ = t1.filter((r) => r.outcome === "success").length;
const t1Med = median(t1.map((r) => r.err).sort((a, b) => a - b));

const children = [
  new Paragraph({ heading: HeadingLevel.TITLE, children: [new TextRun("SmolVLA on the TATARO Lite6")] }),
  new Paragraph({ spacing: { after: 60 }, children: [new TextRun({ text: "Performance summary: training, offline evaluation and real-arm tests", size: 26, color: "444444" })] }),
  new Paragraph({ spacing: { after: 240 }, border: { bottom: { style: BorderStyle.SINGLE, size: 6, color: "2A78D6", space: 6 } },
    children: [new TextRun({ text: "TATARO project · 24–25 November 2026 · checkpoint smolvla_tataro_v2fix, step 20,000", size: 20, color: "666666" })] }),

  h1("1. Summary"),
  p("SmolVLA (LeRobot, 450 M parameters) was fine-tuned to pick a red, green or blue cube named in a language instruction and drop it in the bin, using the TATARO Lite6 demonstration dataset. The model was then tested offline on held-out episodes and on the real arm."),
  bullet("**The task motion is learned.** In every real-arm episode the policy approached, descended, closed the gripper, lifted, carried to the bin and released, with smooth motion and no freezes."),
  bullet(`**Grasp accuracy is not yet good enough.** Over ${attempts.length} real-arm attempts on 24 Nov the gripper target was a median **${median(errs).toFixed(0)} mm** from the cube, and **${nSucc} of ${attempts.length}** attempts put the cube in the bin. A reliable grasp needs about 10 mm.`),
  bullet(`**Even on a training position it varies.** ${t1.length} attempts with the cube on demo position (266, −100) gave errors from ${Math.min(...t1.map((r) => r.err)).toFixed(0)} to ${Math.max(...t1.map((r) => r.err)).toFixed(0)} mm (median ${t1Med.toFixed(0)} mm), ${t1Succ} successes.`),
  bullet("**It works in one area.** On 25 Nov the cube at (268, 35) and (260, −30) was picked 2 out of 2 times, but the best-covered training positions (279, −123), (285, 30) and (350, 6) went 0 for 10, aimed 20–51 mm off."),
  bullet("**Offline numbers were optimistic.** On the validation split the grasp point was 5–10 mm from the demo grasp in the central zone, but those episodes share positions and recording sessions with the training data."),
  bullet("**Likely cause: weak cube localisation.** The demos cover 82 cube positions with a median of 3 episodes each (recoloured copies included). The policy grasps near one preferred area around x ≈ 255–270 mm and only partly follows the cube, even where 5–7 training episodes exist. The frozen vision encoder (cube about 10 px in the table view) is a prime suspect."),
  bullet("**Next step:** retrain with the vision encoder unfrozen and compare with ACT (trainable image encoder); then more data or sim co-training. The colour detector plus a scripted grasp is a reliable baseline meanwhile."),
  gap(),
  T(["Measure", "Result"], [
    ["Training loss (20k steps)", "1.40 → 0.017"],
    ["Offline grasp error, central zone (median)", "4.7–9.9 mm"],
    ["Offline grasp error, rest of workspace (median)", "9.5–19 mm"],
    ["Inference time per 50-step action chunk", "138 ms (RTX 4060 laptop GPU)"],
    [`Real arm 24 Nov, all attempts (n = ${attempts.length})`, `median ${median(errs).toFixed(0)} mm from cube; ${nSucc} successes (${Math.round(100 * nSucc / attempts.length)} %)`],
    [`Real arm, demo position (266, −100) (n = ${t1.length})`, `median ${t1Med.toFixed(0)} mm; ${t1Succ} successes`],
    ["Real arm 25 Nov, preferred area (268, 35) / (260, −30)", "2 of 2 green successes (3 and 12 mm); 1 red guard stop (21 mm)"],
    ["Real arm 25 Nov, best-covered training positions", "0 of 10 (20–51 mm off)"],
  ], [5200, 3826]),

  h1("2. Setup"),
  T(["Item", "Details"], [
    ["Robot", "UFactory Lite6 (6-DOF) with Gripper Lite (about 40 mm opening); Cartesian TCP control via the lerobot_tataro plugin"],
    ["Cameras", "Intel RealSense D455 overhead (warped on ArUco markers 51/52/53 into a 336 × 336 top-down table view) and D435i on the wrist"],
    ["Model", "lerobot/smolvla_base (SmolVLM2-500M backbone, frozen vision encoder, flow-matching action expert, 50-step action chunks); 100 M of 450 M parameters trained"],
    ["Inputs / outputs", "2 camera images + 13-dim state (6 joints, TCP pose, gripper) + task text → absolute TCP target (x, y, z, rotation) + gripper"],
    ["Task text", "“Pick up the {red|green|blue} cube and place it in the bin”"],
    ["Data (v2fix)", "218 train / 25 val episodes (149,115 / 17,537 frames at 30 fps): real hand-guided demos plus recoloured synthetic copies; 82 distinct cube positions, median 3 episodes per position"],
    ["Training", "batch 16, 20,000 steps (about 2.1 epochs), colour-jitter augmentation, 3 h 25 min on an RTX 4060 (8 GB)"],
  ], [1900, 7126]),

  h1("3. Training and data fix"),
  p("Two runs were trained with the same recipe. **v2** used the original table-view dataset. Checking it revealed that 18 recoloured training episodes still showed the **blue** cube under a “red” or “green” label: the recolouring had missed the pale, shaded and fringe pixels of the blue cube. **v2fix** retrained on data repaired by scripts/fix_recolor.py."),
  T(["Training split: overhead cube colour vs label", "v2 (original)", "v2fix (repaired)"], [
    ["Labelled red, cube looks red", "71", "79"],
    ["Labelled green, cube looks green", "67", "75"],
    ["Labelled red or green, cube still blue", "18", "2"],
    ["Episodes rewritten by the fix", "–", "122 of 218"],
  ], [4826, 2100, 2100], { right: [1, 2] }),
  ...img("fig1_loss.png", 560, "Figure 1. Training loss of both runs (log scale). The curves overlap; both end at about 0.017."),

  h1("4. Offline evaluation (no robot)"),
  p("For each validation episode the policy was run on the recorded frames 1.5, 1.0 and 0.5 s before the demonstrator closed the gripper, and the point where its predicted chunk closes the gripper was compared with the real grasp. The central zone is the 250 × 250 mm square around (280, −60) mm where the zone recording was made."),
  ...img("fig2_offline.png", 560, "Figure 2. Median grasp-point error on the validation split, 20k checkpoints."),
  T(["Comparison (20k checkpoint, val split)", "v2", "v2fix"], [
    ["Policy grasp vs detected cube, median", "12–13 mm", "10–13 mm"],
    ["Policy grasp vs detected cube, worst", "about 30 mm", "26 mm"],
    ["5k checkpoint, central zone (1.5 / 1.0 / 0.5 s)", "18 / 15 / 26 mm", "13 / 14 / 16 mm"],
    ["Inference, median per chunk", "149 ms", "138 ms"],
  ], [4826, 2100, 2100]),
  ...img("photo_offline_viz.jpg", 480, "Figure 3. Policy predictions on validation episodes. Yellow cross = detected cube, white circle = demo grasp, lines/dots = predicted chunk and grasp point 1.5 s (blue), 1.0 s (green) and 0.5 s (red) before the grasp. Top: good cases (3–8 mm). Bottom: the cube next to the gripper at HOME and a far corner (25–30 mm)."),

  h1("5. Real-arm tests"),
  h2("5.1 Method"),
  p("All tests used scripts/run_policy.py at a 0.25 m/s speed limit. The test harness added:"),
  bullet("**Placement view:** live overhead image marking target cube positions (demo positions or custom ones), so the cube can be placed within a few mm."),
  bullet("**Live view:** overhead and wrist cameras with the policy's current target (blue ring, red dot when closing) and the detected cube (yellow cross)."),
  bullet("**Success check:** overhead image at HOME before and after each episode → success / missed / moved / unknown."),
  bullet("**Descent guard** (from attempt 17): below 40 mm the policy target must be within 20 mm of the detected cube, otherwise the arm holds and the episode ends. This prevents the collision stops described in 5.4."),
  h2("5.2 Results, 24 November"),
  ...img("fig3_arm_attempts.png", 580, "Figure 4. Distance between the gripper target and the cube when the gripper closed (or when the guard stopped the arm), per attempt."),
  table(["#", "Run", "Cube", "Cube (x, y) mm", "Gripper (x, y, z) mm", "Error mm", "Outcome"], attemptRows,
    [450, 1250, 750, 1500, 1800, 900, 2376], { size: 17, right: [5] }),
  p("Outcomes were taken from the check images, videos and logs. Attempt 10 was a success that the automatic check called “unknown” (the cube looked grey under the light); the check has since been fixed.", { spacing: { before: 120, after: 120 } }),
  ...img("fig4_map.png", 440, "Figure 5. Overhead table view: the 82 demo cube positions (white), test cube positions (yellow +) and where the gripper went (coloured by outcome). The grasps cluster around x ≈ 250–280 mm, near the average demo position (star), wherever the cube was."),
  h2("5.3 Follow-up, 25 November"),
  T(["Target (mm)", "Training episodes there", "Attempts", "Result"], [
    ["(268, 35)", "–", "1 green", "Success, 3 mm"],
    ["(260, −30)", "–", "1 green, 1 red", "Green success (12 mm); red guard stop (21 mm)"],
    ["(285, 30)", "5", "2", "Guard stops, 33 and 47 mm off"],
    ["(350, 6)", "5", "2", "Guard stop 51 mm; one closed ~60 mm away and pushed the cube"],
    ["(279, −123)", "7", "6", "4 guard stops (21–33 mm); 2 closed 15–16 mm off and stopped (likely C31)"],
  ], [1500, 1700, 1700, 4126]),
  p("More training episodes at a position did not help: the policy succeeds where its preferred grasp area happens to be and misses elsewhere.", { spacing: { before: 120, after: 120 } }),
  h2("5.4 A successful episode"),
  ...img("photo_success_overhead.jpg", 600, "Figure 6. 24 Nov attempt 1, overhead table view (annotated replay). The gripper closes 9 mm from the cube, lifts, carries and releases the cube into the bin."),
  ...img("photo_success_wrist.jpg", 600, "Figure 7. The same episode from the wrist camera."),
  ...img("photo_checks.jpg", 460, "Figure 8. Automatic success check, before (left) and after (right) at HOME. Top: success (attempt 1). Bottom: missed (attempt 2): the cube has not moved."),
  h2("5.5 Failure modes"),
  bullet("**Wrong position (most common).** The policy aims near its preferred area and only partly follows the cube; at (278, −70) it closed about 47 mm away three times in a row."),
  bullet("**Fingertip on the cube → collision stop (C31).** With a 15–35 mm error, one finger lands on top of the cube and the arm pushes down. The controller's collision detection stops the arm (error C31). Six attempts ended this way before the descent guard was added; two more at 15–16 mm (inside the 20 mm guard limit) on 25 Nov, so the guard limit should be about 14 mm."),
  bullet("**Right place, wrong height.** Attempt 9 was 3 mm from the cube but closed at z = 20 mm instead of the usual 11–17 mm and slipped off the top."),
  bullet("**Cube knocked away.** In attempt 12 a finger hit the cube edge and flicked it about 10 cm across the table."),
  ...img("photo_grasp_compare.jpg", 520, "Figure 9. Moment of closing. Left: 9 mm from the cube (success). Right: 17 mm off, a finger on the cube (C31 stop)."),
  ...img("photo_knocked.jpg", 600, "Figure 10. Attempt 12: finger lands on the cube edge, the gripper carries nothing, the cube lies about 10 cm away."),

  h1("6. Conclusions and next steps"),
  p("SmolVLA learned the complete pick-and-place sequence and runs fast enough for real-time control (138 ms per 50-step chunk). Its weakness is locating the cube precisely: errors of 15–45 mm are common, about three times what the 40 mm gripper tolerates. The evidence:"),
  bullet("The grasps cluster in one area whatever the cube position, and positions with 5–7 training episodes fail as badly as sparsely covered ones."),
  bullet("Large spread between repeated attempts at an identical position (3–37 mm)."),
  bullet("The parts that are the same in every demonstration (lift, carry, release) are executed reliably."),
  p("Recommended plan:", { spacing: { before: 120, after: 60 } }),
  num("**Unfreeze the vision encoder** (SigLIP, about 90 M parameters; fits on the 8 GB GPU) and fine-tune from the current checkpoint at a lower learning rate; compare with ACT, whose ResNet encoder is trained."),
  num("**Pilot recording:** about 10 demos at each of 3 positions, e.g. (266, −100), (227, −109), (344, −95), with ±1–2 cm shifts and varied cube rotation, under current lighting."),
  num("**If the success rate rises clearly (≥ 80 %):** record a ~5 cm grid over the reachable table, 3–5 demos per cell (about 150–250 demos)."),
  num("**If not:** try a higher-resolution or cropped table view, or sim co-training."),
  num("**Baseline:** for single-cube colour sorting, cube_detector.py (median 7 mm localisation) plus a scripted grasp is already reliable and can serve demonstrations meanwhile."),

  h1("Appendix: files and videos"),
  p("Videos cannot be played inside this document. All run outputs (annotated videos, wrist videos, state CSVs, check images) are in the dataset repository under policy_runs/:"),
  T(["What", "Location"], [
    ["Run outputs, 24–25 Nov (videos, CSV, check images)", "github.com/Pooja15V/TATARO_LeRobot_Dataset → policy_runs/smolvla_v2fix_20k/"],
    ["Best episodes", "…/20261124_151355/ep0_annotated.mp4 (9 mm), …/20261125_091148/ep1_annotated.mp4 (3 mm)"],
    ["Offline prediction images", "~/lerobot_outputs/smolvla_tataro_v2fix/checkpoints/020000/grasp_viz_val/ (laptop)"],
    ["Checkpoint", "~/lerobot_outputs/smolvla_tataro_v2fix/checkpoints/020000/pretrained_model (laptop)"],
    ["Code", "github.com/Pooja15V/TATARO_FirmwareV1.0 → lerobot_tataro/scripts (run_policy.py, replay_run.py, cube_detector.py, eval_grasp_point.py, viz_policy_grasp.py, fix_recolor.py)"],
    ["Data", "github.com/Pooja15V/TATARO_LeRobot_Dataset → tataro/record_tataro_v2fix_split"],
    ["Figures for this report", "lerobot_tataro/reports/smolvla_20261124/ (make_figures.py, build_report.js)"],
  ], [3000, 6026], { size: 17 }),
];

const doc = new Document({
  creator: "TATARO project", title: "SmolVLA on the TATARO Lite6: performance summary",
  styles: {
    default: { document: { run: { font: FONT, size: 21 } } },
    paragraphStyles: [
      { id: "Title", name: "Title", basedOn: "Normal", run: { font: FONT, size: 44, bold: true, color: "1A1A19" },
        paragraph: { spacing: { before: 0, after: 80 }, keepNext: true } },
      { id: "Heading1", name: "Heading 1", basedOn: "Normal", next: "Normal", quickFormat: true,
        run: { font: FONT, size: 30, bold: true, color: "1C5CAB" }, paragraph: { spacing: { before: 320, after: 140 }, keepNext: true, outlineLevel: 0 } },
      { id: "Heading2", name: "Heading 2", basedOn: "Normal", next: "Normal", quickFormat: true,
        run: { font: FONT, size: 24, bold: true, color: "222220" }, paragraph: { spacing: { before: 220, after: 100 }, keepNext: true, outlineLevel: 1 } },
    ],
  },
  numbering: { config: [
    { reference: "bullets", levels: [{ level: 0, format: LevelFormat.BULLET, text: "•", alignment: AlignmentType.LEFT,
      style: { paragraph: { indent: { left: 540, hanging: 270 } } } }] },
    { reference: "numbers", levels: [{ level: 0, format: LevelFormat.DECIMAL, text: "%1.", alignment: AlignmentType.LEFT,
      style: { paragraph: { indent: { left: 540, hanging: 300 } } } }] },
  ] },
  sections: [{
    properties: { page: { size: { width: 11906, height: 16838 }, margin: { top: 1300, bottom: 1300, left: 1440, right: 1440 } } },
    footers: { default: new Footer({ children: [new Paragraph({ alignment: AlignmentType.RIGHT,
      children: [new TextRun({ text: "SmolVLA on the TATARO Lite6 · page ", size: 16, color: "888888" }),
        new TextRun({ children: [PageNumber.CURRENT], size: 16, color: "888888" })] })] }) },
    children,
  }],
});

Packer.toBuffer(doc).then((buf) => {
  const out = path.join(REP, "SmolVLA_TATARO_Performance_Report.docx");
  fs.writeFileSync(out, buf);
  console.log("wrote", out);
});
