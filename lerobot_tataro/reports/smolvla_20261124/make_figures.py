"""Figures + photo montages for the SmolVLA TATARO report (2026-11-24)."""
import json
import re
import sys
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
FIG = HERE / "figs"
FIG.mkdir(exist_ok=True)
sys.path.insert(0, str(HERE.parents[1] / "scripts"))
import cube_detector as cd  # noqa: E402

OUT = Path.home() / "lerobot_outputs"
RUNS = OUT / "policy_runs"
BLUE, ORANGE = "#2a78d6", "#eb6834"
GOOD, CRIT, SER, GREY = "#0ca30c", "#d03b3b", "#ec835a", "#8a8a85"
INK, INK2 = "#222220", "#6b6a64"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "axes.edgecolor": "#c9c8c2",
                     "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2,
                     "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True,
                     "grid.color": "#ecebe6", "grid.linewidth": 0.8, "axes.axisbelow": True})


def save(fig, name):
    fig.savefig(FIG / name, dpi=200, bbox_inches="tight", facecolor="white")
    plt.close(fig)


# ---------- 1. training loss ----------
def loss(log):
    pts = []
    for m in re.finditer(r"step:([0-9.]+)(K?) smpl:\S+ ep:\S+ epch:\S+ loss:([0-9.]+)", (OUT / log).read_text(errors="ignore")):
        s = float(m.group(1)) * (1000 if m.group(2) else 1)
        pts.append((s, float(m.group(3))))
    return np.array(pts)


a, b = loss("smolvla_tataro_v2.log"), loss("smolvla_tataro_v2fix.log")
fig, ax = plt.subplots(figsize=(6.5, 3.0))
ax.plot(a[:, 0], a[:, 1], color=BLUE, lw=2, label="v2 (original data)")
ax.plot(b[:, 0], b[:, 1], color=ORANGE, lw=2, label="v2fix (recolour-fixed data)")
ax.set_yscale("log")
ax.set_xlabel("training step")
ax.set_ylabel("training loss (log scale)")
ax.legend(frameon=False)
ax.annotate(f"final {b[-1, 1]:.3f}", (b[-1, 0], b[-1, 1]), xytext=(-60, 18), textcoords="offset points", color=INK2,
            arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
save(fig, "fig1_loss.png")

# ---------- 2. offline grasp-point error ----------
H = ["1.5 s", "1.0 s", "0.5 s"]
data = {  # median |policy grasp - demo grasp| mm, val split, 20k checkpoints (order 1.5/1.0/0.5 s before grasp)
    ("zone", "v2"): [8.1, 7.8, 6.5], ("zone", "v2fix"): [9.9, 4.7, 6.4],
    ("outside", "v2"): [12, 12, 19.5], ("outside", "v2fix"): [9.5, 14.9, 19.0]}
fig, axs = plt.subplots(1, 2, figsize=(6.5, 2.8), sharey=True)
x = np.arange(3)
for ax, reg, title in zip(axs, ["zone", "outside"], ["Central zone (6 val episodes)", "Rest of workspace (19)"]):
    for k, (m, c) in enumerate([("v2", BLUE), ("v2fix", ORANGE)]):
        v = data[(reg, m)]
        bars = ax.bar(x + (k - 0.5) * 0.36, v, 0.34, color=c, label=m, edgecolor="white", linewidth=1)
        for r, val in zip(bars, v):
            ax.text(r.get_x() + r.get_width() / 2, val + 0.4, f"{val:g}", ha="center", va="bottom", fontsize=8,
                    color=INK2)
    ax.set_xticks(x, H)
    ax.set_xlabel("time before the demo grasp")
    ax.set_title(title, fontsize=10, color=INK)
axs[0].set_ylabel("median error vs demo grasp [mm]")
axs[0].legend(frameon=False, loc="upper left")
save(fig, "fig2_offline.png")

# ---------- 3. real-arm attempts ----------
rows = json.loads(Path(sys.argv[1]).read_text()) if len(sys.argv) > 1 else []
OUTC = {"success": (GOOD, "o", "success"), "missed": (CRIT, "X", "missed"), "c31": (SER, "^", "collision stop (C31)"),
        "guard": (SER, "s", "guard stop (would miss)"), "knocked": (CRIT, "D", "knocked cube away")}
fig, ax = plt.subplots(figsize=(6.5, 3.2))
seen = set()
for i, r in enumerate(rows):
    c, mk, lab = OUTC[r["outcome"]]
    ax.scatter(i + 1, r["err"], s=60, color=c, marker=mk, edgecolor="white", linewidth=1,
               label=None if lab in seen else lab, zorder=3)
    seen.add(lab)
ax.axhspan(0, 10, color=GOOD, alpha=0.08, lw=0)
ax.axhline(10, color=GOOD, lw=0.8, ls="--")
ax.text(0.45, 7.5, "under ~10 mm: cube is grasped", color=INK2, fontsize=8, ha="left", va="top")
ax.set_xlim(0.3, len(rows) + 0.7)
ax.set_ylim(0, 52)
ax.set_xticks(range(1, len(rows) + 1))
ax.tick_params(axis="x", labelsize=7)
ax.set_xlabel("attempt (chronological, 24 Nov 2026)")
ax.set_ylabel("gripper target vs cube [mm]")
ax.legend(frameon=False, fontsize=8, ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.2))
save(fig, "fig3_arm_attempts.png")

# ---------- 4. map: demos, cubes, grasps ----------
bg = cv2.cvtColor(cv2.imread(str(cd.BG)), cv2.COLOR_BGR2RGB)
demo = np.load(FIG / "demo_positions.npy")[:, :2]  # 82 distinct demo cube positions (20 mm clusters)
fig, ax = plt.subplots(figsize=(5.2, 5.2))
ax.imshow(bg, alpha=0.75)
ax.grid(False)
ax.set_xticks([])
ax.set_yticks([])
p = cd.to_table(demo)
ax.scatter(p[:, 0], p[:, 1], s=22, color="white", edgecolor=INK, linewidth=0.8, label="demo cube positions (82)", zorder=3)
mean = cd.to_table([demo.mean(0)])[0]
ax.scatter(*mean, s=160, marker="*", color="black", edgecolor="white", linewidth=1, zorder=5,
           label="average demo position")
for r in rows:
    c, mk, _ = OUTC[r["outcome"]]
    cu, gr = cd.to_table([r["cube"]])[0], cd.to_table([r["close"][:2]])[0]
    ax.plot([cu[0], gr[0]], [cu[1], gr[1]], color=c, lw=1, alpha=0.8, zorder=4)
    ax.scatter(*gr, s=30, color=c, marker=mk, edgecolor="white", linewidth=0.6, zorder=6)
cubes = np.array([cd.to_table([r["cube"]])[0] for r in rows])
ax.scatter(cubes[:, 0], cubes[:, 1], s=90, marker="+", color="#eda100", linewidth=2, zorder=7, label="test cube position")
ax.scatter([], [], s=30, color=GREY, marker="o", label="where the gripper went (colour = outcome)")
ax.legend(frameon=True, fontsize=7.5, loc="upper left", framealpha=0.9)
save(fig, "fig4_map.png")
print("figures done")


# ---------- photos ----------
def frames(video, ts, csv):
    d = pd.read_csv(csv)
    cap = cv2.VideoCapture(str(video))
    out = []
    for t in ts:
        i = int((d.t - t).abs().argmin())
        cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        ok, f = cap.read()
        out.append(f)
    return out


def label(img, txt, scale=0.8):
    img = img.copy()
    y = img.shape[0] - 14
    cv2.putText(img, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 6)
    cv2.putText(img, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 2)
    return img


def grid(imgs, cols, w):
    imgs = [cv2.resize(i, (w, int(w * i.shape[0] / i.shape[1]))) for i in imgs]
    h = max(i.shape[0] for i in imgs)
    imgs = [cv2.copyMakeBorder(i, 0, h - i.shape[0], 0, 0, cv2.BORDER_CONSTANT, value=(255, 255, 255)) for i in imgs]
    imgs += [np.full_like(imgs[0], 255)] * (-len(imgs) % cols)
    pad = lambda i: cv2.copyMakeBorder(i, 4, 4, 4, 4, cv2.BORDER_CONSTANT, value=(255, 255, 255))
    return np.vstack([np.hstack([pad(i) for i in imgs[k:k + cols]]) for k in range(0, len(imgs), cols)])


R = RUNS / "20261124_151355"
ts = [(0.5, "t=0 s  HOME"), (5.0, "t=5 s  approach"), (6.6, "t=6.6 s  grasp"), (12.0, "t=12 s  lift"),
      (16.0, "t=16 s  carry"), (20.0, "t=20 s  release")]
ov = frames(R / "ep0_annotated.mp4", [t for t, _ in ts], R / "ep0.csv")
wr = frames(R / "ep0_wrist.mp4", [t for t, _ in ts], R / "ep0.csv")
cv2.imwrite(str(FIG / "photo_success_overhead.jpg"), grid([label(f, s) for f, (_, s) in zip(ov, ts)], 3, 420))
cv2.imwrite(str(FIG / "photo_success_wrist.jpg"), grid([label(f, s) for f, (_, s) in zip(wr, ts)], 3, 420))

V = OUT / "smolvla_tataro_v2fix/checkpoints/020000/grasp_viz_val"
cv2.imwrite(str(FIG / "photo_offline_viz.jpg"),
            grid([cv2.imread(str(V / f"ep{e:03d}.png")) for e in (3, 17, 12, 21)], 2, 420))
cv2.imwrite(str(FIG / "photo_checks.jpg"), grid([cv2.imread(str(RUNS / "20261124_151355/ep0_check.png")),
                                                 cv2.imread(str(RUNS / "20261124_151355/ep1_check.png"))], 1, 860))
cv2.imwrite(str(FIG / "photo_grasp_compare.jpg"), grid([cv2.imread(str(RUNS / "20261124_151355/ep0_grasp.png")),
                                                        cv2.imread(str(RUNS / "20261124_153122/ep0_grasp.png"))], 2, 420))
R3 = RUNS / "20261124_154818"
k = [(9.5, "t=9.5 s finger on cube"), (14.0, "t=14 s carrying nothing"), (17.0, "t=17 s cube knocked away")]
cv2.imwrite(str(FIG / "photo_knocked.jpg"),
            grid([label(f, s, 0.6) for f, (_, s) in zip(frames(R3 / "ep3_overhead.mp4", [t for t, _ in k], R3 / "ep3.csv"), k)],
                 3, 420))
print("photos done")
