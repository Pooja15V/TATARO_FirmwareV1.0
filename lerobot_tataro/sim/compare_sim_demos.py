#!/usr/bin/env python3
"""Build the TATARO MuJoCo scene (rehab table, markers 51/52/53, Lite6 + Gripper Lite, wrist D435i, overhead D455,
red 3 cm cube, bowl) and compare it with the real recorded demos.

    python3 compare_sim_demos.py      # system python3 (has mujoco); needs demo_reference.npz from export_demo_reference.py

Writes next to this file:
  tataro_scene.xml        the scene (loadable on its own: mujoco.MjModel.from_xml_path)
  sim_vs_demos.png        sim overhead render | real overhead frame, a red dot on every demo's cube position
  sim_overhead_nominal.png / sim_wrist_home.png   the spec's nominal overhead camera / the calibrated wrist camera
  sim_vs_demos.txt        the numbers: HOME pose, markers, bowl, overhead camera, sim vs real

Coordinates: link_base (robot) mm, as in the dataset. World <-> base:  Xb = 0.308 - Yw,  Yb = Xw - 0.0095,  Zb = Zw - 0.935.
"""

from __future__ import annotations

import os
import re
import sys
from pathlib import Path

if Path("/proc/driver/nvidia/version").exists():  # render on the RTX 4060 (see sim_backend/simulation.py)
    os.environ.setdefault("__NV_PRIME_RENDER_OFFLOAD", "1")
    os.environ.setdefault("__GLX_VENDOR_LIBRARY_NAME", "nvidia")

import cv2
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(Path.home() / "TATARO_Reasoning_Execution"))
from sim_backend.simulation import PREFIX, build_scene_xml  # noqa: E402  same patches as the web app's sim

BASE = np.array([0.0095, 0.308, 0.935])
HOME_Q_DATASET = np.array([-0.0052, -0.2059, 0.8203, 0.0, 1.0263, -0.0035])  # ee = (200, 0, 250) mm
HOME_EE_DATASET = np.array([200.0, 0.0, 250.0])
ZONE_C, ZONE_HALF = np.array([280.0, -60.0]), 125.0
BOWL = np.array([128.0, -270.0])  # every demo releases here (z 60 mm)
BOWL_R = 55.0
TABLE_TOP_W = 0.930
W, H = 640, 480
F_OVERHEAD = 387.0  # px at 640x480: matches the real marker spacing at the spec height (D455 colour, 4:3 mode)
CACHE = HERE.parent / "datasets" / "tataro" / ".place_cube_cache.npz"  # cube001 demos: cube px (table) + grasp mm


def fit_overhead(ref):
    """Real overhead camera pose (link_base) from the demos: cube pixel <-> grasp (x, y, cube top 15 mm)."""
    z = np.load(CACHE)
    raw = cv2.perspectiveTransform(z["tab"].reshape(-1, 1, 2).astype(np.float32),
                                   np.linalg.inv(ref["H_cube001"])).reshape(-1, 2).astype(np.float64)
    obj = np.c_[z["grasp"], np.full(len(raw), 15.0)] / 1000
    K = np.array([[F_OVERHEAD, 0, W / 2], [0, F_OVERHEAD, H / 2], [0, 0, 1.0]])
    _, rv, tv, inl = cv2.solvePnPRansac(obj, raw, K, None, reprojectionError=8)
    i = inl.ravel()
    rv, tv = cv2.solvePnPRefineLM(obj[i], raw[i], K, None, rv, tv)
    err = np.linalg.norm(cv2.projectPoints(obj, rv, tv, K, None)[0].reshape(-1, 2) - raw, axis=1)
    R, _ = cv2.Rodrigues(rv)
    return K, R, -R.T @ tv.ravel(), len(i), len(raw), float(np.median(err[i]))


B2W = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.0]])  # rotation link_base -> world


def base_to_world(p_mm):
    p = np.asarray(p_mm, float).reshape(-1, 3) / 1000
    return np.c_[BASE[0] + p[:, 1], BASE[1] - p[:, 0], BASE[2] + p[:, 2]]


def world_to_base(p):
    p = np.asarray(p, float).reshape(-1, 3)
    return np.c_[BASE[1] - p[:, 1], p[:, 0] - BASE[0], p[:, 2] - BASE[2]] * 1000


def scene_xml(cam=None) -> str:
    text = build_scene_xml(n_cubes=0)
    text = re.sub(r'<body name="stack_base".*?</body>', "", text, count=1, flags=re.S)  # web-app stack pad: not real
    for cname in ("d455_overhead_color", "d455_overhead_depth"):
        text = text.replace(f'<camera name="{cname}"', f'<camera name="{cname}" resolution="{W} {H}"', 1)
    # wrist camera at the calibrated hand-eye pose (tataro_handeye_11, link_eef -> colour optical frame)
    eef = re.search(rf'<body name="{PREFIX}link_eef"[^>]*>', text)
    text = text[:eef.end()] + (
        '<body name="tataro_handeye_11_optical_frame" pos="0.0644231 -0.0296551 0.0270531" '
        'quat="0.6996711 0.0085666 -0.0026546 0.7144088">'
        f'<camera name="wrist" pos="0 0 0" quat="0 1 0 0" fovy="43.3" resolution="{W} {H}"/></body>') + text[eef.end():]
    cx, cy, cz = base_to_world([*ZONE_C, 15.0])[0]
    bx, by, _ = base_to_world([*BOWL, 0.0])[0]
    objs = (
        f'<body name="cube_red" pos="{cx:.4f} {cy:.4f} {TABLE_TOP_W + 0.015:.4f}"><freejoint name="cube_red_joint"/>'
        '<inertial mass="0.028" pos="0 0 0" diaginertia="4.2e-06 4.2e-06 4.2e-06"/>'
        '<geom name="cube_red_geom" type="box" size="0.015 0.015 0.015" rgba="0.92 0.18 0.18 1" '
        'friction="0.85 0.01 0.005" condim="4"/></body>'
        f'<body name="target_bowl" pos="{bx:.4f} {by:.4f} {TABLE_TOP_W + 0.004:.4f}">'
        f'<geom name="bowl_base" type="cylinder" size="{BOWL_R / 1000} 0.004" rgba="0.25 0.28 0.32 1"/></body>')
    if cam is not None:  # calibrated overhead camera "overhead" (the export's d455_overhead_color = nominal spec)
        _, R, C = cam
        cpos = BASE + B2W @ C
        x_w, y_w = B2W @ (R.T @ [1, 0, 0]), B2W @ (R.T @ [0, -1, 0])  # MuJoCo cam: x right, y up
        fovy = 2 * np.degrees(np.arctan((H / 2) / F_OVERHEAD))
        objs += (f'<camera name="overhead" pos="{cpos[0]:.4f} {cpos[1]:.4f} {cpos[2]:.4f}" '
                 f'xyaxes="{x_w[0]:.5f} {x_w[1]:.5f} {x_w[2]:.5f} {y_w[0]:.5f} {y_w[1]:.5f} {y_w[2]:.5f}" '
                 f'fovy="{fovy:.2f}" resolution="{W} {H}"/>')
    return text.replace("</worldbody>", objs + "</worldbody>", 1)


def project(model, data, cam_name, pts_world):
    """World points -> pixel (u, v) of a MuJoCo camera (looks along -z, +y up)."""
    cid = model.camera(cam_name).id
    pos, R = data.cam_xpos[cid], data.cam_xmat[cid].reshape(3, 3)
    pc = (np.asarray(pts_world) - pos) @ R  # camera coords
    f = (H / 2) / np.tan(np.radians(model.cam_fovy[cid]) / 2)
    return np.c_[W / 2 + f * pc[:, 0] / -pc[:, 2], H / 2 - f * pc[:, 1] / -pc[:, 2]]


def main():
    ref = np.load(HERE / "demo_reference.npz")
    K, Rc, Cc, n_in, n_all, rep = fit_overhead(ref)
    xml = scene_xml((K, Rc, Cc))
    (HERE / "tataro_scene.xml").write_text(xml)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    for i, q in enumerate(HOME_Q_DATASET):
        data.qpos[model.jnt_qposadr[model.joint(f"{PREFIX}joint{i + 1}").id]] = q
    mujoco.mj_forward(model, data)
    lines = []

    def say(s=""):
        print(s)
        lines.append(s)

    # ---- 1. HOME pose (forward kinematics vs the dataset's HOME TCP)
    tcp = world_to_base(data.site("tcp").xpos)[0]
    say("1. HOME pose (dataset joint angles -> sim TCP, link_base mm)")
    say(f"   sim TCP ({tcp[0]:.1f}, {tcp[1]:.1f}, {tcp[2]:.1f})  vs  dataset ee ({HOME_EE_DATASET[0]:.0f}, "
        f"{HOME_EE_DATASET[1]:.0f}, {HOME_EE_DATASET[2]:.0f})  -> off by {np.linalg.norm(tcp - HOME_EE_DATASET):.1f} mm")

    # ---- 2. markers: sim body centres vs real (raw overhead px -> table px -> robot mm, map fitted on demos)
    def raw_to_robot(px, name="zone"):
        t = cv2.perspectiveTransform(np.float32(px).reshape(-1, 1, 2), ref[f"H_{name}"])
        return cv2.perspectiveTransform(t, ref["T2R"]).reshape(-1, 2)

    say("\n2. ArUco marker centres (link_base mm; real = detected in the 2026-11-10 overhead frame, mapped with the "
        "table->robot fit on the demos)")
    sim_m, real_m = {}, {}
    for mid in (51, 52, 53):
        sim_m[mid] = world_to_base(data.body(f"aruco_marker_{mid}").xpos)[0][:2]
        real_m[mid] = raw_to_robot(ref[f"m{mid}_zone"].mean(0, keepdims=True))[0]
        d = real_m[mid] - sim_m[mid]
        say(f"   {mid}: sim ({sim_m[mid][0]:6.1f}, {sim_m[mid][1]:6.1f})  real ({real_m[mid][0]:6.1f}, "
            f"{real_m[mid][1]:6.1f})  diff ({d[0]:+5.1f}, {d[1]:+5.1f}) = {np.linalg.norm(d):.0f} mm")
    say(f"   (that map is fitted on cube positions x -64..442 mm and extrapolated to the far markers; better estimate:)")
    for mid in (51, 52, 53):  # ray from the fitted overhead camera through the marker centre, hit the table
        px = ref[f"m{mid}_cube001"].mean(0)
        ray = Rc.T @ np.linalg.inv(K) @ [px[0], px[1], 1.0]
        p = (Cc + ray * (-0.0044 - Cc[2]) / ray[2]) * 1000
        real_m[mid] = p[:2]
        d = real_m[mid] - sim_m[mid]
        say(f"   {mid}: via fitted camera ({p[0]:6.1f}, {p[1]:6.1f})  diff to sim ({d[0]:+5.1f}, {d[1]:+5.1f}) = "
            f"{np.linalg.norm(d):.0f} mm")
    for a, b in ((52, 51), (51, 53)):
        say(f"   distance {a}-{b}: sim {np.linalg.norm(sim_m[a] - sim_m[b]):.0f} mm, real "
            f"{np.linalg.norm(real_m[a] - real_m[b]):.0f} mm")

    # ---- 3. bowl + demo cube positions
    g = ref["grasp"]
    say("\n3. Demo cube positions (TCP x/y when the gripper closed), bowl")
    say(f"   {len(g)} real demos ({(ref['src'] == 'zone').sum()} zone 2026-11-10, {(ref['src'] == 'cube001').sum()} "
        f"cube001 2026-11-06): x {g[:, 0].min():.0f}..{g[:, 0].max():.0f}, y {g[:, 1].min():.0f}..{g[:, 1].max():.0f} mm")
    inside = (np.abs(g[:, :2] - ZONE_C) <= ZONE_HALF).all(1)
    say(f"   inside the 250x250 zone: {inside.sum()}/{len(g)}  (zone demos: {inside[ref['src'] == 'zone'].sum()}/"
        f"{(ref['src'] == 'zone').sum()})")
    rel = ref["release"]
    sim_bowl = world_to_base(data.body("target_bowl").xpos)[0]
    say(f"   bowl: sim centre ({sim_bowl[0]:.0f}, {sim_bowl[1]:.0f}); demos release at ({np.median(rel[:, 0]):.0f}, "
        f"{np.median(rel[:, 1]):.0f}, z {np.median(rel[:, 2]):.0f}) every time")
    say(f"   grasp height (TCP z): median {np.median(g[:, 2]):.1f} mm (5-95%: {np.percentile(g[:, 2], 5):.1f}.."
        f"{np.percentile(g[:, 2], 95):.1f}); sim table top {world_to_base([[0, 0, TABLE_TOP_W]])[0][2]:.1f} mm, "
        f"3 cm cube centre {world_to_base([[0, 0, TABLE_TOP_W + 0.015]])[0][2]:.1f} mm")

    # ---- 4. overhead camera: where the markers land in the sim image vs the real image
    say("\n4. Overhead camera")
    nom = world_to_base(data.cam_xpos[model.camera("d455_overhead_color").id][None])[0]
    say(f"   spec/export (d455_overhead_color): at ({nom[0]:.0f}, {nom[1]:.0f}, {nom[2]:.0f}) mm, looking straight down, "
        f"fovy 52 deg")
    ax_ = Rc.T @ [0, 0, 1]
    say(f"   fitted on {n_in}/{n_all} demos (median reprojection {rep:.1f} px, f fixed {F_OVERHEAD:.0f} px): at "
        f"({Cc[0] * 1000:.0f}, {Cc[1] * 1000:.0f}, {Cc[2] * 1000:.0f}) mm, tilted "
        f"{np.degrees(np.arccos(-ax_[2])):.1f} deg, image rotated {np.degrees(np.arctan2(-(Rc.T @ [1, 0, 0])[0], -(Rc.T @ [1, 0, 0])[1])):.1f} deg, "
        f"fovy {2 * np.degrees(np.arctan((H / 2) / F_OVERHEAD)):.1f} deg  -> scene camera 'overhead'")
    say("   marker centres in the 640x480 image (px): nominal sim | calibrated sim | real")
    sim_px, real_px = {}, {}
    for mid in (51, 52, 53):
        nm = project(model, data, "d455_overhead_color", data.body(f"aruco_marker_{mid}").xpos[None])[0]
        sim_px[mid] = project(model, data, "overhead", data.body(f"aruco_marker_{mid}").xpos[None])[0]
        real_px[mid] = ref[f"m{mid}_cube001"].mean(0)
        d = real_px[mid] - sim_px[mid]
        say(f"   {mid}: ({nm[0]:4.0f}, {nm[1]:4.0f}) | ({sim_px[mid][0]:4.0f}, {sim_px[mid][1]:4.0f}) | "
            f"({real_px[mid][0]:4.0f}, {real_px[mid][1]:4.0f})   calibrated - real ({-d[0]:+3.0f}, {-d[1]:+3.0f}) px")

    # ---- plot: sim overhead | real overhead, red dot per demo cube
    renderer = mujoco.Renderer(model, H, W)
    renderer.update_scene(data, camera="d455_overhead_color")
    nominal_img = renderer.render().copy()
    renderer.update_scene(data, camera="overhead")
    sim_img = renderer.render().copy()
    renderer.update_scene(data, camera="wrist")
    wrist_img = renderer.render().copy()
    renderer.close()
    cv2.imwrite(str(HERE / "sim_wrist_home.png"), cv2.cvtColor(wrist_img, cv2.COLOR_RGB2BGR))

    cv2.imwrite(str(HERE / "sim_overhead_nominal.png"), cv2.cvtColor(nominal_img, cv2.COLOR_RGB2BGR))
    R2T = np.linalg.inv(ref["T2R"])
    raw_of = np.linalg.inv(ref["H_cube001"])

    def robot_to_real_px(xy):
        t = cv2.perspectiveTransform(np.float32(xy).reshape(-1, 1, 2), R2T)
        return cv2.perspectiveTransform(t, raw_of).reshape(-1, 2)

    def robot_to_sim_px(xy, z=15.0):
        xy = np.asarray(xy, float).reshape(-1, 2)
        return project(model, data, "overhead", base_to_world(np.c_[xy, np.full(len(xy), z)]))

    sq = ZONE_C + ZONE_HALF * np.array([[1, 1], [1, -1], [-1, -1], [-1, 1], [1, 1]])
    circ = BOWL + BOWL_R * np.c_[np.cos(np.linspace(0, 2 * np.pi, 60)), np.sin(np.linspace(0, 2 * np.pi, 60))]

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(1, 3, figsize=(23, 6.4), gridspec_kw=dict(width_ratios=[1, 1, 0.85]))
    for ax, img, to_px, title in (
            (axs[0], sim_img, robot_to_sim_px, "MuJoCo sim (calibrated 'overhead' camera, arm at dataset HOME)"),
            (axs[1], ref["img_cube001"], robot_to_real_px, "Real D455 overhead (recording 2026-11-06)")):
        ax.imshow(img)
        p = to_px(g[:, :2])
        ax.scatter(p[:, 0], p[:, 1], s=22, c="red", edgecolors="white", linewidths=0.6,
                   label=f"demo cube positions ({len(g)})")
        z = to_px(sq)
        ax.plot(z[:, 0], z[:, 1], "--", c="yellow", lw=1.4, label="250x250 mm zone")
        b = to_px(circ)
        ax.plot(b[:, 0], b[:, 1], c="cyan", lw=1.4, label="bowl (release point)")
        ax.set_title(title)
        ax.set_xlim(0, W)
        ax.set_ylim(H, 0)
        ax.axis("off")
    for mid in (51, 52, 53):
        axs[0].annotate(str(mid), sim_px[mid], color="lime", fontsize=11, ha="center", va="center", weight="bold")
        axs[1].annotate(str(mid), real_px[mid], color="lime", fontsize=11, ha="center", va="center", weight="bold")
    axs[1].legend(loc="lower left", fontsize=9)

    # ---- 3rd panel: original vs synthetic (recoloured) episodes, robot frame, same orientation as the views
    ax = axs[2]
    tint = {"red": "#d62728", "green": "#2ca02c", "blue": "#1f77b4", "?": "grey"}
    sg, sc = ref["syn_grasp"][:, :2], ref["syn_colour"]
    for col in ("red", "green", "blue"):
        m = ref["colour"] == col
        if m.any():
            ax.scatter(-g[m, 1], g[m, 0], s=26, c=tint[col], edgecolors="black", linewidths=0.4,
                       label=f"original {col} ({m.sum()})", zorder=3)
    for col in ("red", "green", "blue"):
        m = sc == col
        if m.any():
            ax.scatter(-sg[m, 1], sg[m, 0], s={"red": 190, "green": 85, "blue": 135}[col], facecolors="none",
                       edgecolors=tint[col], linewidths=1.4,
                       label=f"synthetic {col} ({m.sum()})", zorder=2)
    ax.plot(-sq[:, 1], sq[:, 0], "--", c="goldenrod", lw=1.4, label="250x250 mm zone")
    ax.plot(-circ[:, 1], circ[:, 0], c="c", lw=1.4, label="bowl")
    ax.scatter([0], [0], marker="s", s=80, c="k", label="robot base")
    ax.set_aspect("equal")
    ax.set_xlabel("robot  -y  (mm, image right)")
    ax.set_ylabel("robot  x  (mm, away from base)")
    ax.grid(alpha=0.3)
    ax.set_title(f"Original ({len(g)}) vs synthetic recoloured ({len(sg)}) episodes\n"
                 "rings = synthetic copies of the blue demos (same positions, recoloured red + green)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), fontsize=8, ncol=4)
    fig.suptitle("Demo cube positions (TCP x/y at grasp) on the sim and the real overhead view - same robot-mm "
                 "points in both", fontsize=12)
    fig.tight_layout()
    fig.savefig(HERE / "sim_vs_demos.png", dpi=110, bbox_inches="tight")
    (HERE / "sim_vs_demos.txt").write_text("\n".join(lines) + "\n")
    print(f"\nwrote {HERE / 'tataro_scene.xml'}, sim_vs_demos.png, sim_vs_demos.txt, sim_wrist_home.png")


if __name__ == "__main__":
    main()
