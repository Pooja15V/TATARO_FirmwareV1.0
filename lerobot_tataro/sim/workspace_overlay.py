#!/usr/bin/env python3
"""Lite6 workspace on the TATARO table, drawn on the same three panels as sim_vs_demos.png.

    python3 workspace_overlay.py      # system python3 (mujoco); needs demo_reference.npz

The workspace is computed in the same MuJoCo scene as compare_sim_demos.py (same table, markers, bowl, calibrated
overhead camera). It uses damped-least-squares IK at every grid point on the table (10 mm grid, TCP at cube-centre
height z = 15 mm). A point is reachable when IK converges (< 2 mm, < 3 deg), the joints stay inside the model limits,
and the arm does not penetrate the table, markers or itself:
  * top-down workspace  - gripper pointing straight down (what the scripted / demo picks use)
  * any-orientation reach - position only (outer limit, the gripper may be tilted)

Writes sim_workspace.png and sim_workspace_mask.npz (grid + masks, robot mm) next to this file.
"""

from __future__ import annotations

import time

import cv2
import mujoco
import numpy as np

from compare_sim_demos import (BOWL, BOWL_R, H, HERE, HOME_Q_DATASET, PREFIX, W, ZONE_C, ZONE_HALF, base_to_world,
                               fit_overhead, project, scene_xml, world_to_base)

STEP = 10.0  # mm grid
Z_TCP = 15.0  # mm, cube centre height (link_base frame; table top is -5 mm)
X_RANGE = (-90.0, 760.0)  # table extent in link_base mm
Y_RANGE = (-425.0, 425.0)
R_MAX = 620.0  # skip grid points certainly out of reach (Lite6 ~440 mm + gripper)
DOWN = np.array([0.0, 0.0, -1.0])
TOP_COL, REACH_COL = "#9b30ff", "#ff8c00"  # purple fill = top-down, orange outline = any orientation


class IK:
    def __init__(self, model, data):
        self.m, self.d = model, data
        self.jids = [model.joint(f"{PREFIX}joint{i + 1}").id for i in range(6)]
        self.qadr = np.array([model.jnt_qposadr[j] for j in self.jids])
        self.dadr = np.array([model.jnt_dofadr[j] for j in self.jids])
        self.lo, self.hi = model.jnt_range[self.jids, 0], model.jnt_range[self.jids, 1]
        self.site = model.site("tcp").id
        self.jacp, self.jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
        robot = {b for b in range(model.nbody) if model.body(b).name.startswith(PREFIX)}
        self.robot_geom = np.array([model.geom_bodyid[g] in robot for g in range(model.ngeom)])
        self.ignore = {model.geom("bowl_base").id, model.geom("cube_red_geom").id}
        # scene robot geoms are contype 2 / conaffinity 1 -> never collide with each other; enable self-collision for
        # this check, ignoring pairs already touching at HOME (rigidly mounted parts: gripper/camera/flange)
        self.d.qpos[self.qadr] = HOME_Q_DATASET
        mujoco.mj_forward(model, data)
        self.home_pairs = {frozenset((c.geom1, c.geom2)) for c in data.contact[: data.ncon]
                           if self.robot_geom[c.geom1] and self.robot_geom[c.geom2]}
        print(f"  self-collision on; {len(self.home_pairs)} always-touching robot geom pairs ignored")

    def fk(self, q):
        self.d.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_comPos(self.m, self.d)

    def solve(self, target_w, q0, orient, iters=150):
        q = q0.copy()
        for _ in range(iters):
            self.fk(q)
            ep = target_w - self.d.site_xpos[self.site]
            zc = self.d.site_xmat[self.site].reshape(3, 3)[:, 2]
            eo = np.cross(zc, DOWN)
            if np.linalg.norm(ep) < 0.002 and (not orient or np.dot(zc, DOWN) > np.cos(np.radians(3))):
                return q, True
            mujoco.mj_jacSite(self.m, self.d, self.jacp, self.jacr, self.site)
            J = self.jacp[:, self.dadr]
            e = ep
            if orient:
                J = np.r_[J, 0.3 * self.jacr[:, self.dadr]]
                e = np.r_[ep, 0.3 * eo]
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(len(e)), e)
            n = np.linalg.norm(dq)
            q = np.clip(q + dq * min(1.0, 0.3 / max(n, 1e-9)), self.lo, self.hi)
        return q, False

    def collides(self, q):
        self.d.qpos[self.qadr] = q
        mujoco.mj_forward(self.m, self.d)
        for c in self.d.contact[: self.d.ncon]:
            if c.dist > -0.001 or c.geom1 in self.ignore or c.geom2 in self.ignore:
                continue
            if frozenset((c.geom1, c.geom2)) in self.home_pairs:
                continue
            if self.robot_geom[c.geom1] or self.robot_geom[c.geom2]:
                return True
        return False


def compute_workspace(model, data):
    ik = IK(model, data)
    # park the scene cube off the table so it never blocks the arm
    cj = model.joint("cube_red_joint")
    data.qpos[model.jnt_qposadr[cj.id]:model.jnt_qposadr[cj.id] + 3] = [5.0, 5.0, 0.0]
    xs = np.arange(X_RANGE[0], X_RANGE[1] + 1e-6, STEP)
    ys = np.arange(Y_RANGE[0], Y_RANGE[1] + 1e-6, STEP)
    top = np.zeros((len(xs), len(ys)), bool)
    reach = np.zeros_like(top)
    alt = HOME_Q_DATASET.copy()
    alt[1:3] = [0.6, 1.4]  # stretched-out seed for far points
    t0 = time.time()
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            if np.hypot(x, y) > R_MAX:
                continue
            tw = base_to_world([x, y, Z_TCP])[0]
            for orient, mask in ((True, top), (False, reach)):
                if not orient and top[i, j]:
                    reach[i, j] = True
                    continue
                for seed in (HOME_Q_DATASET, alt):
                    q0 = seed.copy()
                    q0[0] = np.arctan2(y, x)
                    q, ok = ik.solve(tw, q0, orient)
                    if ok and not ik.collides(q):
                        mask[i, j] = True
                        break
        if i % 10 == 0:
            print(f"  row {i + 1}/{len(xs)}  x={x:.0f} mm  ({time.time() - t0:.0f} s)", flush=True)
    return xs, ys, top, reach


def mask_polys(xs, ys, mask):
    """Boolean grid -> list of robot-mm polygons (outer contours, cell edges)."""
    m = cv2.resize(mask.astype(np.uint8) * 255, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cs:
        c = c.reshape(-1, 2).astype(float)  # (col=j*4, row=i*4)
        xy = np.c_[xs[0] + (c[:, 1] / 4 - 0.5 + 0.125) * STEP, ys[0] + (c[:, 0] / 4 - 0.5 + 0.125) * STEP]
        out.append(np.r_[xy, xy[:1]])
    return out


def densify(poly, step=5.0):
    """Insert points along polygon edges so curved mapping (camera projection) stays smooth."""
    pts = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(np.linalg.norm(b - a) / step))
        pts += [a + (b - a) * t for t in np.linspace(0, 1, n + 1)[1:]]
    return np.array(pts)


def main():
    ref = np.load(HERE / "demo_reference.npz")
    K, Rc, Cc, *_ = fit_overhead(ref)
    model = mujoco.MjModel.from_xml_string(scene_xml((K, Rc, Cc)))
    data = mujoco.MjData(model)

    cache = HERE / "sim_workspace_mask.npz"
    if cache.exists():
        z = np.load(cache)
        xs, ys, top, reach = z["xs"], z["ys"], z["top"], z["reach"]
        print(f"loaded {cache.name}")
    else:
        print("computing workspace (IK on a 10 mm grid) ...")
        # check model: same scene, robot collision geoms 2/1 -> 3/3 so the arm also collides with itself
        cxml = scene_xml((K, Rc, Cc)).replace('contype="2" conaffinity="1"', 'contype="3" conaffinity="3"')
        cm = mujoco.MjModel.from_xml_string(cxml)
        xs, ys, top, reach = compute_workspace(cm, mujoco.MjData(cm))
        np.savez(cache, xs=xs, ys=ys, top=top, reach=reach, z_tcp=Z_TCP)

    # arm back to HOME for the render
    mujoco.mj_resetData(model, data)
    for i, q in enumerate(HOME_Q_DATASET):
        data.qpos[model.jnt_qposadr[model.joint(f"{PREFIX}joint{i + 1}").id]] = q
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, H, W)
    renderer.update_scene(data, camera="overhead")
    sim_img = renderer.render().copy()
    renderer.close()

    g = ref["grasp"]
    XX, YY = np.meshgrid(xs, ys, indexing="ij")

    def inside(pts, mask):
        i = np.clip(np.round((pts[:, 0] - xs[0]) / STEP).astype(int), 0, len(xs) - 1)
        j = np.clip(np.round((pts[:, 1] - ys[0]) / STEP).astype(int), 0, len(ys) - 1)
        return mask[i, j]

    in_top, in_reach = inside(g[:, :2], top), inside(g[:, :2], reach)
    zone_pts = np.c_[XX.ravel(), YY.ravel()]
    zone_m = (np.abs(zone_pts - ZONE_C) <= ZONE_HALF).all(1)
    zone_cov = top.ravel()[zone_m].mean()
    cell = STEP * STEP / 1e6
    summary = [
        f"top-down workspace at z={Z_TCP:.0f} mm: {top.sum() * cell:.3f} m^2, x {XX[top].min():.0f}..{XX[top].max():.0f}, "
        f"y {YY[top].min():.0f}..{YY[top].max():.0f} mm, max radius {np.hypot(XX[top], YY[top]).max():.0f} mm",
        f"any-orientation reach at z={Z_TCP:.0f} mm: {reach.sum() * cell:.3f} m^2, max radius "
        f"{np.hypot(XX[reach], YY[reach]).max():.0f} mm",
        f"demo cube positions inside top-down workspace: {in_top.sum()}/{len(g)}  (any orientation: {in_reach.sum()})",
        f"250x250 zone covered by top-down workspace: {100 * zone_cov:.0f}%",
        f"bowl centre ({BOWL[0]:.0f}, {BOWL[1]:.0f}) top-down reachable: {bool(inside(BOWL[None], top)[0])}",
    ]
    print("\n".join(summary))

    R2T = np.linalg.inv(ref["T2R"])
    raw_of = np.linalg.inv(ref["H_cube001"])

    def robot_to_real_px(xy):
        t = cv2.perspectiveTransform(np.float32(xy).reshape(-1, 1, 2), R2T)
        return cv2.perspectiveTransform(t, raw_of).reshape(-1, 2)

    def robot_to_sim_px(xy, zz=Z_TCP):
        xy = np.asarray(xy, float).reshape(-1, 2)
        return project(model, data, "overhead", base_to_world(np.c_[xy, np.full(len(xy), zz)]))

    sq = ZONE_C + ZONE_HALF * np.array([[1, 1], [1, -1], [-1, -1], [-1, 1], [1, 1]])
    circ = BOWL + BOWL_R * np.c_[np.cos(np.linspace(0, 2 * np.pi, 60)), np.sin(np.linspace(0, 2 * np.pi, 60))]
    top_polys = [densify(p) for p in mask_polys(xs, ys, top)]
    reach_polys = [densify(p) for p in mask_polys(xs, ys, reach)]
    sim_px = {m: project(model, data, "overhead", data.body(f"aruco_marker_{m}").xpos[None])[0] for m in (51, 52, 53)}
    real_px = {m: ref[f"m{m}_cube001"].mean(0) for m in (51, 52, 53)}

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(1, 3, figsize=(23, 6.4), gridspec_kw=dict(width_ratios=[1, 1, 0.85]))
    for ax, img, to_px, title, mk in (
            (axs[0], sim_img, robot_to_sim_px, "MuJoCo sim (calibrated 'overhead' camera, arm at dataset HOME)", sim_px),
            (axs[1], ref["img_cube001"], robot_to_real_px, "Real D455 overhead (recording 2026-11-06)", real_px)):
        ax.imshow(img)
        for k, p in enumerate(top_polys):
            pp = to_px(p)
            ax.fill(pp[:, 0], pp[:, 1], color=TOP_COL, alpha=0.30, lw=0,
                    label="Lite6 top-down workspace" if k == 0 else None)
            ax.plot(pp[:, 0], pp[:, 1], c=TOP_COL, lw=1.6)
        for k, p in enumerate(reach_polys):
            pp = to_px(p)
            ax.plot(pp[:, 0], pp[:, 1], ":", c=REACH_COL, lw=1.8,
                    label="any-orientation reach" if k == 0 else None)
        p = to_px(g[:, :2])
        ax.scatter(p[:, 0], p[:, 1], s=22, c="red", edgecolors="white", linewidths=0.6,
                   label=f"demo cube positions ({len(g)})", zorder=3)
        zz = to_px(sq)
        ax.plot(zz[:, 0], zz[:, 1], "--", c="yellow", lw=1.4, label="250x250 mm zone")
        b = to_px(circ)
        ax.plot(b[:, 0], b[:, 1], c="cyan", lw=1.4, label="bowl (release point)")
        for m in (51, 52, 53):
            ax.annotate(str(m), mk[m], color="lime", fontsize=11, ha="center", va="center", weight="bold")
        ax.set_title(title)
        ax.set_xlim(0, W)
        ax.set_ylim(H, 0)
        ax.axis("off")
    axs[1].legend(loc="lower left", fontsize=8.5)

    ax = axs[2]  # robot frame, same orientation as the images (right = -y, up = +x)
    for k, p in enumerate(top_polys):
        ax.fill(-p[:, 1], p[:, 0], color=TOP_COL, alpha=0.25, lw=0)
        ax.plot(-p[:, 1], p[:, 0], c=TOP_COL, lw=1.6,
                label=f"top-down workspace ({top.sum() * cell:.2f} m²)" if k == 0 else None)
    for k, p in enumerate(reach_polys):
        ax.plot(-p[:, 1], p[:, 0], ":", c=REACH_COL, lw=1.8,
                label=f"any-orientation reach ({reach.sum() * cell:.2f} m²)" if k == 0 else None)
    ax.scatter(-g[in_top, 1], g[in_top, 0], s=24, c="red", edgecolors="black", linewidths=0.4, zorder=3,
               label=f"demo cubes in workspace ({in_top.sum()})")
    ax.scatter(-g[~in_top, 1], g[~in_top, 0], s=40, marker="x", c="black", zorder=3,
               label=f"demo cubes outside ({(~in_top).sum()})")
    ax.plot(-sq[:, 1], sq[:, 0], "--", c="goldenrod", lw=1.4, label="250x250 mm zone")
    ax.plot(-circ[:, 1], circ[:, 0], c="c", lw=1.4, label="bowl")
    tb = np.array([[X_RANGE[0], Y_RANGE[0]], [X_RANGE[0], Y_RANGE[1]], [X_RANGE[1], Y_RANGE[1]],
                   [X_RANGE[1], Y_RANGE[0]], [X_RANGE[0], Y_RANGE[0]]])
    ax.plot(-tb[:, 1], tb[:, 0], c="grey", lw=1.0, label="table edge")
    ax.scatter([0], [0], marker="s", s=80, c="k", label="robot base")
    ax.set_aspect("equal")
    ax.set_xlabel("robot  -y  (mm, image right)")
    ax.set_ylabel("robot  x  (mm, away from base)")
    ax.grid(alpha=0.3)
    ax.set_title(f"Lite6 workspace at cube height (TCP z = {Z_TCP:.0f} mm)\n"
                 f"{in_top.sum()}/{len(g)} demo cubes reachable top-down, zone {100 * zone_cov:.0f}% covered")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), fontsize=8, ncol=4)
    fig.suptitle("Lite6 reachable workspace on the TATARO table (MuJoCo IK, joint limits + collisions) - same "
                 "robot-mm region on the sim and the real overhead view", fontsize=12)
    fig.tight_layout()
    fig.savefig(HERE / "sim_workspace.png", dpi=110, bbox_inches="tight")
    (HERE / "sim_workspace.txt").write_text("\n".join(summary) + "\n")
    print(f"wrote {HERE / 'sim_workspace.png'}")


if __name__ == "__main__":
    main()
