"""
compute_calibration.py
----------------------
Reads calibration_points.csv and fits an affine map from camera (X,Y) to
robot (X,Y) by least squares:

    Robot_X = a1*Camera_X + a2*Camera_Y + tx
    Robot_Y = b1*Camera_X + b2*Camera_Y + ty

In matrix form:  R = A @ C + t   where
    A = [[a1, a2],
         [b1, b2]]   (2x2)
    t = [tx, ty]     (2,)

Saves to calibration_data.pkl:
    {
      "A": 2x2 np.ndarray,
      "t": 2  np.ndarray,
      "rmse": float,            # fit error in mm
      "n_points": int,
      "initial_offset": (-90, -25),  # known camera->TCP offset, for reference
      "notes": "...",
    }

NOTE ON YOUR SETUP:
  The camera looks straight down and axes are ~aligned, rotation assumed
  identity, only X/Y calibrated, Z from depth. The affine fit will absorb
  any residual scale/skew and recover the translation, so the known
  (-90, -25) mm offset is stored only as metadata/sanity reference.
"""

import os
import csv
import pickle

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(HERE, "calibration_points.csv")
PKL_PATH = os.path.join(HERE, "calibration_data.pkl")

INITIAL_OFFSET = (-90.0, -25.0)  # X_offset, Y_offset in mm (given)


def load_points(path=CSV_PATH):
    cam = []
    rob = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            cam.append([float(row["camera_x"]), float(row["camera_y"])])
            rob.append([float(row["robot_x"]), float(row["robot_y"])])
    return np.asarray(cam, dtype=float), np.asarray(rob, dtype=float)


def fit_affine(cam, rob):
    """Solve R = A C + t via least squares.

    Build M = [Cx, Cy, 1] for each point and solve M @ p = R_axis
    separately for robot X and robot Y.
    Returns A (2x2), t (2,).
    """
    n = cam.shape[0]
    M = np.hstack([cam, np.ones((n, 1))])   # (n, 3)

    # Solve for X equation: [a1, a2, tx]
    px, *_ = np.linalg.lstsq(M, rob[:, 0], rcond=None)
    # Solve for Y equation: [b1, b2, ty]
    py, *_ = np.linalg.lstsq(M, rob[:, 1], rcond=None)

    A = np.array([[px[0], px[1]],
                  [py[0], py[1]]])
    t = np.array([px[2], py[2]])
    return A, t


def compute_rmse(cam, rob, A, t):
    pred = (A @ cam.T).T + t           # (n, 2)
    err = pred - rob
    return float(np.sqrt(np.mean(np.sum(err ** 2, axis=1)))), pred


def main():
    if not os.path.exists(CSV_PATH):
        raise FileNotFoundError(
            f"{CSV_PATH} not found. Run collect_calibration_points.py first."
        )

    cam, rob = load_points()
    n = cam.shape[0]
    if n < 3:
        raise ValueError(f"Need at least 3 points to fit an affine map; got {n}.")

    A, t = fit_affine(cam, rob)
    rmse, pred = compute_rmse(cam, rob, A, t)

    print("=== Calibration result ===")
    print("A =\n", np.array2string(A, precision=5))
    print("t =", np.array2string(t, precision=4))
    print(f"Points used: {n}")
    print(f"RMSE: {rmse:.3f} mm")

    # Per-point residuals to help spot bad clicks.
    print("\nPer-point residual (mm):")
    res = np.linalg.norm(pred - rob, axis=1)
    for i, r in enumerate(res):
        flag = "  <-- check" if r > rmse * 2 and r > 1.0 else ""
        print(f"  pt {i+1:2d}: {r:6.3f}{flag}")

    data = {
        "A": A,
        "t": t,
        "rmse": rmse,
        "n_points": n,
        "initial_offset": INITIAL_OFFSET,
        "notes": ("Eye-in-hand, camera looking down, rotation assumed identity. "
                  "Affine map camera(X,Y)->robot(X,Y). Z from depth, not calibrated."),
    }
    with open(PKL_PATH, "wb") as f:
        pickle.dump(data, f)
    print(f"\n[Saved] calibration -> {PKL_PATH}")


if __name__ == "__main__":
    main()