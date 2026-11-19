"""
TATARO_CV_GGCNNViewer.py — Visual validation for TATARO_CV_GGCNNDetection.py.

NOT a robot-motion script. This only displays what the GGCNN pipeline would
propose, so you can judge on real objects/lighting whether it's finding
sane grasp points before it's ever trusted near the arm -- the Cornell
pretrained weights have never seen this workspace.

Draws, every frame:
  - yellow dots  : candidate object locations from find_candidate_regions()
                   (depth-only "sticking up off the table" blobs)
  - green marker : best-quality GGCNN grasp point, with a short line segment
                   at the predicted grasp angle
  - text         : quality score, depth, and robot-frame pose for the best
                   detection

Controls: q = quit.
"""

import cv2
import numpy as np

from TATARO_CV_ROS2Camera import connect_camera, stop_camera, get_frame
from TATARO_CV_ObjectDetectionPipeline import _phase1_init_table
from TATARO_CV_GGCNNDetection import GGCNNDetector, find_candidate_regions, grasp_pose_in_robot_frame

_FONT   = cv2.FONT_HERSHEY_SIMPLEX
_WINDOW = "TATARO - GGCNN Viewer"

MIN_QUALITY = 0.15   # lower than a "trust this" threshold -- shown for
                      # visual judgement, not filtered as strictly as a
                      # real detection would be


def run_viewer():
    pipe = connect_camera()
    try:
        table = _phase1_init_table(pipe)
        if table is None:
            print("[Abort] Table initialisation cancelled.")
            return

        rvec_marker_wrt_camera = table['rvec_marker_wrt_camera']
        Table_corner_3D        = table['Table_corner_3D']
        table_mask              = table['table_mask']
        M                       = table['M']
        warped_size              = table['warped_size']

        print("[GGCNN Viewer] Loading model ...")
        detector = GGCNNDetector()
        print(f"[GGCNN Viewer] Model ready on {detector.device}. Running (q = quit) ...")

        while True:
            color_frame, depth_frame = get_frame(pipe, return_depth=True)
            if color_frame is None or depth_frame is None:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            annotated    = color_frame.copy()
            warped_table = cv2.warpPerspective(color_frame, M, warped_size)

            cv2.putText(annotated, "GGCNN Viewer (visualisation only, no robot motion)",
                        (20, 40), _FONT, 0.6, (0, 255, 0), 1)

            # Show every candidate region (pre-refine) in yellow.
            candidates = find_candidate_regions(depth_frame, table_mask=table_mask)
            for c in candidates:
                cv2.circle(annotated, (c['u'], c['v']), 5, (0, 255, 255), 2)
            print(f"[candidates] {len(candidates)}: "
                  + ", ".join(f"(u={c['u']},v={c['v']},area={c['area']:.0f})" for c in candidates))

            results = detector.detect(color_frame, depth_frame, table_mask=table_mask,
                                       min_quality=MIN_QUALITY)

            if results:
                best = results[0]
                u, v = best['u'], best['v']
                ang  = best['ang_rad']

                # Grasp-axis line segment through the predicted point.
                length = 30
                dx, dy = int(length * np.cos(ang)), int(length * np.sin(ang))
                cv2.line(annotated, (u - dx, v - dy), (u + dx, v + dy), (0, 255, 0), 2)
                cv2.circle(annotated, (u, v), 6, (0, 200, 0), -1)

                pos_robot, yaw_deg = grasp_pose_in_robot_frame(
                    best, Table_corner_3D, rvec_marker_wrt_camera
                )

                cv2.putText(annotated,
                            f"quality={best['quality']:.2f}  z={best['z_m']*100:.1f}cm  "
                            f"ang={np.degrees(ang):.0f}deg",
                            (u + 12, v - 12), _FONT, 0.5, (0, 255, 0), 1)
                cv2.putText(annotated,
                            f"Robot XYZ: {np.round(pos_robot, 1)} cm  Yaw: {yaw_deg:.1f} deg",
                            (20, 70), _FONT, 0.6, (0, 255, 0), 2)

                pt_w = cv2.perspectiveTransform(np.float32([[[u, v]]]), M)
                pw = tuple(pt_w[0, 0].astype(int))
                cv2.circle(warped_table, pw, 8, (0, 200, 0), -1)

                print(f"[GGCNN] quality={best['quality']:.2f}  "
                      f"pos_robot={np.round(pos_robot, 2)} cm  yaw={yaw_deg:.1f} deg  "
                      f"({len(results)} candidate(s) above threshold)")
            else:
                cv2.putText(annotated, "No grasp candidate above quality threshold",
                            (20, 70), _FONT, 0.6, (0, 0, 255), 2)

            cv2.imshow(_WINDOW, annotated)
            cv2.imshow("Warped Table (top-down)", warped_table)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        stop_camera(pipe)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    run_viewer()
