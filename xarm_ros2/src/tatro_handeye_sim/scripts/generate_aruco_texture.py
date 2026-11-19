#!/usr/bin/env python3
"""
generate_aruco_texture.py
──────────────────────────────────────────────────────────────────────────────
Generates the ArUco marker ID=0 PNG texture for the Gazebo model.

Run ONCE before launching the simulation:
    python3 tatro_handeye_sim/scripts/generate_aruco_texture.py

Dependencies:
    pip install opencv-contrib-python   (for cv2.aruco)

Dictionary used: DICT_ARUCO_ORIGINAL
    This is the default dictionary in aruco_ros (ROS 2 Humble).
    If your aruco_ros uses a different dictionary, change DICTIONARY_ID below.
"""
import os
import sys
import numpy as np

# ── aruco dictionary selection ────────────────────────────────────────────
# Match the dictionary your aruco_ros detector uses.
# Common choices:
#   cv2.aruco.DICT_ARUCO_ORIGINAL   ← aruco_ros default
#   cv2.aruco.DICT_4X4_50
#   cv2.aruco.DICT_6X6_250
MARKER_ID   = 0
MARKER_SIZE_PX = 800     # output PNG size in pixels (before border)
BORDER_FRACTION = 0.20   # white quiet zone = 20% of marker size on each side

try:
    import cv2
    import cv2.aruco as aruco
except ImportError:
    sys.exit(
        "ERROR: opencv-contrib-python not found.\n"
        "Install with:  pip install opencv-contrib-python\n"
        "or:            sudo apt install python3-opencv\n"
    )


def main():
    # ── output path ───────────────────────────────────────────────────────
    # Prefer the installed share directory (works after colcon build);
    # fall back to the source tree when run directly.
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('tatro_handeye_sim')
    except Exception:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_share  = os.path.dirname(script_dir)

    texture_dir = os.path.join(pkg_share, 'models', 'aruco_marker_0',
                               'materials', 'textures')
    os.makedirs(texture_dir, exist_ok=True)
    output_path = os.path.join(texture_dir, 'aruco_marker_0.png')

    # ── generate marker ───────────────────────────────────────────────────
    try:
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    except AttributeError:
        # Newer OpenCV API
        dictionary = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

    try:
        marker_img = aruco.generateImageMarker(dictionary, MARKER_ID, MARKER_SIZE_PX)
    except AttributeError:
        marker_img = aruco.drawMarker(dictionary, MARKER_ID, MARKER_SIZE_PX)

    # ── add white quiet zone (required by ArUco spec) ─────────────────────
    border_px = int(MARKER_SIZE_PX * BORDER_FRACTION)
    marker_with_border = cv2.copyMakeBorder(
        marker_img,
        border_px, border_px, border_px, border_px,
        cv2.BORDER_CONSTANT,
        value=255
    )

    # ── save ──────────────────────────────────────────────────────────────
    cv2.imwrite(output_path, marker_with_border)
    total_size = MARKER_SIZE_PX + 2 * border_px
    print(f"ArUco marker ID={MARKER_ID} written to:\n  {output_path}")
    print(f"  Image size: {total_size}×{total_size} px "
          f"(marker {MARKER_SIZE_PX}px + {border_px}px border each side)")
    print("\nNext: launch the simulation with GAZEBO_MODEL_PATH set, e.g.:")
    print("  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:"
          "<path_to_tatro_handeye_sim>/models")
    print("  ros2 launch tatro_handeye_sim tatro_gazebo_calibration.launch.py")


if __name__ == '__main__':
    main()
