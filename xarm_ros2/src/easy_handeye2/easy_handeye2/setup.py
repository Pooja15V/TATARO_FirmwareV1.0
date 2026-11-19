import os
from glob import glob

from setuptools import setup

package_name = 'easy_handeye2'

setup(
 name=package_name,
 version='0.5.0',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
     # ament_python does not auto-generate the rqt_gui plugin resource-index
     # marker from the <rqt_gui plugin="..."/> exports in package.xml the way
     # ament_cmake packages do (via pluginlib_export_plugin_description_file).
     # Without this marker, rqt's plugin discovery silently fails with
     # "found no plugin matching ...RqtHandeyeCalibrator" even though
     # plugin_calibrator.xml is installed and the class imports fine.
     ('share/ament_index/resource_index/rqt_gui__pluginlib__plugin',
      ['resource/rqt_gui__pluginlib__plugin/' + package_name]),
     ('share/' + package_name + '/resource', glob(os.path.join('resource', '*.ui'))),
     ('share/' + package_name, ['package.xml']),
     ('share/' + package_name, ['plugin_evaluator.xml','plugin_calibrator.xml']),
     (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*')))
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Marco Esposito',
 maintainer_email='esposito@imfusion.com',
 description='Simple, hardware-independent ROS2 library for hand-eye calibration',
 license='BSD',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
                'handeye_server = easy_handeye2.handeye_server:main',
                'handeye_server_robot = easy_handeye2.handeye_server_robot:main',
                'handeye_publisher = easy_handeye2.handeye_publisher:main',
                'handeye_calibration_commander = easy_handeye2.handeye_calibration_commander:main',
     ],
   },
  scripts = [
      'scripts/rqt_calibrator.py',
      'scripts/rqt_evaluator.py',
  ]
)