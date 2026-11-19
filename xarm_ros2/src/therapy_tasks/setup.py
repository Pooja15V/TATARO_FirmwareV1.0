import os
from glob import glob

from setuptools import setup

package_name = 'therapy_tasks'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pooja',
    maintainer_email='thamizpooja1514@gmail.com',
    description='TATARO progressive reach therapy: FSM, button monitor, motion and perception modules',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tataro_button_monitor = therapy_tasks.button_monitor:main',
            'tataro_keyboard_button_monitor = therapy_tasks.keyboard_button_monitor:main',
            'tataro_button_move_test = therapy_tasks.button_move_test:main',
            'tataro_level1_grid_test = therapy_tasks.level1_grid_test:main',
            'tataro_grasp_lift_test = therapy_tasks.grasp_lift_test:main',
            'tataro_therapy_fsm = therapy_tasks.fsm_node:main',
        ],
    },
)
