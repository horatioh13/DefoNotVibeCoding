from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mission_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'ncnn_model'),
         glob('ncnn_model/*')),
    ],
    install_requires=['setuptools', 'pyglet', 'imgui[FULL]'],
    zip_safe=True,
    include_package_data=True,
    maintainer='william',
    maintainer_email='williamwarrenmeeks@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_gui = mission_control.control_gui:main',
            'teleop = mission_control.teleop:main',
        ],
    },
)
