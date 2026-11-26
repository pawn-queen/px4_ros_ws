from glob import glob
from setuptools import setup

name = 'px4_powerplant_slam'
setup(
    name=name,
    version='0.1.0',
    packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml', 'README.md']),
        ('share/' + name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    maintainer='pawn',
    maintainer_email='pawn@example.com',
    description='Isolated RGB-D SLAM for the powerplant simulator.',
    license='BSD-3-Clause',
    entry_points={'console_scripts': [
        'simulation_inputs = px4_powerplant_slam.inputs:main',
    ]},
)
