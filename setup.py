from os.path import join, dirname, realpath
from setuptools import setup
import sys

assert sys.version_info.major == 3 and sys.version_info.minor >= 6, \
    "The repo is designed to work with Python 3.6 and greater." \
    + "Please install it before proceeding."

with open(join("version", "version.py")) as version_file:
    exec(version_file.read())
    
setup(
    name='mujoco_ur5_control',
    py_modules=['mujoco_ur5'],
    version=__version__,
    install_requires=[
        'mujoco_py==2.1.2.14',
        'simple_pid',
        'matplotlib==3.3.4',
		'termcolor',
		'pyqtgraph',
		'PyQt5',
        'numpy'
    ],
    description="Teaching tools for introducing people to mujoco ur5 vontrol.",
    author="zhifeishen",
)
