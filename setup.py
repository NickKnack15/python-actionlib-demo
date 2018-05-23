# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['actionlib_demo', 'actionlib_demo_tests'],
    package_dir={'': 'src'})

setup(**setup_args)
