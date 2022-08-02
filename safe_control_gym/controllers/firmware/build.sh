cd /home/spencer/Documents/DSL/crazyswarm/ros_ws/src/crazyswarm/scripts/pycrazyswarm/cfsim
rm -rf build
rm _cffirmware.cpython-39-x86_64-linux-gnu.so cffirmware_wrap.c cffirmware.py
export CSW_PYTHON=python3
make
python3 -m pip install -e .