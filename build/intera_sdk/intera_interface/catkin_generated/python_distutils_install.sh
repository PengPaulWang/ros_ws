#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/staff/peng/ros_ws/src/intera_sdk/intera_interface"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/staff/peng/ros_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/staff/peng/ros_ws/install/lib/python3/dist-packages:/home/staff/peng/ros_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/staff/peng/ros_ws/build" \
    "/home/staff/peng/miniconda3/envs/SpotDogPy38/bin/python3" \
    "/home/staff/peng/ros_ws/src/intera_sdk/intera_interface/setup.py" \
    egg_info --egg-base /home/staff/peng/ros_ws/build/intera_sdk/intera_interface \
    build --build-base "/home/staff/peng/ros_ws/build/intera_sdk/intera_interface" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/staff/peng/ros_ws/install" --install-scripts="/home/staff/peng/ros_ws/install/bin"
