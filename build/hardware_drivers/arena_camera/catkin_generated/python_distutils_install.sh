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

echo_and_run cd "/home/gem/BobaBee_ws/src/hardware_drivers/arena_camera"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/gem/BobaBee_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/gem/BobaBee_ws/install/lib/python3/dist-packages:/home/gem/BobaBee_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/gem/BobaBee_ws/build" \
    "/usr/bin/python3" \
    "/home/gem/BobaBee_ws/src/hardware_drivers/arena_camera/setup.py" \
     \
    build --build-base "/home/gem/BobaBee_ws/build/hardware_drivers/arena_camera" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/gem/BobaBee_ws/install" --install-scripts="/home/gem/BobaBee_ws/install/bin"
