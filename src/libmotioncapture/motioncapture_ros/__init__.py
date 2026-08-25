# This __init__.py allows `import motioncapture_ros` to
# automatically load the compiled C++ pybind11 module.
from motioncapture_ros.motioncapture import *  # noqa: F401,F403
from motioncapture_ros.motioncapture import connect, __version__  # noqa: F401
