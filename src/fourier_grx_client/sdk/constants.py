import numpy

BASE_LINK = "base"

# fmt: off
DEFAULT_POSITIONS = numpy.rad2deg(numpy.array(
    [
        0.0, 0.0, -0.2618, 0.5236, -0.2618, 0.0,  # left leg
        0.0, 0.0, -0.2618, 0.5236, -0.2618, 0.0,  # right leg
        0.0, 0.0, 0.0,  # waist
        0.0, 0.0, 0.0,  # head
        0.0, 0.2, 0.0, -0.3, 0.0, 0.0, 0.0,  # left arm
        0.0, -0.2, 0.0, -0.3, 0.0, 0.0, 0.0,  # right arm
    ]
))
# fmt: on
