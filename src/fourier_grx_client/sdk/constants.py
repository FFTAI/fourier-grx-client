import numpy as np

# fmt: off
DEFAULT_POSITIONS = np.rad2deg(np.array(
    [
        0.0, 0.0, -0.2618, 0.5236, -0.2618, 0.0,  # left leg (6)
        0.0, 0.0, -0.2618, 0.5236, -0.2618, 0.0,  # right leg (6)
        0.0, 0.0, 0.0,  # waist (3)
        0.0, 0.0, 0.0,  # head (3)
        0.0, 0.2, 0.0, -0.3, 0.0, 0.0, 0.0,  # left arm (7)
        0.0, -0.2, 0.0, -0.3, 0.0, 0.0, 0.0,  # right arm (7)
    ]
))
# fmt: on
