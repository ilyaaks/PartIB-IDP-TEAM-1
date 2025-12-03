"""
Path Planning Module for Autonomous Robot Navigation

This module defines the Path class containing static methods for generating
navigation sequences between different locations on the robot's workspace.

Each path method returns two lists:
    1. Movement commands: sequence of directions ("straight", "right_forward", "left_forward", "stop")
    2. Line counts: number of lines to detect at each movement step

Locations:
    - G: Start point
    - A0, A1: Bay A ground floor and first floor positions
    - B0, B1: Bay B ground floor and first floor positions
    - Green_bay, Blue_bay, Red_bay, Yellow_bay: Color-coded bay locations
"""


class Path:
    """
    Path planning class containing static methods for robot navigation.
    
    All methods follow the naming convention: _path_<source>_to_<destination>
    Each method takes current_position and desired_position tuples as parameters.
    """

    # ========================================================================
    # PATHS FROM START POINT (G) TO BAYS
    # ========================================================================

    @staticmethod
    def _path_G_to_B0(current_position, desired_position):
        # From Start Point to Bay B ground floor
        return ["straight", "right_forward", "left_forward", "stop"], [0, 2, 2, int(desired_position[2])]

    @staticmethod
    def _path_G_to_B1(current_position, desired_position):
        # From Start Point to Bay B first floor
        return (["straight", "right_forward", "left_forward", "left_forward", "left_forward", "left_forward",
                 "left_forward", "stop"],
                [0, 2, 2, 8, 1, 1, 1, int(desired_position[2])])

    @staticmethod
    def _path_G_to_A0(current_position, desired_position):
        # From Start Point to Bay A ground floor
        return ["straight", "left_forward", "right_forward", "stop"], [0, 2, 2, int(desired_position[2])]

    @staticmethod
    def _path_G_to_A1(current_position, desired_position):
        # From Start Point to Bay A first floor
        return (["straight", "left_forward", "right_forward", "right_forward", "right_forward", "right_forward",
                 "right_forward", "stop"],
                [0, 2, 2, 8, 1, 1, 1, int(desired_position[2])])

    # ========================================================================
    # PATHS FROM BAYS TO START POINT (G)
    # ========================================================================

    @staticmethod
    def _path_B0_to_G(current_position, desired_position):
        # From Bay B ground floor to Start Point
        return ["right_forward", "left_forward", "stop"], [int(current_position[2]), 2, 1]

    @staticmethod
    def _path_B1_to_G(current_position, desired_position):
        # From Bay B first floor to Start Point
        return (["right_forward", "right_forward", "right_forward", "right_forward", "right_forward",
                 "left_forward", "stop"],
                [int(current_position[2]), 1, 1, 1, 8, 2, 1])

    @staticmethod
    def _path_A0_to_G(current_position, desired_position):
        # From Bay A ground floor to Start Point
        return ["left_forward", "right_forward", "stop"], [int(current_position[2]), 2, 1]

    @staticmethod
    def _path_A1_to_G(current_position, desired_position):
        # From Bay A first floor to Start Point
        return (["left_forward", "left_forward", "left_forward", "left_forward", "left_forward",
                 "right_forward", "stop"],
                [int(current_position[2]), 1, 1, 1, 8, 2, 1])

    # ========================================================================
    # GREEN BAY PATHS
    # ========================================================================

    @staticmethod
    def _path_Green_bay_to_B0(current_position, desired_position):
        # From Green bay to Bay B ground floor
        return ["right_forward", "left_forward", "stop"], [1, 3, int(desired_position[2])]

    @staticmethod
    def _path_Green_bay_to_B1(current_position, desired_position):
        # From Green bay to Bay B first floor
        return (["right_forward", "left_forward", "left_forward", "left_forward", "left_forward",
                 "left_forward", "stop"],
                [1, 3, 8, 1, 1, 1, int(desired_position[2])])

    @staticmethod
    def _path_Green_bay_to_A0(current_position, desired_position):
        # From Green bay to Bay A ground floor
        return ["left_forward", "right_forward", "stop"], [1, 1, int(desired_position[2])]

    @staticmethod
    def _path_Green_bay_to_A1(current_position, desired_position):
        # From Green bay to Bay A first floor
        return (["left_forward", "right_forward", "right_forward", "right_forward", "right_forward",
                 "right_forward", "stop"],
                [1, 1, 8, 1, 1, 1, int(desired_position[2])])

    @staticmethod
    def _path_B0_to_Green_bay(current_position, desired_position):
        # From Bay B ground floor to Green bay
        return ["straight", "right_forward", "left_forward", "stop"], [0, int(current_position[2]), 3, 1]

    @staticmethod
    def _path_B1_to_Green_bay(current_position, desired_position):
        # From Bay B first floor to Green bay
        return (["straight", "right_forward", "right_forward", "right_forward", "right_forward", "right_forward",
                 "left_forward", "stop"],
                [0, int(current_position[2]), 1, 1, 1, 8, 3, 1])

    @staticmethod
    def _path_A0_to_Green_bay(current_position, desired_position):
        # From Bay A ground floor to Green bay
        return ["straight", "left_forward", "right_forward", "stop"], [0, int(current_position[2]), 1, 1]

    @staticmethod
    def _path_A1_to_Green_bay(current_position, desired_position):
        # From Bay A first floor to Green bay
        return (["straight", "left_forward", "left_forward", "left_forward", "left_forward", "left_forward",
                 "right_forward", "stop"],
                [0, int(current_position[2]), 1, 1, 1, 8, 2, 1])

    # ========================================================================
    # BLUE BAY PATHS
    # ========================================================================

    @staticmethod
    def _path_Blue_bay_to_B0(current_position, desired_position):
        # From Blue bay to Bay B ground floor
        return ["right_forward", "left_forward", "stop"], [1, 4, int(desired_position[2])]

    @staticmethod
    def _path_Blue_bay_to_B1(current_position, desired_position):
        # From Blue bay to Bay B first floor
        return (["right_forward", "left_forward", "left_forward", "left_forward", "left_forward",
                 "left_forward", "stop"],
                [1, 4, 8, 1, 1, 1, int(desired_position[2])])

    @staticmethod
    def _path_Blue_bay_to_A0(current_position, desired_position):
        # From Blue bay to Bay A ground floor
        return ["stop"], [int(desired_position[2]) + 1]

    @staticmethod
    def _path_Blue_bay_to_A1(current_position, desired_position):
        # From Blue bay to Bay A first floor
        return (["right_forward", "right_forward", "right_forward", "right_forward", "stop"],
                [1 + 8, 1, 1, 1, int(desired_position[2])])

    @staticmethod
    def _path_B0_to_Blue_bay(current_position, desired_position):
        # From Bay B ground floor to Blue bay
        return ["straight", "right_forward", "left_forward", "stop"], [0, int(current_position[2]), 4, 0]

    @staticmethod
    def _path_B1_to_Blue_bay(current_position, desired_position):
        # From Bay B first floor to Blue bay
        return (["straight", "right_forward", "right_forward", "right_forward", "right_forward", "right_forward",
                 "left_forward", "stop"],
                [0, int(current_position[2]), 1, 1, 1, 8, 4, 1])

    @staticmethod
    def _path_A0_to_Blue_bay(current_position, desired_position):
        # From Bay A ground floor to Blue bay
        return ["straight", "stop"], [0, int(current_position[2]) + 2]

    @staticmethod
    def _path_A1_to_Blue_bay(current_position, desired_position):
        # From Bay A first floor to Blue bay
        return (["straight", "left_forward", "left_forward", "left_forward", "left_forward", "stop"],
                [0, int(current_position[2]), 1, 1, 1, 10])

    # ========================================================================
    # RED BAY PATHS
    # ========================================================================

    @staticmethod
    def _path_Red_bay_to_B0(current_position, desired_position):
        # From Red bay to Bay B ground floor
        return ["stop"], [int(desired_position[2]) + 1]

    @staticmethod
    def _path_Red_bay_to_B1(current_position, desired_position):
        # From Red bay to Bay B first floor
        return (["left_forward", "left_forward", "left_forward", "left_forward", "stop"],
                [9, 1, 1, 1, int(desired_position[2])])

    @staticmethod
    def _path_Red_bay_to_A0(current_position, desired_position):
        # From Red bay to Bay A ground floor
        return ["left_forward", "right_forward", "stop"], [1, 4, int(desired_position[2])]

    @staticmethod
    def _path_Red_bay_to_A1(current_position, desired_position):
        # From Red bay to Bay A first floor
        return (["left_forward", "right_forward", "right_forward", "right_forward", "right_forward",
                 "right_forward", "stop"],
                [1, 4, 8, 1, 1, 1, int(desired_position[2])])

    @staticmethod
    def _path_B0_to_Red_bay(current_position, desired_position):
        # From Bay B ground floor to Red bay
        return ["straight", "stop"], [0, int(current_position[2]) + 2]

    @staticmethod
    def _path_B1_to_Red_bay(current_position, desired_position):
        # From Bay B first floor to Red bay
        return (["straight", "right_forward", "right_forward", "right_forward","right_forward", "stop"],
                [0, int(current_position[2]), 1, 1, 1, 10])

    @staticmethod
    def _path_A0_to_Red_bay(current_position, desired_position):
        print(current_position, "type is", type(current_position))
        print(desired_position, "type is", type(desired_position))
        # From Bay A ground floor to Red bay
        return ["straight", "left_forward", "right_forward", "stop"], [0, int(current_position[2]), 4, 1]

    @staticmethod
    def _path_A1_to_Red_bay(current_position, desired_position):
        # From Bay A first floor to Red bay
        return (["straight", "left_forward", "left_forward", "left_forward", "left_forward", "left_forward",
                 "right_forward", "stop"],
                [0, int(current_position[2]), 1, 1, 1, 8, 4, 1])

    # ========================================================================
    # YELLOW BAY PATHS
    # ========================================================================

    @staticmethod
    def _path_Yellow_bay_to_B0(current_position, desired_position):
        # From Yellow bay to Bay B ground floor
        return ["right_forward", "left_forward", "stop"], [1, 1, int(desired_position[2])]

    @staticmethod
    def _path_Yellow_bay_to_B1(current_position, desired_position):
        # From Yellow bay to Bay B first floor
        return (["left_forward", "left_forward", "left_forward", "left_forward", "stop"],
                [9, 1, 1, 1, int(desired_position[2])])

    @staticmethod
    def _path_Yellow_bay_to_A0(current_position, desired_position):
        # From Yellow bay to Bay A ground floor
        return ["left_forward", "right_forward", "stop"], [1, 3, int(desired_position[2])]

    @staticmethod
    def _path_Yellow_bay_to_A1(current_position, desired_position):
        # From Yellow bay to Bay A first floor
        return (["left_forward", "right_forward", "right_forward", "right_forward", "right_forward",
                 "right_forward", "stop"],
                [1, 4, 8, 1, 1, 1, int(desired_position[2])])

    @staticmethod
    def _path_B0_to_Yellow_bay(current_position, desired_position):
        # From Bay B ground floor to Yellow bay
        return ["straight", "right_forward", "left_forward", "stop"], [0, int(current_position[2]), 1, 1]

    @staticmethod
    def _path_B1_to_Yellow_bay(current_position, desired_position):
        # From Bay B first floor to Yellow bay
        return (["straight", "right_forward", "right_forward", "right_forward", "right_forward", "right_forward",
                 "left_forward", "stop"],
                [0, int(current_position[2]), 1, 1, 1, 8, 1, 1])

    @staticmethod
    def _path_A0_to_Yellow_bay(current_position, desired_position):
        # From Bay A ground floor to Yellow bay
        return ["straight", "left_forward", "right_forward", "stop"], [0, int(current_position[2]), 3, 1]

    @staticmethod
    def _path_A1_to_Yellow_bay(current_position, desired_position):
        # From Bay A first floor to Yellow bay
        return (["straight", "left_forward", "left_forward", "left_forward", "left_forward", "left_forward",
                 "right_forward", "stop"],
                [0, int(current_position[2]), 1, 1, 1, 8, 3, 1])

    # ========================================================================
    # BAY-TO-BAY NAVIGATION
    # ========================================================================

    @staticmethod
    def _path_A0_to_A1(current_position, desired_position):
        # From Bay A ground floor to Bay A first floor
        return (["right_forward", "right_forward", "right_forward", "right_forward", "stop"],
                [2, 1, 1, 1, 7 - int(desired_position[2])])

    @staticmethod
    def _path_B0_to_B1(current_position, desired_position):
        # From Bay B ground floor to Bay B first floor
        return (["left_forward", "left_forward", "left_forward", "left_forward", "stop"],
                [2, 1, 1, 1, 7 - int(desired_position[2])])
