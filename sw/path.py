# from G0 to B01
# return ["right", 1, "left", 0]


# generate the Path class framework
class Path:
    @staticmethod
    def _path_G_to_B0(current_position, desired_position):
        # Implement the path logic from G to B01
        return ["straight", "right_forward", "left_forward", "stop"], [0, 2, 2, int(desired_position[2])]

    @staticmethod
    def _path_G_to_B1(current_position, desired_position):
        # Implement the path logic from G to B1
        return ["straight", "right_forward", "left_forward", "left_forward", "left_forward", "left_forward", "left_forward", "stop"], [0, 2, 2, 8, 1, 1, 1, int(desired_position[2])]

    @staticmethod
    def _path_G_to_A0(current_position, desired_position):
        # Implement the path logic from G to A0
        return ["straight", "left_forward", "right_forward", "stop"], [0, 2, 2, int(desired_position[2])]

    @staticmethod
    def _path_G_to_A1(current_position, desired_position):
        # Implement the path logic from G to A1
        return ["straight", "left_forward", "right_forward", "right_forward", "right_forward", "right_forward", "right_forward", "stop"], [0, 2, 2, 8, 1, 1, 1, int(desired_position[2])]

    @staticmethod
    def _path_B0_to_G(current_position, desired_position):
        # Implement the path logic from B0 to G
        return ["straight", "right_forward", "left_forward", "stop"], [0, int(current_position[2]), 2, 1]

    @staticmethod
    def _path_B1_to_G(current_position, desired_position):
        # Implement the path logic from B1 to G
        return ["straight", "right_forward", "right_forward", "right_forward", "right_forward", "right_forward", "left_forward", "stop"], [0, int(current_position[2]), 1, 1, 1, 8, 2, 1] #sequence of turns and number of lines at which to detect it

    @staticmethod
    def _path_A0_to_G(current_position, desired_position):
        # Implement the path logic from A0 to G
        return ["straight", "left_forward", "right_forward", "stop"], [0, int(current_position[2]), 2, 1]

    @staticmethod
    def _path_A1_to_G(current_position, desired_position):
        # Implement the path logic from A1 to G
        return ["straight", "left_forward", "left_forward", "left_forward", "left_forward", "left_forward", "right_forward", "stop"], [0, int(current_position[2]), 1, 1, 1, 8, 2, 1]

    ## Add paths for the bays R-Y respectively:
    @staticmethod
    def _path_Green_bay_to_B0(current_position, desired_position):
        return ["straight", "right_forward", "left_forward", "stop"], [0, 2, 3, int(desired_position[2])]

    @staticmethod
    def _path_Green_bay_to_B1(current_position, desired_position):
        return ["straight", "right_forward", "left_forward", "left_forward", "left_forward", "left_forward", "left_forward", "stop"], [0, 2, 3, 8, 1, 1, 1, int(desired_position[2])]

    @staticmethod
    def _path_Green_bay_to_A0(current_position, desired_position):
        return ["straight", "left_forward", "right_forward", "stop"], [0, 2, 1, int(desired_position[2])]

    @staticmethod
    def _path_Green_bay_to_A1(current_position, desired_position):
        return ["straight", "left_forward", "right_forward", "right_forward", "right_forward", "right_forward", "right_forward", "stop"], [0, 2, 1, 8, 1, 1, 1, int(desired_position[2])]

    @staticmethod
    def _path_B0_to_Green_bay(current_position, desired_position):
        return ["straight", "right_forward", "left_forward", "stop"], [0, int(current_position[2]), 3, 1]

    @staticmethod
    def _path_B1_to_Green_bay(current_position, desired_position):
        return ["straight", "right_forward", "right_forward", "right_forward", "right_forward", "right_forward", "left_forward", "stop"], [0, int(current_position[2]), 1, 1, 1, 8, 3, 1]

    @staticmethod
    def _path_A0_to_Green_bay(current_position, desired_position):
        return ["straight", "left_forward", "right_forward", "stop"], [0, int(current_position[2]), 1, 1]

    @staticmethod
    def _path_A1_to_Green_bay(current_position, desired_position):
        return ["straight", "left_forward", "left_forward", "left_forward", "left_forward", "left_forward", "right_forward", "stop"], [0, int(current_position[2]), 1, 1, 1, 8, 2, 1]

    # Blue bay paths
    @staticmethod
    def _path_Blue_bay_to_B0(current_position, desired_position):
        return ["straight", "right_forward", "left_forward", "stop"], [0, 2, 4, int(desired_position[2])]

    @staticmethod
    def _path_Blue_bay_to_B1(current_position, desired_position):
        return ["straight", "right_forward", "left_forward", "left_forward", "left_forward", "left_forward", "left_forward", "stop"], [0, 2, 4, 8, 1, 1, 1, int(desired_position[2])]

    @staticmethod
    def _path_Blue_bay_to_A0(current_position, desired_position):
        return ["straight", "stop"], [0, int(desired_position[2]) + 2]

    @staticmethod
    def _path_Blue_bay_to_A1(current_position, desired_position):
        return ["straight", "right_forward", "right_forward", "right_forward", "right_forward", "stop"], [0, 2 + 8, 1, 1, 1, int(desired_position[2])]

    @staticmethod
    def _path_B0_to_Blue_bay(current_position, desired_position):
        return ["straight", "right_forward", "left_forward", "stop"], [0, int(current_position[2]), 4, 0]

    @staticmethod
    def _path_B1_to_Blue_bay(current_position, desired_position):
        return ["straight", "right_forward", "right_forward", "right_forward", "right_forward", "right_forward", "left_forward", "stop"], [0, int(current_position[2]), 1, 1, 1, 8, 4, 1]

    @staticmethod
    def _path_A0_to_Blue_bay(current_position, desired_position):
        return ["straight", "stop"], [0, int(current_position[2]) + 2]

    @staticmethod
    def _path_A1_to_Blue_bay(current_position, desired_position):
        return ["straight", "left_forward", "left_forward", "left_forward", "left_forward", "stop"], [0, int(current_position[2]), 1, 1, 1, 10]

    # Red bay paths (bottom-right)
    @staticmethod
    def _path_Red_bay_to_B0(current_position, desired_position):
        return ["straight", "stop"], [0, int(desired_position[2]) + 2]
    @staticmethod
    def _path_Red_bay_to_B1(current_position, desired_position):
        return ["straight", "left_forward", "left_forward", "left_forward", "left_forward", "stop"], [0, 10 , 1, 1, 1, int(desired_position[2])]
    @staticmethod
    def _path_Red_bay_to_A0(current_position, desired_position):
        return ["straight", "left_forward", "right_forward", "stop"], [0, 2, 4, int(desired_position[2])]
    @staticmethod
    def _path_Red_bay_to_A1(current_position, desired_position):
        return ["straight", "left_forward", "right_forward", "right_forward", "right_forward", "right_forward", "right_forward", "stop"], [0, 2, 4, 8, 1, 1, 1, int(desired_position[2])]
    @staticmethod
    def _path_B0_to_Red_bay(current_position, desired_position):
        return ["straight", "stop"], [0, int(desired_position[2]) + 2]
    @staticmethod
    def _path_B1_to_Red_bay(current_position, desired_position):
        return ["straight", "right_forward", "right_forward", "right_forward", "stop"], [0, int(desired_position[2]),1,1,1,10]
    @staticmethod
    def _path_A0_to_Red_bay(current_position, desired_position):
        return ["straight", "left_forward", "right_forward", "stop"], [0,int(desired_position[2]),4,0]
    @staticmethod
    def _path_A1_to_Red_bay(current_position, desired_position):
        return ["straight", "left_forward", "left_forward", "left_forward", "left_forward", "left_forward", "right_forward", "stop"], [0, int(current_position[2]), 1, 1, 1, 8, 4, 1]

    # Yellow bay paths (bottom-right, next to red)
    @staticmethod
    def _path_Yellow_bay_to_B0(current_position, desired_position):
        return ["straight", "right_forward", "left_forward", "stop"], [0, 2,1, int(desired_position[2])]    
    @staticmethod
    def _path_Yellow_bay_to_B1(current_position, desired_position):
        return ["straight", "left_forward", "left_forward", "left_forward", "left_forward", "stop"], [0, 10, 1, 1, 1, int(desired_position[2])]

    @staticmethod
    def _path_Yellow_bay_to_A0(current_position, desired_position):
        return ["straight", "left_forward", "right_forward", "stop"], [0, 2, 4, int(desired_position[2])]

    @staticmethod
    def _path_Yellow_bay_to_A1(current_position, desired_position):
        return ["straight", "left_forward", "right_forward", "right_forward", "right_forward", "right_forward", "right_forward", "stop"], [0, 2, 4, 8, 1, 1, 1, int(desired_position[2])]

    @staticmethod
    def _path_B0_to_Yellow_bay(current_position, desired_position):
        return ["straight", "right_forward", "left_forward", "stop"], [0,int(desired_position[2]),1,1]
    @staticmethod
    def _path_B1_to_Yellow_bay(current_position, desired_position):
        return ["straight", "right_forward", "right_forward", "right_forward", "right_forward", "right_forward", "left_forward", "stop"], [0,int(desired_position[2]),1,1,1,8,1,1]
    @staticmethod
    def _path_A0_to_Yellow_bay(current_position, desired_position):
        return ["straight", "left_forward", "right_forward", "stop"], [0, int(current_position[2]), 3, 1]

    @staticmethod
    def _path_A1_to_Yellow_bay(current_position, desired_position):
        return ["straight", "left_forward", "left_forward", "left_forward", "left_forward", "left_forward", "right_forward", "stop"], [0, int(current_position[2]), 1, 1, 1, 8,, 4]

    # Go to next bay commands:
    @staticmethod
    def _path_A0_to_A1(current_position, desired_position):
        return ["right_forward", "right_forward", "right_forward", "right_forward", "stop"], [2, 1, 1, 1, 7 - int(desired_position[2])]

    @staticmethod
    def _path_B0_to_B1(current_position, desired_position):
        return ["left_forward", "left_forward", "left_forward", "left_forward", "stop"], [2, 1, 1, 1, 7 - int(desired_position[2])]