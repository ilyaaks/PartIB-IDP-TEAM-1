# from G0 to B01
# return ["right", 1, "left", 0]


# generate the Path class framework
class Path:
    @staticmethod
    def _path_G_to_B0(current_position, desired_position):
        # Implement the path logic from G to B0
        if current_position == "G0" and desired_position == "B01":
            return ["right_forward", "left_forward", "left_forward"], [2,2,1] #sequence of turns and number of lines at which to detect it
        pass

    @staticmethod
    def _path_G_to_B1(current_position, desired_position):
        # Implement the path logic from G to B1
        pass

    @staticmethod
    def _path_G_to_A0(current_position, desired_position):
        # Implement the path logic from G to A0
        pass

    @staticmethod
    def _path_G_to_A1(current_position, desired_position):
        # Implement the path logic from G to A1
        pass

    @staticmethod
    def _path_B0_to_G(current_position, desired_position):
        # Implement the path logic from B0 to G
        if current_position == "B01" and desired_position == "G0":
            #First we need to reverse backwards and reverse:
            return ["reverse", "right_backward", "straight", "left_backward"], [0,0,0,2] #sequence of turns and number of lines at which to detect it
        pass

    @staticmethod
    def _path_B1_to_G(current_position, desired_position):
        # Implement the path logic from B1 to G
        pass

    @staticmethod
    def _path_A0_to_G(current_position, desired_position):
        # Implement the path logic from A0 to G
        pass

    @staticmethod
    def _path_A1_to_G(current_position, desired_position):
        # Implement the path logic from A1 to G
        pass
