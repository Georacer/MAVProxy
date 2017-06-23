from pymavlink import mavutil

class SurveyCreator:

    def __init__(self, target_system, target_component):
        # Specify the available pattern options
        self.options = {"Clover" : self.clover,
                        "Loiter" : self.loiter}

        self.system = target_system
        self.component = target_component
        self.scale = 50  # Pattern diameter in meters
        self.loiter_radius = 50

    def create_survey(self, poi, altitude=100, pattern="Loiter"):
        if pattern not in self.options:
            print("ERROR: Invalid pattern given")
            return None
        else:
            return self.options[pattern](poi, altitude)

    def loiter(self, poi, altitude):
        print("Creating a loiter with coordinates (%f,%f) and altitude %g") % (poi[0], poi[1], altitude)
        answer = []

        w = mavutil.mavlink.MAVLink_mission_item_message(
            self.system,  # Set target system
            self.component,  # Set target component
            1,  # Set sequence number
            0,  # Frame type, 0 global (absolute alt), 3 for global_relative (relative alt)
            mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,  # Command ID
            0, 1,  # Current and Autocontinue
            0,  # Param 1
            0,  # Param 2
            self.loiter_radius,  # Loiter radius, signed
            0,  # Desired yaw angle
            poi[0],  # Item latitude
            poi[1],  # Item longitude
            altitude  # Item altitude
        )
        answer.append(w)
        return answer

    def clover(self, poi, altitude):
        print("Creating a clover")
        return None

