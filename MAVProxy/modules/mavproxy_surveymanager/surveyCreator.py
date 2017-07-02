from pymavlink import mavutil
import utm

class SurveyCreator:

    def __init__(self, target_system, target_component):
        # Specify the available pattern options
        self.options = {"Clover" : self.clover,
                        "Loiter" : self.loiter}

        self.system = target_system
        self.component = target_component
        self.seq_no = 0
        self.scale = 200  # Pattern diameter in meters
        self.loiter_radius = 50
        self.debug = False

    def create_survey(self, poi, altitude=100, pattern="Loiter"):
        if pattern not in self.options:
            print("ERROR: Invalid pattern given")
            return None
        else:
            self.seq_no = 1
            POI = self.create_ROI(poi[0], poi[1], 0, self.seq_no)
            self.seq_no = self.seq_no + 1
            pattern = self.options[pattern](poi, altitude)
            return [POI] + pattern

    def loiter(self, poi, altitude):
        '''Generate a loiter pattern'''
        print("Creating a loiter with coordinates (%f,%f) and altitude %g") % (poi[0], poi[1], altitude)
        pattern = []
        w = self.create_mission_item(poi[0], poi[1], altitude, mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, self.seq_no, 0, 0, self.loiter_radius, 0)
        self.seq_no = self.seq_no + 1
        pattern.append(w)
        return pattern

    def clover(self, poi, altitude):
        '''Generate a clover pattern'''
        if self.debug: print("Entered the clover pattern creator")
        pattern = []
        # Specify the clover pattern in normalized coordinates
        w1 = (0, 0)
        w2 = (-0.5, 0)
        w3 = (-0.5, 0.5)
        w4 = (0, 0.5)
        w5 = (0, -0.5)
        w6 = (0.5, -0.5)
        w7 = (0.5, 0)
        w8 = (-0.5, -0.5)
        w9 = (0.5, 0.5)
        offsets_norm = [w1, w2, w3, w4, w1, w5, w6, w7, w1, w2, w8, w5, w1, w4, w9, w7]
        # Scale the normalized coordinates to actual span
        offsets = [(self.scale*point[0], self.scale*point[1]) for point in offsets_norm]
        # Create WGS84 waypoints around the POI
        waypoints = [self.offset_waypoint(poi,offset) for offset in offsets]
        # Create mission items for each waypoint coordinate pair
        for pair in waypoints:
            wp = self.create_waypoint(pair[0], pair[1], altitude, self.seq_no)
            pattern.append(wp)
            self.seq_no = self.seq_no + 1
        # Add a repeat command to the end
        pattern.append(self.create_loop(self.seq_no))
        return pattern

    def create_mission_item(self, lat, lon, alt, cmd_id, seq_no, param1, param2, param3, param4):
        '''Helper method for creating waypoints'''
        if self.debug: print("Entered mission item creator")
        w = mavutil.mavlink.MAVLink_mission_item_message(
            self.system,  # Set target system
            self.component,  # Set target component
            seq_no,  # Set sequence number
            3,  # Frame type, 0 global (absolute alt), 3 for global_relative (relative alt)
            cmd_id,  # Command ID
            0, 1,  # Current and Autocontinue
            param1,  # Param1
            param2,  # Param2
            param3,  # Param3
            param4,  # Param4
            lat,  # Item latitude
            lon,  # Item longitude
            alt  # Item altitude
        )
        return w

    def create_waypoint(self, lat, lon, alt, seq_no):
        if self.debug: print("Entered waypoint creator")
        w = self.create_mission_item(lat, lon, alt, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, seq_no, 0, 0, 0, 0)
        return w

    def create_ROI(self, lat, lon, alt, seq_no):
        if self.debug: print("Entered ROI creation")
        w = self.create_mission_item(lat, lon, alt, mavutil.mavlink.MAV_CMD_DO_SET_ROI, seq_no, 3, 0, 0, 0)
        return w

    def create_loop(self, seq_no):
        if self.debug: print("Entered loop creator")
        w = self.create_mission_item(
            0,  # no lat needed
            0,  # no lon needed
            0,  # no alt needed
            mavutil.mavlink.MAV_CMD_DO_JUMP,  # the jump cmd_id
            seq_no,  # the sequence number for this mission item
            1,  # jump to the first waypoint
            -1,  # jump unlimited times
            0,  # N/A param
            0,  # N/A param
        )
        return w

    def offset_waypoint(self, poi, offset):
        '''move a waypoint coordinate pair by a given offset in 2D'''
        if self.debug: print("Entered offset applicator")
        #  poi is an [lat, lon] array
        #  offset is an [offset_lat, offset_lon] array in meters
        (utm_lat, utm_lon, num, let) = utm.from_latlon(poi[0], poi[1])
        new_poi = utm.to_latlon(utm_lat+offset[0], utm_lon+offset[1], num, let)
        return new_poi