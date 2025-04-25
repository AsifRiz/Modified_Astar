import rospy
import time
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import Header

class DroneController:
    def __init__(self):
        rospy.init_node('drone3_control_script')
        
        # Create publisher and service clients
        self.local_position_publisher = rospy.Publisher('/drone3/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.arming_client = rospy.ServiceProxy('/drone3/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/drone3/mavros/set_mode', SetMode)
        self.takeoff_client = rospy.ServiceProxy('/drone3/mavros/cmd/takeoff', CommandTOL)
        
        # Drone 3 waypoints from waypoints.txt
        self.drone_waypoints = [
            (8.0, 10.0, 10), (9.0, 10.0, 10), (10.0, 10.0, 10), (11.0, 10.0, 10), (12.0, 10.0, 10),
            (13.0, 10.0, 10), (14.0, 9.0, 10), (15.0, 8.0, 10), (16.0, 8.0, 10), (17.0, 8.0, 10),
            (18.0, 8.0, 10), (19.0, 8.0, 10), (20.0, 8.0, 10), (21.0, 8.0, 10), (22.0, 8.0, 10),
            (23.0, 7.0, 10), (24.0, 7.0, 10), (25.0, 7.0, 10), (26.0, 7.0, 10), (27.0, 7.0, 10),
            (28.0, 7.0, 10), (29.0, 7.0, 10), (30.0, 7.0, 10), (31.0, 7.0, 10), (32.0, 7.0, 10),
            (33.0, 7.0, 10), (37.0, 8.5, 10), (38.0, 8.5, 10), (39.0, 8.5, 10), (35.2, 6.0, 10),
            (36.2, 5.0, 10), (37.2, 4.0, 10), (38.2, 4.0, 10), (39.2, 4.0, 10), (40.2, 3.0, 10),
            (41.2, 3.0, 10), (42.2, 3.0, 10), (43.2, 3.0, 10), (44.2, 3.0, 10), (48.0, 4.0, 10),
            (49.0, 5.0, 10), (50.0, 6.0, 10), (51.0, 7.0, 10), (51.0, 8.0, 10), (51.0, 9.0, 10),
            (52.0, 10.0, 10), (52.0, 11.0, 10), (52.0, 12.0, 10), (53.0, 13.0, 10), (54.0, 14.0, 10),
            (55.0, 15.0, 10), (56.0, 16.0, 10), (57.0, 17.0, 10), (58.0, 18.0, 10), (59.0, 19.0, 10),
            (60.0, 20.0, 10), (61.0, 21.0, 10), (62.0, 22.0, 10), (63.0, 23.0, 10), (64.0, 24.0, 10),
            (65.0, 25.0, 10), (66.0, 26.0, 10), (67.0, 27.0, 10), (68.0, 28.0, 10), (69.0, 29.0, 10),
            (70.0, 24.0, 10), (67.0, 28.0, 10), (68.0, 29.0, 10), (69.0, 30.0, 10), (70.0, 31.0, 10),
            (71.0, 32.0, 10), (72.0, 33.0, 10), (73.0, 34.0, 10), (74.0, 35.0, 10), (75.0, 36.0, 10),
            (76.0, 37.0, 10), (77.0, 38.0, 10), (78.0, 39.0, 10), (79.0, 40.0, 10), (80.0, 41.0, 10),
            (81.0, 42.0, 10), (82.0, 43.0, 10), (83.0, 44.0, 10), (84.0, 45.0, 10), (85.0, 46.0, 10),
            (86.0, 47.0, 10), (87.0, 48.0, 10), (88.0, 49.0, 10), (89.0, 50.0, 10), (90.0, 51.0, 10),
            (91.0, 52.0, 10), (92.0, 53.0, 10), (93.0, 54.0, 10), (94.0, 55.0, 10), (98.0, 57.5, 10),
            (98.0, 58.5, 10), (99.0, 59.5, 10), (99.0, 60.5, 10), (99.0, 61.5, 10), (99.0, 62.5, 10),
            (99.0, 63.5, 10), (99.0, 64.5, 10), (98.0, 65.5, 10), (94.0, 65.0, 10), (94.0, 66.0, 10),
            (93.0, 67.0, 10), (93.0, 68.0, 10), (93.0, 69.0, 10), (93.0, 70.0, 10), (93.0, 71.0, 10),
            (93.0, 72.0, 10), (93.0, 73.0, 10), (93.0, 74.0, 10), (93.0, 75.0, 10), (93.0, 76.0, 10),
            (93.0, 77.0, 10), (92.0, 78.0, 10), (92.0, 79.0, 10), (92.0, 80.0, 10), (91.0, 81.0, 10),
            (91.0, 82.0, 10), (90.0, 83.0, 10), (90.0, 84.0, 10), (89.0, 85.0, 10), (89.0, 86.0, 10),
            (88.0, 87.0, 10)
        ]

    def set_flight_mode(self, mode):
        rospy.wait_for_service('/drone3/mavros/set_mode')
        try:
            mode_response = self.set_mode_client(custom_mode=mode)
            return mode_response.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def arm_drone(self):
        rospy.wait_for_service('/drone3/mavros/cmd/arming')
        try:
            arm_response = self.arming_client(True)
            return arm_response.success
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def takeoff_drone(self, altitude):
        rospy.wait_for_service('/drone3/mavros/cmd/takeoff')
        try:
            takeoff_response = self.takeoff_client(altitude=altitude)
            return takeoff_response.success
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def move_drone(self, x, y, z, yaw=0):
        position_target = PositionTarget()
        position_target.header = Header()
        position_target.header.stamp = rospy.Time.now()
        position_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        position_target.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | 
                                   PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX | 
                                   PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | 
                                   PositionTarget.FORCE | PositionTarget.IGNORE_YAW_RATE)
        position_target.position.x = x
        position_target.position.y = y
        position_target.position.z = z
        position_target.yaw = yaw
        self.local_position_publisher.publish(position_target)

    def follow_waypoints(self):
        rate = rospy.Rate(10)  # 10 Hz
        rospy.loginfo("Following waypoints...")
        
        for waypoint in self.drone1_waypoints:
            if rospy.is_shutdown():
                break
                
            x, y, z = waypoint
            self.move_drone(x, y, z)
            rospy.loginfo(f"Moving to waypoint: ({x}, {y}, {z})")
            rate.sleep()
            
            # Wait until we reach the waypoint (simplified - in real implementation you'd check position)
            rospy.sleep(2)

    def run(self):
        try:
            # Wait for MAVROS to be ready
            rospy.sleep(1)
            
            # Set to GUIDED mode
            rospy.loginfo("Setting to GUIDED mode...")
            self.set_flight_mode('GUIDED')
            rospy.sleep(2)
            
            # Arm the drone
            rospy.loginfo("Arming drone...")
            self.arm_drone()
            rospy.sleep(2)
            
            # Takeoff to 10 meters
            rospy.loginfo("Taking off...")
            self.takeoff_drone(10)
            rospy.sleep(10)  # Wait for takeoff to complete
            
            # Follow waypoints
            self.follow_waypoints()
            
            # Land (simplified - in real implementation you'd use a landing service)
            rospy.loginfo("Mission complete, landing...")
            self.move_drone(0, 0, 0)  # Move to origin at ground level
            rospy.sleep(5)
            
        except rospy.ROSInterruptException:
            rospy.loginfo("Mission interrupted")

if __name__ == '__main__':
    controller = DroneController()
    controller.run()
