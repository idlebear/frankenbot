#!/usr/bin/env python
import rospy
import numpy as np
import pdb

# required messages
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


car_width = 0.30 # m
car_deceleration_safety_margin = 2 # m/s^2 (assuming the car can't really decelerate as fast as promised
car_deceleration = 8.26 - car_deceleration_safety_margin # m/s^2
ttc_threshold = 0.5 # s

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):

        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        self.seq = 0
        self.brake_engaged = False

        self.ebrake_publisher = rospy.Publisher( "/brake", AckermannDriveStamped, queue_size = 1)
        self.ebrake_state_publisher = rospy.Publisher( "/brake_bool", Bool, queue_size = 1)

        self.scan_subscriber = rospy.Subscriber( "/scan", LaserScan, self.scan_callback )
        self.odom_subscriber = rospy.Subscriber( "/odom", Odometry, self.odom_callback )

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x
        if odom_msg.twist.twist.linear.y != 0:
            rospy.loginfo( "Slipping sideways!" )


    def scan_callback(self, scan_msg):
        # TODO: calculate TTC

        # pdb.set_trace()
        speed = self.speed
        if speed != 0:
            ranges = np.array( scan_msg.ranges )

            angles = np.arange( scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment )
            sin_angles = np.where( angles != 0, abs(np.sin(angles)), 0.000000001 )
           
            # calculate the max lengths of interest -- for those that are too far away, mark them as 
            # infinite
            if speed > 0:
                max_ranges = np.where( (angles > -np.pi/2.) & (angles < np.pi/2.), car_width / (2. * sin_angles), 0 )  
            else:
                # car is moving backwards, look at the other half of the ranges
                max_ranges = np.where( (angles < -np.pi/2.) | (angles > np.pi/2.), car_width / (2. * sin_angles), 0 )  
                speed = abs(speed)    

            diffs = max_ranges - ranges
            
            # the only angles we care about are those that are positive, but not nan
            mask = ( diffs > 0 ) & ( ~np.isnan(diffs) ) & ( ~np.isinf(diffs) )
            ttc = np.min( np.where( mask, ranges * abs(np.cos(angles)) / speed, np.inf ) ).astype(float)
        else:
            ttc = np.inf
            speed = 0

        ttc_threshold = speed/car_deceleration

        if ttc < ttc_threshold:
            if not self.brake_engaged:
                rospy.loginfo( "Engaging Brake!  Current Speed: %f, minTTC: %f, threshold: %f", self.speed, ttc, ttc_threshold )
            self.brake_engaged = True

            msg = AckermannDriveStamped()
            msg.header.seq = self.seq
            self.seq += 1
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "";

            msg.drive.speed = 0
            msg.drive.steering_angle = 0
            msg.drive.steering_angle_velocity = 0
            msg.drive.acceleration = 0
            msg.drive.jerk = 0
            self.ebrake_publisher.publish( msg )

        else:
            self.brake_engaged = False

        msg = Bool()
        msg.data = self.brake_engaged
        self.ebrake_state_publisher.publish( msg )

def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()
