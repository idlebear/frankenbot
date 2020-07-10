#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
# from threading import Thread, Lock
from time import sleep

#ROS Imports
import rospy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg  import Quaternion, PoseStamped, Pose, Point
from nav_msgs.msg       import Odometry
from sensor_msgs.msg    import Image, LaserScan
from std_msgs.msg       import Bool, Int32MultiArray
from tf.transformations import quaternion_from_euler

#PID CONTROL PARAMS (indices)
kp = 0
kd = 1
ki = 2
servo_offset = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270. * math.pi / 180. # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters


# Valid car states
PARKED = 0
PARKING = 1
STARTING = 2
STARTED = 3

# Define a transistion time
state_transition_time = 0.5 # seconds

# Angle configuration parameters
zero_angle = 0      # directly left (or right), 90deg to the car's forward motion
forward_angle = 70. * math.pi / 180.  # angle forward of the car
# short_forward_angle = 60. * math.pi / 180.

# speed/angle buckets
high_speed_limit = 10. * math.pi / 180.
high_speed = 2 # m/s
med_speed_limit = 20. * math.pi / 180.
med_speed = 1.5 # m/s
low_speed = 1. # m/s

# max number of scans to process (rough approximation of a time limit)
max_steps = 50000

# planning distance ahead of the car
distance_forward = 0.5 # m

speed_threshold = 0.01
location_threshold = 0.1

min_steering_angle = -math.pi * 2. / 3.
max_steering_angle = math.pi * 2. / 3.

# TTC checking -- should move to its own module
car_width = 0.40 # m
car_deceleration_safety_margin = 2 # m/s^2 (assuming the car can't really decelerate as fast as promised
car_deceleration = 8.26 - car_deceleration_safety_margin # m/s^2


# Starting Pose
start_x = -0.5 
start_y = 0 
start_z = 0
start_heading =  0 # math.pi / 2

def clamp( a, min_val, max_val ):
    return min( max( a, min_val ), max_val )

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):

        self.reset()

        #Topics & Subs, Pubs
        odom_topic = '/odom'
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        # subscribe to odometry so we know what's going on
        self.odom_sub = rospy.Subscriber( odom_topic, Odometry, self.odom_callback )
        # Subscribe to LIDAR
        self.lidar_sub = rospy.Subscriber( lidarscan_topic, LaserScan, self.lidar_callback ) 

        # Publish to drive
        self.drive_pub = rospy.Publisher( drive_topic, AckermannDriveStamped, queue_size = 1 ) 
        
        # PID parameters 
        self.p = [0. for i in range(3) ]

    def reset( self ):
        # initialize state parameters
        self.state = PARKED
        self.state_start_time = rospy.Time.now()

        # initialize PID parameters
        self.prev_error = 0
        self.prev_time = None
        self.integral = 0
        self.diff = 0

        # quality reporting
        self.cumulative_squared_error = 0
        self.count = 0
        self.distance = 0

        self.pose = Pose();
        self.direction = 1.
        self.speed = 0

    def stats( self ):
        rms = math.sqrt( self.cumulative_squared_error / self.count )
        return ( self.distance, self.count, rms )

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #
        # data is a LaserScan message

        desired_angle_difference = (angle - math.pi/2.) - data.angle_min;

        steps = int(desired_angle_difference / data.angle_increment)
        # rospy.loginfo( "Distance for angle -- min: %f, requested: %f, desired: %f", data.angle_min, angle, desired_angle_difference )
        try:
            if not np.isnan( data.ranges[steps] ):
                return data.ranges[steps]
            else:
                return None
        except:
            return None

    def pid_control(self, error, velocity):

        # update PID parameters
        now = rospy.Time.now()
            
        if self.prev_time is not None:
            # Differential update
            self.diff = error - self.prev_error

            # Integral update
            interval_err = error * ( now - self.prev_time ).to_sec()
            self.integral = self.integral + interval_err
            self.cumulative_squared_error += interval_err * interval_err

            # calculate the angle to respond to the measured error -- simple first implementation
            # just multiply the error by the combination of P + I + D
            angle = clamp( self.p[kp] * error + self.p[ki] * self.integral + self.p[kd] * self.diff, min_steering_angle, max_steering_angle )
            if abs(angle) <= high_speed_limit:
                velocity = high_speed
            elif abs(angle) <= med_speed_limit:
                velocity = med_speed
            else:
                velocity = low_speed

            # if self.count % 5 == 0:
            #    rospy.loginfo( "Error: %f, Velocity: %f, Angle: %f", error, velocity, angle )
        
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = angle * self.direction
            drive_msg.drive.speed = velocity * self.direction
            self.drive_pub.publish(drive_msg)

        # update the internal state variables
        self.prev_error = error
        self.prev_time = now
        
    def check_collision( self, data, speed ):
        if speed != 0:
            ranges = np.array( data.ranges )
            angles = np.arange( data.angle_min, data.angle_max, data.angle_increment )
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
            # ranges = ranges * abs( np.cos(angles))
            ttc_index = np.argmin( np.where( mask, ranges * abs( np.cos(angles))/ speed, np.inf ) )
            min_range = ranges[ ttc_index ]
            ttc = min_range * abs(np.cos(angles[ttc_index])) / speed
            # rospy.loginfo( "speed: %f, minTTC: %f, index: %d, minRange: %f", speed, ttc, ttc_index, min_range )
        else:
            ttc = np.inf

        ttc_threshold = 1.5 # speed/car_deceleration

        if ttc < ttc_threshold:
            # rospy.loginfo( "Collision imminent -- Current Speed: %f, minTTC: %f, threshold: %f", self.speed, ttc, ttc_threshold )
            return True

        return False

    def followWall(self, data, desired_left_distance, left_side = True):
        
        error = None

        if left_side:
            direction = -1
            za = math.pi - zero_angle
            fa = math.pi - forward_angle
        else:
            direction = 1
            za = zero_angle
            fa = forward_angle

        zero_dist = self.getRange( data, za )
        forward_dist = self.getRange( data, fa )
        
        # if self.count % 5 == 0:
        #    rospy.loginfo( "Range -- zero: %f, far: %f", zero_dist, forward_dist )
        
        if zero_dist is not None and forward_dist is not None:
            # two good readings - calculate the distance
            alpha = math.atan( (forward_dist * math.cos(forward_angle-zero_angle) - zero_dist) / 
                               forward_dist * math.sin(forward_angle-zero_angle) )
            wall_dist = zero_dist * math.cos( alpha )

            # project the distance from the wall forward in time, assuming we're still travelling
            # at 2 m/s, and we want to know where we'll be in 1 sec (arbitrary, I know)
            projected_wall_dist = wall_dist + distance_forward * math.sin( alpha )
            far_error = desired_left_distance - projected_wall_dist
            # rospy.loginfo( "far error: %f", far_error )

            return direction * far_error

        else:
            # other conditions we aren't accounting for right now -- could be an open
            # angle such that there's no forward reading -- assume a corner?  
            #
            # For now, return None to indicate failure
            rospy.loginfo( "Unable to calculate angles.  Forward: %f -  side: %f", forward_dist, zero_dist )

        return None 

    def odom_callback( self, data ):
        now = rospy.Time.now()

        dx = self.pose.position.x - data.pose.pose.position.x
        dy = self.pose.position.y - data.pose.pose.position.y
        self.distance += math.sqrt( dx*dx + dy*dy )

        self.pose = data.pose.pose
        # rospy.loginfo( "X: %f, Y: %f", self.pose.position.x, self.pose.position.y )

        self.speed = data.twist.twist.linear.x
        if self.state == PARKED:
            if self.speed > speed_threshold:
                self.change_state( STARTING, now )

        elif self.state == PARKING:
            if (self.state_start_time - now).to_sec() > state_transition_time:
                if self.speed > speed_threshold:
                    self.change_state( STARTING, now )
                else:
                    self.change_state( PARKED, now )

        elif self.state == STARTING:
            if (self.state_start_time - now).to_sec() > state_transition_time:
                if self.speed <= speed_threshold:
                    self.change_state( PARKING, now )
                else:
                    self.change_state( STARTED, now )

        elif self.state == STARTED:
            if self.speed <= speed_threshold:
                self.change_state( PARKING, now )


    def change_state( self, new_state, start_time ):
        self.state = new_state
        self.state_start_time = start_time

    def lidar_callback(self, data):
        """
        Handle the scan message -- if the car is currently stopped, this message can be ignored 
        """
        if self.state == STARTING or self.state == STARTED:
            self.count += 1
            error = self.followWall( data, DESIRED_DISTANCE_LEFT, left_side = True )
            
            # collision_imminent = self.check_collision( data, self.speed )
            # if collision_imminent:
            #     self.direction *= -1.

            #send error to pid_control
            if error is not None:
                self.pid_control(error, VELOCITY)

class Controller:

    def __init__( self, follower ):
        self.mux_pub = rospy.Publisher( "/mux", Int32MultiArray, queue_size = 1 )   
        self.pose_pub = rospy.Publisher( "/pose", PoseStamped, queue_size = 1 )
        self.follower = follower
        self.seq = 0

        self.running = False

        self.collision_sub = rospy.Subscriber( "/collision", Bool, self.collision_callback )

    def reset( self ):
        # turn everything off
        msg = Int32MultiArray()
        msg.data = [ 0, 0, 0, 0, 0 ]
        self.mux_pub.publish(msg)

        # move the car
        pose_msg = PoseStamped()
        pose_msg.header.seq = self.seq
        self.seq += 1
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = rospy.Time.now()

        pose_msg.pose.position.x = start_x
        pose_msg.pose.position.y = start_y
        pose_msg.pose.position.z = start_z
        pose_msg.pose.orientation = Quaternion( *quaternion_from_euler( 0, 0, start_heading ) )

        while True:
            dx = (self.follower.pose.position.x - start_x)
            dy = (self.follower.pose.position.y - start_y)
            dist = math.sqrt( dx*dx + dy*dy)
            if dist <= location_threshold:
                break
            self.pose_pub.publish(pose_msg)
            # rospy.loginfo( "Waiting for move from x: %f, y: %f -- movng to: x: %f, y: %f -  dist: %f", self.follower.pose.position.x, self.follower.pose.position.y, start_x, start_y, dist )
            sleep( 0.5 )
            dist = math.sqrt( dx*dx + dy*dy)

            # BUGBUG -- add a timeout here to allow for failure...

        # reset internal variables
        self.follower.reset()

    def start( self, p ):
        self.follower.p = p

        msg = Int32MultiArray()
        msg.data = [ 0, 0, 0, 0, 1 ]   # nav is currently the 5th index/entry
        self.mux_pub.publish(msg)

        self.follower.state = STARTING
        self.running = True

    def stop( self ):
        msg = Int32MultiArray()
        msg.data = [ 0, 0, 0, 0, 0 ]   # nav is currently the 5th index/entry
        self.mux_pub.publish(msg)

        self.follower.state = PARKED
        self.running = False

    def collision_callback( self, data ):
        # collsion -- update internal state
        self.running = False
        self.follower.state = PARKING
        self.stop()

    def wait( self ):
        # simple poll for now...
        while self.running:
            rospy.sleep( 0.1 )
            if self.follower.count > max_steps:
                # that's far enough
                self.stop()

    def run_test( self, p ):
        self.reset()
        self.start( p )
        self.wait()
        self.stop()
        (distance, count, rms) = self.follower.stats()
        rospy.loginfo( "Test Complete -- Distance: %f, Steps: %f, E_rms: %f", distance, count, rms )
        return -distance 

    def twiddle_test( self ):
        # making things a little more complicated here -- automating the twiddle testing
        # to find optimal parameters for driving via PID
        #
        # Steps -- 
        # 1. Ensure MUX is off
        # 2. Move car to start position
        #     a. wait for odometry to confirm
        # 3. Start test 
        # 4. Wait for finish (collision or timeout)
        # 5. Collect distance and cumulative error for comparison (have to decide which to prioritize but likely 
        #    distance)
    
        p = [ 0, 0, 0 ]
        dp = [1, 1, 1]
    
        interval_increment = 1.1
        interval_decrement = 0.9
    
        min_error = self.run_test( p )
        while sum(dp) > 0.01:
            for i in range( len(p) ):
                p[i] += dp[i]
                err = self.run_test( p )
                if err < min_error:
                    min_error = err 
                    dp[i] *= interval_increment
                else:
                    # no improvement up, try down
                    p[i] -= 2 * dp[i]
                    err = self.run_test( p )
                    if err < min_error:
                        min_error = err 
                        dp[i] *= interval_increment
                    else:
                        # no improvement in either direction, try decreasing the step size
                        p[i] += dp[i]
                        dp[i] *= interval_decrement
            rospy.loginfo( "Candidate Solution: [%f, %f, %f] with best error: %f and dp: %f", p[0], p[1], p[2], -min_error, sum(dp) )
    
    
    def increasing( self, p, index, inc, steps ):
        min_error = self.run_test( p )
        start = p[index]
        for i in range(1, steps):
            p[index] = start + i * inc
    
            err = self.run_test( p )
            if err < min_error:
                best_inc = i
                min_error = err
            rospy.loginfo( "Candidate Solution: [%f, %f, %f] with best error: %f", p[0], p[1], p[2], -min_error )

    def single( self, p ):
        min_error = self.run_test( p )
        rospy.loginfo( "Candidate Solution: [%f, %f, %f] with best error: %f", p[0], p[1], p[2], -min_error )

            

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)

    controller = Controller( wf )
    # controller.increasing( [0,0,0], 0, 0.5, 100 )
    controller.twiddle_test()
    # controller.single( [9,0,0] )

    rospy.signal_shutdown( "Tests complete" )
    rospy.spin()


if __name__=='__main__':
	main(sys.argv)
