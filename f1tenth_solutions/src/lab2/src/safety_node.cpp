
#include <limits>
#include <numeric>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>

// ROS header messages
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

const double car_deceleration_safety_margin = 2.5;
const double car_deceleration = 8.26 - car_deceleration_safety_margin; // m/s^2
const double car_width = 0.4; // m

class Safety {

    public:
        Safety() : brake_state( false ), speed( 0 ), seq(0), n( ros::NodeHandle() ) {

            /*
            One publisher should publish to the /brake topic with an
            ackermann_msgs/AckermannDriveStamped brake message.

            One publisher should publish to the /brake_bool topic with a
            std_msgs/Bool message.

            You should also subscribe to the /scan topic to get the
            sensor_msgs/LaserScan messages and the /odom topic to get
            the nav_msgs/Odometry messages

            The subscribers should use the provided odom_callback and 
            scan_callback as callback methods

            NOTE that the x component of the linear velocity in odom is the speed
            */

            brake_state_publisher = n.advertise<std_msgs::Bool>("/brake_bool", 1);
            brake_publisher = n.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1);

            laser_scan_subscriber = n.subscribe( "/scan", 1, &Safety::scan_callback, this );
            odometry_subscriber = n.subscribe( "/odom", 1, &Safety::odom_callback, this );

        }
        void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
            speed = odom_msg->twist.twist.linear.x;
        }

        void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
            // TODO: calculate TTC

            // TTC is calculated by dividing the measured distance between the two objects (the vehicle and the walls) 
            // by the relative velocity of the vehicle (since the walls don't move)
            
            int num_readings = int( ( scan_msg->angle_max - scan_msg->angle_min ) / scan_msg->angle_increment ) + 1;

            // construct the list of angles - probably only need to do this once, but for now
            // (and simplicity) 
            std::vector<double> angle_offsets( num_readings );
            std::iota( angle_offsets.begin(), angle_offsets.end(), 0 );
            Eigen::ArrayXd angles = Eigen::Map< Eigen::Array<double,Eigen::Dynamic,1> >(angle_offsets.data(), num_readings);
            angles *= scan_msg->angle_increment;
            angles += scan_msg->angle_min;
            
            double time_to_collision;

            if( speed == 0 ) {
                // Not moving -- TTC is inf by defn
                time_to_collision = std::numeric_limits<double>::infinity();
                brake_state = false;
            } else {
                double sign = 1;
                double s = speed;
                if( speed < 0 ) {
                    sign = -1;
                    s = abs( speed );
                }

                Eigen::Array<double,Eigen::Dynamic,1> ranges(num_readings);
                for( int i = 0; i < num_readings; i++ ) {
                    ranges[i] = double( scan_msg->ranges[i] );
                }

                // probably don't need to explicitly break out the sign here since we are effectively
                // assuming that the forward angles (which are positive cos() values) correspond with 
                // positive speed and the time to to collision is positive.  If the speed calculation
                // was relative, then we'd need to reverse the sign since the targets are static and
                // the relative velocity (V_wall -  V_car) is negative. 
                Eigen::ArrayXd ttc_data = sign * ranges / ( s * angles.cos() );

                // taking all of the raw values results in far too many false positives when we are
                // passing close to a wall -- find the minimum distance for each angle in a corridor
                // centered on the car

                Eigen::ArrayXd max_ranges = car_width / (2.0 * angles.sin().abs());

                for( int i = 0; i < num_readings; i++ ) {
                    double max_range_at_angle = max_ranges[i];
                    if( isnan( max_range_at_angle ) ) {
                        max_range_at_angle = scan_msg->range_max;
                    }

                    if( ttc_data[i] < 0 || ranges[i] >= max_range_at_angle ) {
                        // negative times and contacts outside the corridor can be ignored
                        ttc_data[i] = std::numeric_limits<double>::infinity();
                    }
                }
                time_to_collision = ttc_data.minCoeff();
                double time_to_collision_threshold = s / car_deceleration;

                if( time_to_collision < time_to_collision_threshold ) {
                    if( brake_state == false ) {
                        ROS_INFO( "Engaging brake -- V: %f, TTC: %f", speed, time_to_collision );
                    }
                    brake_state = true;

                    // publish the zero velocity/brakes on message
                    ackermann_msgs::AckermannDriveStamped msg = {};
                    msg.header.seq = ++seq;
                    msg.header.frame_id = "";
                    msg.header.stamp = ros::Time::now();

                    msg.drive.speed = 0;

                    brake_publisher.publish( msg );

                } else {
                    brake_state = false;
                }
            }
            // publish the current brake state
            std_msgs::Bool state_msg = {};
            state_msg.data = brake_state;
            brake_state_publisher.publish( state_msg );
        }

    // The class that handles emergency braking
private:
    ros::NodeHandle n;

    double speed;
    bool brake_state;
    int seq;

    ros::Publisher brake_publisher;
    ros::Publisher brake_state_publisher;

    ros::Subscriber laser_scan_subscriber;
    ros::Subscriber odometry_subscriber;


};


int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}