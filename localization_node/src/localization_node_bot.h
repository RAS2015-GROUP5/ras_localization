#ifndef LO_H
#define LO_H

#endif // LO_H

#include <ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <ras_arduino_msgs/PWM.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <string>
#include <vector>
#include <std_msgs/Float64.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <std_msgs/Int64.h>
#include <nav_msgs/Odometry.h>
#include <distance_publisher/IrDistance.h>
#include "tf/transform_datatypes.h"
#include<tf/transform_listener.h>
//#include<tf/Quarternion.h>

//#include <geometry_msgs/Quarternion.h>

#define VEL_SPREAD 0.0
#define ROT_SPREAD 0.0

#define N_PARTICLES 1000
#define MAX_X 3.7995
#define MIN_X 0.0

#define MAX_Y 3.60
#define MIN_Y 0.0


class localization_node
{
public:

    double currth,prevx,prevy,prevth,currx,curry;

    double distance_;
    double calculated_distance_fl_;
    double calculated_distance_fr_;
    double calculated_distance_lf_;
    double calculated_distance_lb_;
    double calculated_distance_rf_;
    double calculated_distance_rb_;


    struct mypoint{
        double x;
        double y;
        double theta;

        mypoint(){
            x = 0.0;
            y = 0.0;
            theta = 0.0;
        }
    };

    ros::NodeHandle n_;

    ros::Publisher localization_pub_;

    ros::Publisher pose_pub_;

    ros::Publisher vis_pub_;

    ros::Publisher right_ir_loc_pub_;

    ros::Publisher left_ir_loc_pub_;

    ros::Subscriber encoder_sub_;

    ros::Subscriber dist_sub_;

    ros::Subscriber velocity_sub_;

    ros::Subscriber odom_sub;

    geometry_msgs::Pose2D pose_;

    int i;

    double mega_delta_x_;
    double mega_delta_y_;
    double mega_delta_theta_;

   double x_offset_; //Sensor offset from center of the robot
   double y_offset_;

   double x_fw_offset_;
   double y_fw_offset_;


    double ipt_x_l_;

    double ipt_y_l_;

    double ipt_x_r_;

    double ipt_y_r_;

    /*Pose variables*/

    /*Deltas*/

    double delta_sl_;
    double delta_sr_;
    double delta_s_;

    double delta_x_;
    double delta_y_;
    double delta_theta_;


    /*Kinematic constants*/

    double b_; //Wheel base
    double d_; //Wheel diameter

    double degrees_per_tick_;

    /*Particle filter variables*/

    double sigma_;

    double epsilon_;

    int callback_counter = 0;


    /*Marker variables*/

    /*TODO rename variables with trailing underscore(_) OR move to another node*/



    visualization_msgs::Marker marker;

    visualization_msgs::Marker int_marker;

    visualization_msgs::Marker right_ir_marker;

    visualization_msgs::Marker left_ir_marker;

    visualization_msgs::MarkerArray all_points;

    geometry_msgs::PoseArray all_poses;

    visualization_msgs::Marker point;

    double weights_[N_PARTICLES];

    double normalized_weights_[N_PARTICLES];

    double rightirx_;
    double rightiry_;

    double leftirx_;
    double leftiry_;

    struct segment{

    double x1;
    double y1;
    double x2;
    double y2;


    };

    tf::StampedTransform lftransform_;
    tf::TransformListener tflistener_lf_;


    tf::StampedTransform lbtransform_;
    tf::TransformListener tflistener_lb_;

    tf::StampedTransform rftransform_;
    tf::TransformListener tflistener_rf_;


    tf::StampedTransform rbtransform_;
    tf::TransformListener tflistener_rb_;


    tf::StampedTransform fwltransform_;
    tf::TransformListener tflistener_fwl_;


    tf::StampedTransform fwrtransform_;
    tf::TransformListener tflistener_fwr_;


    double sensor_sigma_;

    mypoint all_particles_[N_PARTICLES];

    segment map_segments[15];

    int num_particles_;

    double velX_;
    double velTheta_;

    localization_node();

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void encoderCallback(const ras_arduino_msgs::Encoders encodermsg);

    void resample();

    void update();

    double getWallDistance(double x1, double y1, double theta1, segment s);

    double getClosestWallDistance(double x, double y, double theta);
    
    void distCallback(const distance_publisher::IrDistance &dist);

    void velCallback(const geometry_msgs::Twist &twist);

    double measurement_probability(const double, const double,const double);

    double calculate_weights();

    void initialize_particles(bool known=false);

    void normalize_weights();

    int get_line_intersection(double p0_x, double p0_y, double p1_x, double p1_y,
        double p2_x, double p2_y, double p3_x, double p3_y, double *distance, double &intx, double &inty );

    double getClosestWallDistanceFL(double x, double y, double theta);

    double getClosestWallDistanceFR(double x, double y, double theta);

    double getClosestWallDistanceLF(const double x, const double y, const double theta);

    double getClosestWallDistanceLB(const double x, const double y, const double theta);

    double getClosestWallDistanceRF(double x, double y, double theta);

    double getClosestWallDistanceRB(double x, double y, double theta);

    void particle_filter();

    void update_markers();

    void update_particles();

    int sign(double val);

    void printParticles();

};

