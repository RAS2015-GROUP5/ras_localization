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
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <string>
#include <vector>
#include <std_msgs/Float64.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <std_msgs/Int64.h>



class localization_node
{
public:

    std_msgs::Float64 distance_to_front_;

    double distance_;
    double calculated_distance_;

    std_msgs::Float64 true_distance_to_front_;

    struct mypoint{
        float x;
        float y;
        float theta;
    };

    ros::NodeHandle n_;
    ros::Publisher localization_pub_;

    ros::Publisher distance_pub_;

    ros::Publisher front_adc_pub_;

    ros::Publisher points_pub_;

    ros::Publisher vis_pub_;

    ros::Subscriber encoder_sub_;

    ros::Subscriber adc_sub_;

    geometry_msgs::Pose2D pose_;

    int i;

    int point_id =0;

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

    //geometry_msgs::Point p;




    /*Marker variables*/

    /*TODO rename variables with trailing underscore(_) OR move to another node*/



    visualization_msgs::Marker marker;

    visualization_msgs::MarkerArray all_points;

    visualization_msgs::Marker point;



    std::vector<double> weights_;
    std::vector<double> normalized_weights_;

    float k;



    struct segment{

    double x1;
    double y1;
    double x2;
    double y2;


    };

    std_msgs::Int64 front_adc_value_;

    double sensor_sigma_;

    mypoint all_particles_[1001];


    segment map_segments[15];

    int num_particles_;


    localization_node();

    void encoderCallback(const ras_arduino_msgs::Encoders encodermsg);

    void Resample();

    void update();

    void distanceConverter(int adc);

    double getWallDistance(double x1, double y1, double theta1, segment s);

    double getClosestWallDistance(double x, double y, double theta);

    void adcCallback(const ras_arduino_msgs::ADConverter);

    double measurement_probability(double, double , double);

    double calculate_weights();

    void initialize_particles(bool known=false);

    void normalize_weights();





};

