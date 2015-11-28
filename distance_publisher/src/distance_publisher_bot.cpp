#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<geometry_msgs/Pose2D.h>
#include<math.h>
#include<distance_publisher/IrDistance.h>
#include<ras_arduino_msgs/ADConverter.h>





class ir_distance_publisher
{

public:

    ros::NodeHandle n_;
    
    ros::Subscriber ir_sub_;
    
    ros::Publisher distance_pub_;
    
    
    distance_publisher::IrDistance ir_distance_;
    
    
    
    ir_distance_publisher()
    {

        n_ = ros::NodeHandle("n");
        ir_sub_= n_.subscribe("/arduino/adc",10,&ir_distance_publisher::ir_callback, this);
        distance_pub_ = n_.advertise<distance_publisher::IrDistance>("/ir/distance",10);
   	
    }


    
    void ir_callback(ras_arduino_msgs::ADConverter adc)
    {
    
        int left_front_channel = adc.ch1;

        int left_back_channel = adc.ch2;

        int right_front_channel = adc.ch3;

        int right_back_channel = adc.ch4;

        int front_left_channel = adc.ch5;

        int front_right_channel = adc.ch6;

        ir_distance_.front_left_distance=3.158*0.000001 * pow(front_left_channel, 2) - 0.003395*pow(front_left_channel,1) + 1.014;

        if(ir_distance_.front_left_distance>0.8||ir_distance_.front_left_distance<0.1)
        {
            ir_distance_.front_left_distance=0.8;
        }


        ir_distance_.front_right_distance=3.158*0.000001 * pow(front_right_channel, 2) - 0.003395*pow(front_right_channel,1) + 1.014;

        if(ir_distance_.front_right_distance>0.8||ir_distance_.front_right_distance<0.1)
        {
            ir_distance_.front_right_distance=0.8;
        }


        ir_distance_.left_front_distance = 3.158*0.000001 * pow(left_front_channel, 2) - 0.003395*pow(left_front_channel,1) + 1.014;


        if(ir_distance_.left_front_distance>0.8||ir_distance_.left_front_distance<0.1)
        {
            ir_distance_.left_front_distance=0.8;
        }

        ir_distance_.left_back_distance = 3.158*0.000001 * pow(left_back_channel, 2) - 0.003395*pow(left_back_channel,1) + 1.014;


        if(ir_distance_.left_back_distance>0.8||ir_distance_.left_back_distance<0.1)
        {
            ir_distance_.left_back_distance=0.8;
        }


        ir_distance_.right_front_distance = 3.158*0.000001 * pow(right_front_channel, 2) - 0.003395*pow(right_front_channel,1) + 1.014;


        if(ir_distance_.right_front_distance>0.8||ir_distance_.right_front_distance<0.1)
        {
            ir_distance_.right_front_distance=0.8;
        }


        ir_distance_.right_back_distance = 3.158*0.000001 * pow(right_back_channel, 2) - 0.003395*pow(right_back_channel,1) + 1.014;


        if(ir_distance_.right_back_distance>0.8||ir_distance_.right_back_distance<0.1)
        {
            ir_distance_.right_back_distance=0.8;
        }

        distance_pub_.publish(ir_distance_);
    	
    }





};




int main( int argc, char** argv)
{


    ros::init(argc, argv, "distance_publisher");

    ir_distance_publisher dpub;
    
    ros::Rate loop_rate(10);

    while(dpub.n_.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;



}
