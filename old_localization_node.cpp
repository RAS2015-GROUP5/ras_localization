#include"localization_node.h"


localization_node::localization_node()
{
    //Node inits

    n_ = ros::NodeHandle("n");




    //Publishers

    points_pub_ = n_.advertise<visualization_msgs::MarkerArray>( "/visualization/particles", 0 ); //topic name to be changed

    localization_pub_ = n_.advertise<geometry_msgs::Pose2D>("/localization/encoder",1);

    vis_pub_ = n_.advertise<visualization_msgs::Marker>( "/visualization_marker", 0 ); //topic name to be changed

    distance_pub_ = n_.advertise<std_msgs::Float64>("/front/distance",1);

    front_adc_pub_ = n_.advertise<std_msgs::Int64>("/front/adc",1);



    //Subscribers

    encoder_sub_ = n_.subscribe("/kobuki/encoders",1,&localization_node::encoderCallback, this);
    adc_sub_ = n_.subscribe("/kobuki/adc",1,&localization_node::adcCallback, this);


    //Constants and initializations


    std::random_device rd;
    std::mt19937 gen(rd());

    std::normal_distribution<> gauss(0,0.03);


    degrees_per_tick_ = 1.0;
    epsilon_= 0.001;


    pose_.x = 1.0;
    pose_.y = 3.1;
    pose_.theta = M_PI/2.0;


    b_ = 0.21; //Wheel base
    d_ = 0.1; //Wheel diameter

    sigma_ = 0.0352;

    k = 5.5;

    //Dead Reckoning Section






    //Walls//
    map_segments[0].x1 = 0;map_segments[0].y1=0; map_segments[0].x2=372;map_segments[0].y2=0; //Correct
    map_segments[1].x1 = 0;map_segments[1].y1=66; map_segments[1].x2=0;map_segments[1].y2=366; //Correct
    map_segments[2].x1 = 372;map_segments[2].y1=0; map_segments[2].x2=372;map_segments[2].y2=366; //Correct
    map_segments[3].x1 = 0;map_segments[3].y1=366; map_segments[3].x2=372;map_segments[3].y2=366; //Correct

    //Interior free walls//
    map_segments[4].x1 = 54;map_segments[4].y1=249; map_segments[4].x2=118.5;map_segments[4].y2=249; //Correct
    map_segments[5].x1 = 54;map_segments[5].y1=249; map_segments[5].x2=54;map_segments[5].y2=190.5; //Correct
    map_segments[6].x1 = 118.5;map_segments[6].y1=249; map_segments[6].x2=118.5;map_segments[6].y2=190.5; //Correct


    //Interior Walls//


    map_segments[7].x1 = 0;map_segments[7].y1=66; map_segments[7].x2=300;map_segments[7].y2=66; //Correct
    map_segments[8].x1 = 120;map_segments[8].y1=66; map_segments[8].x2=120;map_segments[8].y2=132; //Correct
    map_segments[9].x1 = 240;map_segments[9].y1=44; map_segments[9].x2=240;map_segments[9].y2=315; //Correct
    map_segments[10].x1 = 372;map_segments[10].y1=133.5; map_segments[10].x2=312;map_segments[10].y2=133.5; //Correct
    map_segments[11].x1 = 372;map_segments[11].y1=265.5; map_segments[11].x2=312;map_segments[11].y2=265.5; //Correct
    map_segments[12].x1 = 300;map_segments[12].y1=199.5; map_segments[12].x2=240;map_segments[12].y2=199.5; //Correct
    map_segments[13].x1 = 180;map_segments[13].y1=195; map_segments[13].x2=240;map_segments[13].y2=195; //Correct
    map_segments[14].x1 = 180;map_segments[14].y1=276; map_segments[14].x2=240;map_segments[14].y2=276; //Correct

    /*
    for(i=0;i<=14;i++) //For running on the real robot;
    {
        map_segments[i].x1 = map_segments[i].x1/1.5;
        map_segments[i].y1 = map_segments[i].y1/1.5;
        map_segments[i].x2 = map_segments[i].x2/1.5;
        map_segments[i].y2 = map_segments[i].y2/1.5;
    }
*/

    for(i=0;i<=14;i++) //I Messed up the map coordinates, converting to m;
    {
        map_segments[i].x1 = map_segments[i].x1/100; //metres
        map_segments[i].y1 = map_segments[i].y1/100; //metres
        map_segments[i].x2 = map_segments[i].x2/100; //metres
        map_segments[i].y2 = map_segments[i].y2/100; //metres
    }


    //Particle filter section



    weights_ = std::vector<double> (1000,0);
    distance_ = -1;


    //Marker section//



    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;



    point.header.frame_id = "base_link";
    point.header.stamp = ros::Time();
    point.ns = "particle";
    point.type = visualization_msgs::Marker::CUBE;
    point.action = visualization_msgs::Marker::ADD;


    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;



    point.scale.y = 0.01;
    point.scale.x = 0.01;
    point.scale.z = 0.01;
    point.color.a = 1.0;
    
    point.color.g = 1.0;
    


    all_points.markers.resize(1000);



    for (int i=0;i<=999;i++)
    {

        all_points.markers[i].header.frame_id = "base_link";
        all_points.markers[i].header.stamp = ros::Time();
        all_points.markers[i].ns = "my_namespace";
        all_points.markers[i].id = i;
        all_points.markers[i].type = visualization_msgs::Marker::CUBE;
        all_points.markers[i].action = visualization_msgs::Marker::ADD;

        all_points.markers[i].pose.position.x = gauss(gen);


        all_points.markers[i].pose.position.y = gauss(gen);
        all_points.markers[i].pose.position.z = 0;

        all_points.markers[i].pose.orientation.x = 0;
        all_points.markers[i].pose.orientation.y = 0;
        all_points.markers[i].pose.orientation.z = 0;
        all_points.markers[i].pose.orientation.w = 1;

        all_points.markers[i].scale.x = 0.001;
        all_points.markers[i].scale.y = 0.001;
        all_points.markers[i].scale.z = 0.001;

        all_points.markers[i].color.g = 1.0;
        all_points.markers[i].color.a = 1.0;

        weights_[i] = 1/1000.0;


    }







    
    











}


double localization_node::measurement_probability(double distance, double true_distance, double sensor_sigma)
{
    return (1.0/(sensor_sigma*sqrt(2.0*M_PI)))*exp(-((distance-true_distance)*(distance-true_distance))/(2*sensor_sigma*sensor_sigma));
}


void localization_node::adcCallback(const ras_arduino_msgs::ADConverter adc)
{
    front_adc_value_.data = adc.ch1;

    distanceConverter(adc.ch1);


}

void localization_node::encoderCallback(const ras_arduino_msgs::Encoders encodermsg)
{

    //Delta Calculations
    delta_sl_ = ((M_PI*d_)*encodermsg.delta_encoder1)/360.0; //Meters
    delta_sr_ = ((M_PI*d_)*encodermsg.delta_encoder2)/360.0; //Meters

    delta_s_ = (delta_sl_+ delta_sr_)/2.0; //Meters

    delta_theta_ = ((delta_sr_ - delta_sl_)/b_); //Degrees?

    delta_x_ = delta_s_*cos(pose_.theta + (delta_theta_/2.0));

    delta_y_ = delta_s_*sin(pose_.theta + (delta_theta_/2.0));

    //Pose Calculations




    //Update

    pose_.x = pose_.x + delta_x_; //Changed from 0.1

    pose_.y = pose_.y + delta_y_;

    pose_.theta = (pose_.theta + delta_theta_); //Radians

    pose_.theta = fmod(pose_.theta,2*3.14159); //Modulus to get values in 0 - 2 pi range



    //Marker update
    marker.pose.position.x = pose_.x;
    marker.pose.position.y = pose_.y;





    //Particle motion update

    for (int i=0;i<=999;i++)
    {
        all_particles_[i].x = all_particles_[i].x + cos(all_particles_[i].theta)*delta_x_;
        all_particles_[i].y = all_particles_[i].y + sin(all_particles_[i].theta)*delta_x_;
    }










}

void localization_node::normalize_weights()
{
    double sum_weights = 0;
    for(int i = 0; i<=1000 ; i++)
    {
        sum_weights = sum_weights + weights_[i];
    }

    for(int i = 0; i<=1000 ; i++)
    {
        normalized_weights_[i] = weights_[i]/sum_weights;
    }



}


double localization_node::calculate_weights()
{
    for(int i=0;i<=1000;i++)
    {
        double pred_distance = getClosestWallDistance(all_particles_[i].x,all_particles_[i].y,all_particles_[i].theta);
        weights_[i] = measurement_probability(pred_distance, calculated_distance_, sensor_sigma_);
    }

}

void localization_node::initialize_particles(bool known)
{
    if(known)
    {

    }

    else
    {
        for (int i=0;i<=num_particles_;i++)
        {
            int rand_theta = (rand()%628);
            all_particles_[i].x = (rand()%250)/100.0;
            all_particles_[i].y = (rand()%250)/100.0;
            all_particles_[i].theta = rand_theta/100.0;
        }
    }
}

void localization_node::Resample()
{

    int index = rand()%1000;

    double beta = 0;

    double max = (*std::max_element(weights_.begin(),weights_.end()));

    mypoint new_all_particles_[1000];


    for (int i=0;i<=999;i++)
    {

        beta = beta + (rand()*int((2*max*1000)))/1000.0;

        while (beta > weights_[i])
        {
            beta = beta - weights_[i];
            index = (index + 1)%1000 ;
        }

        new_all_particles_[i] = all_particles_[index];

        //append p[index];


    }

    for (int i=0;i<=999;i++)
    {
        all_particles_[i] = new_all_particles_[i];
    }

}


void localization_node::update()
{
    
    distance_ = getClosestWallDistance(pose_.x,pose_.y,pose_.theta);

    distance_to_front_.data = distance_;
    localization_pub_.publish(pose_);
    vis_pub_.publish(marker);
    points_pub_.publish(all_points);

    distance_pub_.publish(distance_to_front_);

    front_adc_pub_.publish(front_adc_value_);

    ROS_INFO("True Distance:");
    ROS_INFO("%f",true_distance_to_front_.data);

    ROS_INFO("\n \n");

    ROS_INFO("Calculated Distance:");
    ROS_INFO("%f",distance_to_front_.data);



}


void localization_node::distanceConverter(int adc)
{
    calculated_distance_= 0.000001376*adc*adc -0.0002367*adc + 0.9025;

    if(calculated_distance_>0.8)
    {
        calculated_distance_ = 100;
    }

    if(calculated_distance_<0.1)
    {
        calculated_distance_ = -1;
    }

    true_distance_to_front_.data = calculated_distance_;



}



double localization_node::getWallDistance(double x1, double y1, double theta1, localization_node::segment s)
{
    double m1 = tan(theta1);
    double c1 = y1-(m1*x1);


    double x2 = x1 + 0.1;
    double y2 = m1*x2 + c1;

    double A1 = y2-y1;
    double B1 = x1-x2;
    double C1 = A1*x1 + B1*y1;


    double A2 = s.y2 - s.x2;
    double B2 = s.x1 - s.x2;
    double C2 = A2*s.x1 + B2*s.y1;

    double det = A1*B2 - A2*B1;

    double x;
    double y;



    if (det==0)
    {
        return -1;
    }

    else
    {
        x = (B2*C1 - B1*C2);
        y = (A1*C2 - A2*C1);

        double dist = sqrt(pow(x-x1,2)+pow(y-y1,2));
        return dist;

        //Need to check whether the point lies on the segment;
    }





}


double localization_node::getClosestWallDistance(double x, double y, double theta)
{
    double min = 999;
    double new_min = 0;

    for(i=0;i<=14;i++)
    {
        new_min = getWallDistance(x, y, theta, map_segments[i]);

        if(new_min<min)
        {
            min=new_min;
        }

        if(new_min==0)
        {
            //return -1.0f;
        }

    }

    return 555.0;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "localization_node");

    localization_node l_node;

    ros::Rate loop_rate(10);

    
    while(l_node.n_.ok())
    {


        l_node.update();
        
        ros::spinOnce();
        
        loop_rate.sleep();

    }


    return 0;
}
