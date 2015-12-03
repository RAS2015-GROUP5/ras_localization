#include"localization_node_bot.h"


localization_node::localization_node()
{

    //Node inits

    n_ = ros::NodeHandle("n");

    //Publishers

    localization_pub_ = n_.advertise<geometry_msgs::Pose2D>("/localization/encoder",1);

    vis_pub_ = n_.advertise<visualization_msgs::Marker>( "/visualization/dr", 10 ); //topic name to be changed

    dr_pub_ = n_.advertise<geometry_msgs::PoseArray>("/particles",100);

    mean_pub_ = n_.advertise<visualization_msgs::Marker>( "/mean", 100 );

    laser_pub_ = n_.advertise<visualization_msgs::Marker>( "/lasers", 100 );


    //Subscribers

    encoder_sub_ = n_.subscribe("/arduino/encoders",1,&localization_node::encoderCallback, this);

    dist_sub_ = n_.subscribe("/ir/distance",1,&localization_node::distCallback, this);

    //Constants and initializations

    std::random_device rd;

    std::mt19937 gen(rd());

    std::normal_distribution<> gauss(INIT_X,0.3);

    std::normal_distribution<> mini_gauss(0.25);

    variance_ = 0;

    epsilon_= 0.001;

    pose_.x = INIT_X;
    pose_.y = INIT_Y;
    pose_.theta = INIT_THETA;

    mean_angle_ = 0;

    b_ = 0.21; //Wheel base
    d_ = 0.1; //Wheel diameter

    sigma_ = 0.5;

    callback_counter = 0;

    mega_delta_x_= 0;

    mega_delta_y_= 0;

    mega_delta_theta_ = 0;

    x_offset_ = 0.10; //Sensor offset from center of the robot
    y_offset_ = 0.13;

    x_fw_offset_ = 0.12;
    y_fw_offset_ = 0.13;

    median_fl = median_fr = median_lf = median_lb = median_rf = median_rb = std::vector<double>(4,0);

    //Dead Reckoning Section

    //Walls//
    map_segments[0].x1 = 0;map_segments[0].y1=0; map_segments[0].x2=3.72;map_segments[0].y2=0; //Correct
    map_segments[1].x1 = 0;map_segments[1].y1=0.66; map_segments[1].x2=0;map_segments[1].y2=3.66; //Correct
    map_segments[2].x1 = 3.72;map_segments[2].y1=0; map_segments[2].x2=3.72;map_segments[2].y2=3.66; //Correct
    map_segments[3].x1 = 0;map_segments[3].y1=3.66; map_segments[3].x2=3.72;map_segments[3].y2=3.66; //Correct

    //Interior free walls//
    map_segments[4].x1 = 0.54;map_segments[4].y1=2.49; map_segments[4].x2=1.185;map_segments[4].y2=2.49; //Correct
    map_segments[5].x1 = 0.54;map_segments[5].y1=2.49; map_segments[5].x2=0.54;map_segments[5].y2=1.905; //Correct
    map_segments[6].x1 = 1.185;map_segments[6].y1=2.49; map_segments[6].x2=1.185;map_segments[6].y2=1.905; //Correct


    //Interior Walls//


    map_segments[7].x1 = 0;map_segments[7].y1=0.66; map_segments[7].x2=3;map_segments[7].y2=0.66; //Correct
    map_segments[8].x1 = 1.20;map_segments[8].y1=0.66; map_segments[8].x2=1.20;map_segments[8].y2=1.32; //Correct
    map_segments[9].x1 = 2.40;map_segments[9].y1=0.44; map_segments[9].x2=2.40;map_segments[9].y2=3.15; //Correct
    map_segments[10].x1 = 3.72;map_segments[10].y1=1.335; map_segments[10].x2=3.12;map_segments[10].y2=1.335; //Correct
    map_segments[11].x1 = 3.72;map_segments[11].y1=2.655; map_segments[11].x2=3.12;map_segments[11].y2=2.655; //Correct
    map_segments[12].x1 = 3.00;map_segments[12].y1=1.995; map_segments[12].x2=2.40;map_segments[12].y2=1.995; //Correct
    map_segments[13].x1 = 1.80;map_segments[13].y1=1.95; map_segments[13].x2=2.40;map_segments[13].y2=1.95; //Correct
    map_segments[14].x1 = 1.80;map_segments[14].y1=2.76; map_segments[14].x2=2.40;map_segments[14].y2=2.76; //Correct


    for(int i=0;i<=14;i++) //For running on the real robot;
    {
        map_segments[i].x1 = map_segments[i].x1/1.50;
        map_segments[i].y1 = map_segments[i].y1/1.50;
        map_segments[i].x2 = map_segments[i].x2/1.50;
        map_segments[i].y2 = map_segments[i].y2/1.50;
    }


    //Test maze walls




    //Particle filter section


    //Marker section//

    dr_marker.header.frame_id = "/map";
    dr_marker.header.stamp = ros::Time();
    dr_marker.ns = "my_namespace";
    dr_marker.id = 0;
    dr_marker.type = visualization_msgs::Marker::SPHERE;

    dr_marker.action = visualization_msgs::Marker::ADD;
    dr_marker.pose.position.x = INIT_X;
    dr_marker.pose.position.y = INIT_Y;
    dr_marker.pose.position.z = 0;
    dr_marker.pose.orientation.x = 0.0;
    dr_marker.pose.orientation.y = 0.0;
    dr_marker.pose.orientation.z = 0.0;
    dr_marker.pose.orientation.w = 1.0;
    dr_marker.scale.x = 0.1;
    dr_marker.scale.y = 0.1;
    dr_marker.scale.z = 0.1;
    dr_marker.color.a = 1.0; // Don't forget to set the alpha!
    dr_marker.color.r = 1.0;
    dr_marker.color.g = 0.0;
    dr_marker.color.b = 0.0;



    mean_marker.header.frame_id = "/map";
    mean_marker.header.stamp = ros::Time();
    mean_marker.ns = "my_namespace";
    mean_marker.id = 0;
    mean_marker.type = visualization_msgs::Marker::CYLINDER;

    mean_marker.action = visualization_msgs::Marker::ADD;
    mean_marker.pose.position.x = INIT_X;
    mean_marker.pose.position.y = INIT_Y;
    mean_marker.pose.position.z = 0;
    mean_marker.pose.orientation.x = 0.0;
    mean_marker.pose.orientation.y = 0.0;
    mean_marker.pose.orientation.z = 0.0;
    mean_marker.pose.orientation.w = 1.0;
    mean_marker.scale.x = b_;
    mean_marker.scale.y = b_;
    mean_marker.scale.z = 0.1;
    mean_marker.color.a = 1.0; // Don't forget to set the alpha!
    mean_marker.color.r = 1.0;
    mean_marker.color.g = 1.0;
    mean_marker.color.b = 1.0;



    all_poses.poses.clear();

    double yaw;


    all_poses.header.stamp = ros::Time::now();
    all_poses.header.frame_id = "/map";

    for(int i=0;i<=N_PARTICLES-1;i++)
    {
        mypoint m;

        geometry_msgs::Pose pose;

        //all_particles_[i].x = gauss(gen);
        //all_particles_[i].y = gauss(gen);

         //all_particles_[i].x = MAX_X*((double)std::rand() / (double)RAND_MAX);
         //all_particles_[i].y = MAX_Y*((double)std::rand() / (double)RAND_MAX);
         //all_particles_[i].theta = 2*M_PI*((double)std::rand() / (double)RAND_MAX);
        all_particles_[i].x = INIT_X;
        all_particles_[i].y = INIT_Y;
         yaw = INIT_THETA;

        pose.position.x = all_particles_[i].x;
        pose.position.y = all_particles_[i].y;

        if(all_particles_[i].x==0&&all_particles_[i].y==0)
        {
            std::cout<<"Pose initialized to 0 in constructor"<<std::endl;
        }

        //

        

        pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        weights_[i] = 1;

        all_poses.poses.push_back(pose);

    }

}

double localization_node::moving_average(std::vector<double> &vector)
{
    double retval;

    retval = (vector[0] + vector[1] + vector[2] + vector[3])/5.0;

    return retval;

}

void localization_node::distCallback(const distance_publisher::IrDistance &dist)
{

    median_lf[0] = median_lf[1]; median_lf[1] = median_lf[2]; median_lf[2] = median_lf[3]; //Shift

    median_lf[3] = dist.left_front_distance;

    median_lb[0] = median_lb[1]; median_lb[1] = median_lb[2]; median_lb[2] = median_lb[3]; //Shift

    median_lb[3] = dist.left_back_distance;

    median_rf[0] = median_rf[1]; median_rf[1] = median_rf[2]; median_rf[2] = median_rf[3]; //Shift

    median_rf[3] = dist.right_front_distance;

    median_rb[0] = median_rb[1]; median_rb[1] = median_rb[2]; median_rb[2] = median_rb[3]; //Shift

    median_rb[3] = dist.right_back_distance;

    median_fr[0] = median_fr[1]; median_fr[1]=median_fr[2]; median_fr[2] = median_fr[3];

    median_fr[3] = dist.front_right_distance;

    median_fl[0] = median_fl[1]; median_fl[1]=median_fl[2]; median_fl[2] = median_fl[3];

    median_fl[3] = dist.front_left_distance;


    calculated_distance_fl_=moving_average(median_fl);
    calculated_distance_fr_=moving_average(median_fr);
    calculated_distance_lf_ = moving_average(median_lf);
    calculated_distance_lb_ = moving_average(median_lb);
    calculated_distance_rf_ = moving_average(median_rf);
    calculated_distance_rb_ = moving_average(median_rb);







    /*
    calculated_distance_fl_= dist.front_left_distance;

    calculated_distance_fr_ = dist.front_right_distance;

    calculated_distance_lf_= dist.left_front_distance;

    calculated_distance_lb_ = dist.left_back_distance;

    calculated_distance_rf_= dist.right_front_distance;

    calculated_distance_rb_ = dist.right_back_distance;

*/

}

double localization_node::measurement_probability(const double measured_distance,const double predicted_distance,const double sensor_sigma)
{
    double a = (1.0/(sensor_sigma*sqrt(2.0*M_PI)));
    if(isnan(a)) std::cout << "a is nan\n";

    double c = -((measured_distance-predicted_distance)*(measured_distance-predicted_distance))/(2.0*sensor_sigma*sensor_sigma);
    if(isnan(c)) std::cout << "c is nan\n";

    double pr =  a*exp(c);

    if(isnan(pr)) std::cout << "pr is nan\n";

    return pr;
}


double localization_node::normalizer_p_hit(double &val, bool long_active)
{
    /*
    double normalizer = 0.0;
    double sum=0.0;

    double max_range;


    if(long_active)
    {
        max_range = RANGE_FOR_LONG;
    }

    else
    {
        max_range = RANGE_FOR_SHORT;
    }


    double a = 0;
    double b = max_range;

    double n = 1e3; // Tune for speed/accuracy
    double c = (b-a)/n;

    for (double k = 1.; k < n-1; k+=1.)
        sum += measurement_probability(a + k*c,val,SENSOR_SIGMA);

    normalizer = c * ((measurement_probability(a,val,sensor_sigma_) + measurement_probability(b,val,sensor_sigma_)) / 2 + sum);

    normalizer = 1/normalizer;

    return normalizer;


*/
    return 1;
}

void localization_node::encoderCallback(const ras_arduino_msgs::Encoders encodermsg)
{


    std::random_device rd;
    std::mt19937 gen(rd());

    std::normal_distribution<> mini_gauss2(0.005,0.01);

    //Delta Calculations
    delta_sl_ = -((M_PI*d_)*encodermsg.delta_encoder1)/360.0; //Meters
    delta_sr_ = -((M_PI*d_)*encodermsg.delta_encoder2)/360.0; //Meters

    delta_s_ = (delta_sl_+ delta_sr_)/2.0; //Meters

    delta_theta_ = ((delta_sr_ - delta_sl_)/b_); //Degrees?

    delta_x_ = delta_s_*cos(pose_.theta + (delta_theta_/2.0));

    delta_y_ = delta_s_*sin(pose_.theta + (delta_theta_/2.0));

    //Pose Calculations




    //Update

    pose_.x = pose_.x + delta_x_;

    pose_.y = pose_.y + delta_y_; //Changing to minus because our encoders are fucked up

    pose_.theta = (pose_.theta + delta_theta_); //Radians

    pose_.theta = fmod(pose_.theta,2*3.14159); //Modulus to get values in 0 - 2 pi range





    //Marker update

    dr_marker.pose.position.x = pose_.x;
    dr_marker.pose.position.y = pose_.y;

    mega_delta_x_ = mega_delta_x_+ delta_x_;
    mega_delta_y_ = mega_delta_y_+ delta_y_;
    mega_delta_theta_ = fmod(mega_delta_theta_+delta_theta_,2*M_PI);

    double velocity = sqrt(mega_delta_x_*mega_delta_x_ + mega_delta_y_*mega_delta_y_);

    //Particle update

    if(callback_counter>5&&callback_counter%10==0)
    {

        //std::cout<<"Resampling"<<std::endl;
        for(int i=0;i<=N_PARTICLES-1;i++)
        {
            all_particles_[i].x = all_particles_[i].x + velocity*cos(all_particles_[i].theta) + mini_gauss2(gen);// K_RAND_X*((double)std::rand() / (double)RAND_MAX); //- ((double)std::rand() / (double)RAND_MAX)/K_RAND_X;
            all_particles_[i].y = all_particles_[i].y + velocity*sin(all_particles_[i].theta) + mini_gauss2(gen);//K_RAND_Y*((double)std::rand() / (double)RAND_MAX);// - ((double)std::rand() / (double)RAND_MAX)/K_RAND_Y;
            all_particles_[i].theta = fmod(all_particles_[i].theta + mega_delta_theta_,2*M_PI) + ((double)std::rand() / (double)RAND_MAX)/K_RAND_THETA; //- ((double)std::rand() / (double)RAND_MAX)/K_RAND_THETA;

            //resample();


        }



        updateMean();


        mega_delta_x_ = 0;
        mega_delta_y_ = 0;
        mega_delta_theta_ = 0;



    }
    callback_counter++;

}

void localization_node::normalize_weights()
{


    double sum_weights = 0;
    double tempsum = 0;



    for(int i = 0; i<=N_PARTICLES-1 ; i++)
    {
        sum_weights = sum_weights + weights_[i];
        //std::cout<<sum_weights<<"\r";
    }


    for(int i = 0; i<=N_PARTICLES-1 ; i++)
    {
        weights_[i] = weights_[i]/sum_weights;

        if(isnan(weights_[i]))
        {
            weights_[i] = 0;
        }
    }





}


double localization_node::rand_probability_long(double measurement)
{
    double z_max = 0.8;

    if(measurement<z_max&&measurement>0)
    {
        return 1/z_max;
    }
    else
    {
        return 0.001;
    }
}

double localization_node::rand_probability_short(double measurement)
{
    double z_max = 0.4;

    if(measurement<z_max&&measurement>0)
    {
        return 1/z_max;
    }
    else
    {
        return 0.001;
    }
}

double localization_node::calculate_weights()
{

    bool long_active = true;

    for(int i=0;i<=N_PARTICLES-1;i++)
    {
        if(all_particles_[i].x>MAX_X||all_particles_[i].x<MIN_X)
        {
            all_particles_[i].x = pose_.x;
            weights_[i]=0;
            all_particles_[i].y = pose_.y;
            // std::cout<<"Deleting particle"<<std::endl;
        }


        if(all_particles_[i].y>MAX_Y||all_particles_[i].y<MIN_Y)
        {
            all_particles_[i].x = pose_.x;
            weights_[i]=0;
            all_particles_[i].y = pose_.y;

        }

        else
        {



            double pred_distance_lf = getClosestWallDistanceLF(all_particles_[i].x,all_particles_[i].y,all_particles_[i].theta);

            double prob_lf = Z_HIT * normalizer_p_hit(calculated_distance_lf_)*measurement_probability(pred_distance_lf, calculated_distance_lf_, SENSOR_SIGMA) + Z_RAND*rand_probability_long(calculated_distance_lf_);

            double pred_distance_lb = getClosestWallDistanceLB(all_particles_[i].x,all_particles_[i].y,all_particles_[i].theta);

            double prob_lb = Z_HIT * normalizer_p_hit(calculated_distance_lb_)*measurement_probability(pred_distance_lb, calculated_distance_lb_, SENSOR_SIGMA) + Z_RAND * rand_probability_short(calculated_distance_lb_);


            double pred_distance_rf = getClosestWallDistanceRF(all_particles_[i].x,all_particles_[i].y,all_particles_[i].theta);

            double prob_rf = Z_HIT*normalizer_p_hit(calculated_distance_rf_)*measurement_probability(pred_distance_rf, calculated_distance_rf_, SENSOR_SIGMA) + Z_RAND * rand_probability_short(calculated_distance_rf_);


            double pred_distance_rb = getClosestWallDistanceRB(all_particles_[i].x,all_particles_[i].y,all_particles_[i].theta);

            double prob_rb = Z_HIT*normalizer_p_hit(calculated_distance_rb_)* measurement_probability(pred_distance_rb, calculated_distance_rb_, SENSOR_SIGMA) + Z_RAND *  rand_probability_short(calculated_distance_rb_);


            double pred_distance_fl = getClosestWallDistanceFL(all_particles_[i].x,all_particles_[i].y,all_particles_[i].theta);

            double prob_fl = Z_HIT*normalizer_p_hit(calculated_distance_fl_,long_active)*measurement_probability(pred_distance_fl, calculated_distance_fl_, SENSOR_SIGMA)+Z_RAND*rand_probability_short(calculated_distance_fl_);


            double pred_distance_fr = getClosestWallDistanceFR(all_particles_[i].x,all_particles_[i].y,all_particles_[i].theta);

            double prob_fr = Z_HIT*normalizer_p_hit(calculated_distance_fr_,long_active)*measurement_probability(pred_distance_fr,calculated_distance_fr_,SENSOR_SIGMA) + Z_RAND* rand_probability_long(calculated_distance_fr_);




            weights_[i] = prob_lf*prob_lb*prob_rf*prob_rb*prob_fr;//*prob_fl;

            //weights_[i] = prob_rf*prob_rb;

        }
        if(isnan(weights_[i]))
        {
            weights_[i] = 0;
            std::cout<<"Weight"<<i<<" is Nan"<<std::endl;
        }

        // std::cout<<"w " << i <<"\t"<< weights_[i]<<std::endl;

    }

}


void localization_node::update_markers()
{
    all_poses.poses.clear();

    //Adding to get rid of some stupid scope errors

    std::random_device rd;
    std::mt19937 gen(rd());

    std::normal_distribution<> gauss(0.005);


    for(int i=0;i<=N_PARTICLES-1;i++)
    {
        geometry_msgs::Pose pose;

        pose.position.x = all_particles_[i].x;
        pose.position.y = all_particles_[i].y;

        pose.orientation = tf::createQuaternionMsgFromYaw(all_particles_[i].theta);

        all_poses.poses.push_back(pose);

    }


}

void localization_node::updateMean()
{
    double meanx = 0;
    double meany = 0;
    double weight_sum = 0;

    double x = 0;
    double y = 0;

    variance_ = 0;

    for (int i=0;i<=N_PARTICLES-1;i++)
    {
        meanx = meanx + weights_[i]*all_particles_[i].x;
        meany = meany + weights_[i]*all_particles_[i].y;
        weight_sum = weight_sum + weights_[i];

        x = x + cos(all_particles_[i].theta);
        y = y + sin(all_particles_[i].theta);

    }

    mean_angle_ = atan2(y,x);

    //std::cout<<mean_angle_*180/M_PI<<std::endl;

    //meanx = meanx/N_PARTICLES;
    //meany = meany/N_PARTICLES;

    meanx = meanx/weight_sum;
    meany = meany/weight_sum;


    /*
    for (int i=0;i<=N_PARTICLES-1;i++)
    {
        variance_ += pow(all_particles_[i].x - meanx,2);
    }
*/

    mean_marker.pose.position.x = meanx;
    mean_marker.pose.position.y = meany;


}

void localization_node::resample()
{
    normalize_weights();

    mypoint new_all_particles_[N_PARTICLES];

    int index = rand()%N_PARTICLES; //Random number from one to 1000

    double beta = 0;

    double max = 0;

    for (int i=0;i<=N_PARTICLES-1;i++)
    {
        if(weights_[i]>max)
        {
            max=weights_[i];
        }

    }


    for (int i=0;i<=N_PARTICLES-1;i++)
    {

        double rand_val = ((double)std::rand() / (double)RAND_MAX);

        beta = beta + rand_val*2.0*max ;

        while (beta > weights_[index])
        {
            beta = beta - weights_[index];
            index = (index + 1)%N_PARTICLES ;
        }

        new_all_particles_[i] = all_particles_[index];
    }

    for(int i=0;i<=N_PARTICLES-1;i++)
    {
        all_particles_[i] = new_all_particles_[i];
    }
}

/*
    //Working resampling
    if(0)
    {
        mypoint new_all_particles_[N_PARTICLES];

        double sum = 0.0;
        double probabilities[N_PARTICLES];
        //Calculate normalizer

        for (int i=0;i<N_PARTICLES-1;i++)
        {
            sum+=weights_[i];
        }

        double cumsum = 0.0;

        for (int i=0;i<N_PARTICLES-1;i++)
        {
            cumsum += weights_[i]/sum;
            probabilities[i] = cumsum;
        }




        for (int i=0;i<=N_PARTICLES-1;i++)
        {
            double temp = (double)std::rand() / (double)RAND_MAX;

            for (int pos=0;pos<N_PARTICLES-1;pos++)
            {

                if(temp<probabilities[pos])

                {
                    new_all_particles_[i] = all_particles_[pos];

                    //new_all_particles_[i].x = new_all_particles_[i].x + mini_gauss(gen);
                    //new_all_particles_[i].y = new_all_particles_[i].y + 0.1*(double)std::rand() / (double)RAND_MAX;
                    //new_all_particles_[i].theta = fmod(new_all_particles_[i].theta + mini_gauss(gen)*(double)std::rand() / (double)RAND_MAX, 2*M_PI);


                    break;
                }
            }
        }


        for(int i=0;i<=N_PARTICLES-1;i++)
        {
            all_particles_[i] = new_all_particles_[i];
        }

    }
*/
/*
    if(0)
    {
        //Systematic resampling

        int N  = N_PARTICLES;

        std::array<double,N_PARTICLES> positions;

        std::array<double,N_PARTICLES> indexes;

        std::array<double, N_PARTICLES> cumsum;

        mypoint new_all_particles_[N_PARTICLES];

        double sum = 0;

        for(int i=0; i<=N_PARTICLES-1;i++)
        {
            positions[i] = (i + (double)std::rand() / (double)RAND_MAX)/N;
            indexes[i] = 0;
        }


        for (int i=0;i<=N_PARTICLES-1;i++)
        {
            sum+=weights_[i];
            cumsum[i] = sum;

        }

        int i=0; int j=0;

        while(i<=N_PARTICLES-1)
        {
            if(positions[i]<cumsum[j])
            {
                indexes[i] = j;
                i+= 1;
            }
            else
            {
                j+=1;
            }

        }


        for(int i=0;i<=N_PARTICLES-1;i++)
        {
            int ind = indexes[i];
            new_all_particles_[i] = all_particles_[ind];
            all_particles_[i] = new_all_particles_[i]; //Replace ith particle

            weights_[i] = weights_[ind];


        }

        //NOTE -to write normalizer

        normalize_weights();
    }





}
*/
int localization_node::get_line_intersection(double p0_x, double p0_y, double p1_x, double p1_y,
                                             double p2_x, double p2_y, double p3_x, double p3_y, double *distance,double &intx, double &inty) //Taken from github
{
    double i_x; double i_y;
    float s02_x, s02_y, s10_x, s10_y, s32_x, s32_y, s_numer, t_numer, denom, t;

    s10_x = p1_x - p0_x;
    s10_y = p1_y - p0_y;
    s32_x = p3_x - p2_x;
    s32_y = p3_y - p2_y;

    denom = s10_x * s32_y - s32_x * s10_y;
    if (denom == 0)
        return 0; // Collinear
    bool denomPositive = denom > 0;

    s02_x = p0_x - p2_x;
    s02_y = p0_y - p2_y;
    s_numer = s10_x * s02_y - s10_y * s02_x;
    if ((s_numer < 0) == denomPositive)
        return 0; // No collision

    t_numer = s32_x * s02_y - s32_y * s02_x;
    if ((t_numer < 0) == denomPositive)
        return 0; // No collision

    if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive))
        return 0; // No collision
    // Collision detected
    t = t_numer / denom;
    //if (i_x != 0)
    intx = p0_x + (t * s10_x);
    //if (i_y != 0)
    inty = p0_y + (t * s10_y);

    //

    i_x = intx;
    i_y = inty;

    *distance = sqrt((i_x-p0_x)*(i_x-p0_x)+(i_y-p0_y)*(i_y-p0_y));



    return 1;
}

void localization_node::draw_cool_laser(double x1, double y1, double x2, double y2,double color=1)
{

    visualization_msgs::Marker laser;
    geometry_msgs::Point pt1;
    geometry_msgs::Point pt2;

    laser.type = visualization_msgs::Marker::LINE_STRIP;
    laser.action = visualization_msgs::Marker::ADD;
    laser.header.frame_id = "/map";
    laser.header.stamp = ros::Time::now();
    laser.id = 1;
    laser.scale.x = 0.01;
    laser.scale.y = 0.1;
    laser.scale.z = 0.1;
    laser.color.a = 1.0;
    laser.color.r = color;//((double)std::rand() / (double)RAND_MAX);
    laser.color.g = 1.0;//((double)std::rand() / (double)RAND_MAX);
    laser.color.b = 0.0;//((double)std::rand() / (double)RAND_MAX);

    pt1.x = x1;
    pt1.y = y1;
    pt1.z = 0.1;

    pt2.x = x2;
    pt2.y = y2;
    pt2.z = 0.1;

    laser.points.push_back(pt1);
    laser.points.push_back(pt2);



    laser_pub_.publish(laser);

}

double localization_node::getClosestWallDistanceFR(double x, double y, double theta)
{

    double min = 999;
    double new_min = 0;

    double contactx,dx;
    double contacty,dy;

    double theta1 = theta;



    double x1 = x + (x_fw_offset_*cos(-theta) - y_fw_offset_*sin(-theta));
    double y1 = y + (-x_fw_offset_*sin(-theta)- y_fw_offset_*cos(-theta));

    double x2 = x + ((x_fw_offset_+0.8)*cos(-theta)-(y_fw_offset_)*sin(-theta));
    double y2 = y + (-(x_fw_offset_+0.8)*sin(-theta)-(y_fw_offset_)*cos(-theta));



    double front_dist;

    int collision;

    for(int i=0;i<=14;i++)
    {
        //collision = get_line_intersection(x1,y1,x2,y2,map_segments[i].x1, map_segments[i].y1, map_segments[i].x2, map_segments[i].y2, &front_dist, contactx, contacty);
        collision = get_line_intersection(map_segments[i].x1, map_segments[i].y1, map_segments[i].x2, map_segments[i].y2,x1,y1,x2,y2, &front_dist, contactx, contacty);

        if(front_dist>0.8)
        {
            front_dist = 0.8;
        }

        if(collision)
        {
            
            if(front_dist>0)
            {
                if(front_dist<min)
                {
                    min=front_dist;
                    dx=contactx;
                    dy=contacty;
                    

                }

            }
        }

        if(min>RANGE_FOR_LONG)
        {
            min = RANGE_FOR_LONG;
        }

    }

    //draw_cool_laser(x1,y1,dx,dy);
    return min + WALL_THICKNESS;

}


double localization_node::getClosestWallDistanceFL(double x, double y, double theta)
{

    double min = 999;
    double new_min = 0;

    double contactx,dx;
    double contacty,dy;


    double theta1 = theta;



    double x1 = x + (x_fw_offset_*cos(-theta) + y_fw_offset_*sin(-theta));
    double y1 = y + (-x_fw_offset_*sin(-theta)+ y_fw_offset_*cos(-theta));

    double x2 = x + ((x_fw_offset_+2.8)*cos(-theta)+(y_fw_offset_)*sin(-theta));
    double y2 = y + (-(x_fw_offset_+2.8)*sin(-theta)+(y_fw_offset_)*cos(-theta));



    double front_dist;

    int collision;

    for(int i=0;i<=14;i++)
    {
        collision = get_line_intersection(x1,y1,x2,y2,map_segments[i].x1, map_segments[i].y1, map_segments[i].x2, map_segments[i].y2, &front_dist, contactx, contacty);


        if(front_dist>0.8)
        {
            front_dist = 0.8;
        }

        if(collision)
        {
            if(front_dist>0)
            {
                if(front_dist<min)
                {
                    min=front_dist;
                    dx=contactx;
                    dy=contacty;
                  
                }
            }
        }
        if(min>RANGE_FOR_LONG)
        {
            min = RANGE_FOR_LONG;
        }
    }
    //draw_cool_laser(x1,y1,x2,y2);
    return min + WALL_THICKNESS;

}

double localization_node::getClosestWallDistanceLF(const double x,const double y,const double theta)
{

    double min = 999;
    double new_min = 0;

    double contactx,dx;
    double contacty,dy;

    double theta1 = theta + M_PI/2;

    double x1 = x + (x_offset_*cos(-theta) + y_offset_*sin(-theta));
    double y1 = y + (-x_offset_*sin(-theta)+ y_offset_*cos(-theta));

    double x2 = x + (x_offset_*cos(-theta)+(y_offset_+0.8)*sin(-theta));
    double y2 = y + (-x_offset_*sin(-theta)+(y_offset_+0.8)*cos(-theta));

    

    double left_dist;

    int collision;

    for(int i=0;i<=14;i++)
    {
        collision = get_line_intersection(x1,y1,x2,y2,map_segments[i].x1, map_segments[i].y1, map_segments[i].x2, map_segments[i].y2, &left_dist, contactx, contacty);

        if(left_dist>0.8)
        {
            left_dist = 0.8;
        }

        if(collision)
        {
            if(left_dist>0)
            {
                if(left_dist<min)
                {
                    min=left_dist;
                    dx=contactx;
                    dy=contacty;

                }

            }
        }



    }


    if(min>RANGE_FOR_SHORT)
    {
        min = RANGE_FOR_SHORT;
    }

    //draw_cool_laser(x1,y1,dx,dy);

    return min+ WALL_THICKNESS;

}

double localization_node::getClosestWallDistanceLB(const double x,const double y,const double theta)
{

    double min = 999;
    double new_min = 0;

    double contactx,dx;
    double contacty,dy;

    double theta1 = theta + M_PI/2; //TODO

    double x1 = x + (-x_offset_*cos(-theta) + y_offset_*sin(-theta));
    double y1 = y + (x_offset_*sin(-theta)+ y_offset_*cos(-theta));

    double x2 = x + (-x_offset_*cos(-theta)+(y_offset_+0.8)*sin(-theta));
    double y2 = y + (x_offset_*sin(-theta)+(y_offset_+0.8)*cos(-theta));

    double left_dist;

    int collision;

    for(int i=0;i<=14;i++)
    {
        collision = get_line_intersection(x1,y1,x2,y2,map_segments[i].x1, map_segments[i].y1, map_segments[i].x2, map_segments[i].y2, &left_dist, contactx, contacty);


        if(left_dist>0.8)
        {
            left_dist = 0.8;
        }

        if(collision)
        {
            if(left_dist>0)
            {
                if(left_dist<min)
                {
                    min=left_dist;
                    dx=contactx;
                    dy=contacty;

                }

            }
        }

    }
    if(min>RANGE_FOR_SHORT)
    {
        min = RANGE_FOR_SHORT;
    }
    //draw_cool_laser(x1,y1,dx,dy);

    return min+ WALL_THICKNESS;


}


double localization_node::getClosestWallDistanceRF(double x, double y, double theta)
{
    double min = 999;
    double new_min = 0;

    double contactx,dx;
    double contacty,dy;

    double theta1 = theta - M_PI/2;

    double x1 = x + (x_offset_*cos(-theta) + -y_offset_*sin(-theta));
    double y1 = y + (-x_offset_*sin(-theta)+ -y_offset_*cos(-theta));

    double x2 = x + (x_offset_*cos(-theta)+(-y_offset_-0.8)*sin(-theta));
    double y2 = y + (-x_offset_*sin(-theta)+(-y_offset_-0.8)*cos(-theta));

    double right_dist;

    int collision;

    for(int i=0;i<=14;i++) //Iterate over the walls
    {
        collision = get_line_intersection(x1,y1,x2,y2,map_segments[i].x1, map_segments[i].y1, map_segments[i].x2, map_segments[i].y2, &right_dist, contactx, contacty);

        if(right_dist>0.8)
        {
            right_dist = 0.8;
        }


        if(collision)
        {
            if(right_dist>0)
            {
                if(right_dist<min)
                {
                    min=right_dist;
                    dx=contactx;
                    dy=contacty;
                }
            }
        }
    }

    if(min>RANGE_FOR_SHORT)
    {
        min = RANGE_FOR_SHORT;
    }
    //draw_cool_laser(x1,y1,dx,dy);
    return min+ WALL_THICKNESS;
}

double localization_node::getClosestWallDistanceRB(double x, double y, double theta)
{
    double min = 999;
    double new_min = 0;

    double contactx;
    double contacty;

    double dx;
    double dy;

    double theta1 = theta - M_PI/2;

    double x1 = x + (-x_offset_*cos(-theta) + -y_offset_*sin(-theta));
    double y1 = y + (x_offset_*sin(-theta)+ -y_offset_*cos(-theta));
    double x2 = x + (-x_offset_*cos(-theta)+(-y_offset_-0.8)*sin(-theta));
    double y2 = y + (x_offset_*sin(-theta)+(-y_offset_-0.8)*cos(-theta));
    double right_dist;
    int collision;

    for(int i=0;i<=14;i++) //Iterate over the walls
    {
        collision = get_line_intersection(x1,y1,x2,y2,map_segments[i].x1, map_segments[i].y1, map_segments[i].x2, map_segments[i].y2, &right_dist, contactx, contacty);


        if(right_dist>0.8)
        {
            right_dist = 0.8;
        }


        if(collision)
        {
            if(right_dist>0)
            {

                if(right_dist<min)
                {
                    min=right_dist;
                    dx=contactx;
                    dy=contacty;


                }
            }
        }



    }


    if(min>RANGE_FOR_SHORT)
    {
        min = RANGE_FOR_SHORT;
    }


    //draw_cool_laser(x1,y1,dx,dy);
    return min+ WALL_THICKNESS;

}

void localization_node::print_five_particles()
{
    for(int i=0;i<=5;i++)
    {
        std::cout<<weights_[i]<<std::endl;
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "localization_node");

    localization_node l_node;

    ros::Rate loop_rate(10);

    while(l_node.n_.ok())
    {
        int counter=0;

        l_node.dr_pub_.publish(l_node.all_poses);


        double testx = l_node.mean_marker.pose.position.x;
        double testy = l_node.mean_marker.pose.position.y;
        double testtheta = l_node.mean_angle_;

        double testx2 = l_node.pose_.x;
        double testy2 = l_node.pose_.y;
        double testtheta2 = l_node.pose_.theta;


       // std::cout<<"Pose  "<<testx<<"\t"<<testy<<"\t"<<testtheta<<"\t"<<std::endl;
       // std::cout<<"Predicted(mean): \t"<<"Predicted(DR): \t"<<"Calculated: \t \n";
       // std::cout<<"Front Left: "<<l_node.getClosestWallDistanceFL(testx, testy, testtheta)<<"\t\t"<<l_node.getClosestWallDistanceFL(testx2, testy2, testtheta2)<<"\t\t"<<l_node.calculated_distance_fl_<<"\n";
//        std::cout<<"Front Right: "<<l_node.getClosestWallDistanceFR(testx, testy, testtheta)<<"\t\t"<<l_node.getClosestWallDistanceFR(testx2, testy2, testtheta2)<<"\t\t"<<l_node.calculated_distance_fr_<<"\n";

       // std::cout<<"Left Front: "<<l_node.getClosestWallDistanceLF(testx, testy, testtheta)<<"\t"<<l_node.getClosestWallDistanceLF(testx2, testy2, testtheta2)<<"\t\t"<<l_node.calculated_distance_lf_<<"\n";
       // std::cout<<"Left Back: "<<l_node.getClosestWallDistanceLB(testx, testy, testtheta)<<"\t\t"<<l_node.getClosestWallDistanceLB(testx2, testy2, testtheta2)<<"\t\t"<<l_node.calculated_distance_lb_<<"\n";
       // std::cout<<"Right Front: "<<l_node.getClosestWallDistanceRF(testx, testy, testtheta)<<"\t"<<l_node.getClosestWallDistanceRF(testx2, testy2, testtheta2)<<"\t\t"<<l_node.calculated_distance_rf_<<"\n";
       // std::cout<<"Right Back: "<<l_node.getClosestWallDistanceRB(testx, testy, testtheta)<<"\t"<<l_node.getClosestWallDistanceRB(testx2, testy2, testtheta2)<<"\t\t"<<l_node.calculated_distance_rb_<<"\n \n";



        l_node.calculate_weights();
        l_node.resample();
        l_node.update_markers();
        l_node.vis_pub_.publish(l_node.dr_marker);
        l_node.mean_pub_.publish(l_node.mean_marker);


        ros::spinOnce();

        counter++;

        loop_rate.sleep();


    }


    return 0;
}
