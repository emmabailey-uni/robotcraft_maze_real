#include <iostream>
#include <cstdlib>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <rosserial_arduino/Test.h>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884L
#endif na



class ReactiveController
{
private:

    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub;

    ros::Subscriber ir_front_sensor;
    ros::Subscriber ir_left_sensor;
    ros::Subscriber ir_right_sensor;

    double front_obstacle_distance;
    double right_obstacle_distance;
    double left_obstacle_distance;
    bool robot_stopped;

    // Desired distance to the right wall
    float desired_linear_velocity;
    float desired_side_wall_distance;
    float front_obstacle_distance_threshold;
    float desired_wall;

    // Wall distance controller parameters
    int k_p;
    int k_i;

    bool front_ir_flag = false;


    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();

        float dt = 0.1;


        static float error;
        static float integral_error;

        float proportional;
        float integral;
        float angular_input;

        // Switch case variables
        static bool init_flag = false;
        static int swtich_case = 0;
        static int count = 0;


        if(!init_flag){
        	// INIT
        	swtich_case = 0;
        } else if (front_obstacle_distance <= front_obstacle_distance_threshold){
        	// PID
        	swtich_case = 2;
        } else{
        	// MANUEVER
        	swtich_case = 1;
        }

        if(desired_wall == 1.0){
            switch(swtich_case) {

            case 0 :        // Initialisation
                            if(!front_ir_flag){
                                msg.linear.x = 0.0;
                            }else if(front_obstacle_distance > front_obstacle_distance_threshold){
                                msg.linear.x = desired_linear_velocity;
                            } else {
                            	msg.linear.x = 0.0;
                                init_flag = true;
                            }
                            msg.angular.z = 0.0;
                            break;       // and exits the switch

            case 1 :        // PI controller

                            // Calculate proportional part
                            error = desired_side_wall_distance - right_obstacle_distance;
                            if(abs(error) < 0.02){
                                error = 0;
                            }
                            proportional =  error*k_p;

                            // Calculate integral part
                            integral_error += error*dt;
                            integral = integral_error*k_i;

                            // Calculate final angular velocity input
                            angular_input = integral + proportional;

                            // Calculating maximum possibl angular velocity request given desired linear velocity 
                            float angular_max = (2*desired_linear_velocity-2*0.016*0.075)/0.094;

                            if(angular_input > angular_max){
                            	angular_input = angular_max;
                            }
                            if(angular_input < -angular_max){
                            	angular_input = -angular_max;
                            }
                            ROS_INFO("ANG ERROR: %f", error);
                            // Write velocities for the next time step
                            msg.angular.z = angular_input;
                            msg.linear.x = desired_linear_velocity;

                            ROS_INFO("Case PID");
                            break;

            case 2 :        // Avoidance manuever
                            msg.linear.x = 0.001;
                            msg.angular.z = 0.7;

                            error = 0;
                            integral_error = 0;

                            ROS_INFO("Case MANUEVER");
                            break;
            }

        }
        else{
            switch(swtich_case) {

            case 0 :        // Initialisation
                            if(!front_ir_flag){
                                msg.linear.x = 0.0;
                            }else if(front_obstacle_distance > front_obstacle_distance_threshold){
                                msg.linear.x = desired_linear_velocity;
                            } else {
                                init_flag = true;
                                msg.linear.x = 0.0;
                            }
                            msg.angular.z = 0.0;
                            break;       // and exits the switch

            case 1 :        // PI controller

                            // Calculate proportional part
                            error = desired_side_wall_distance - left_obstacle_distance;
                            proportional =  error*k_p;

                            // Calculate integral part
                            integral_error += error*dt;
                            integral = integral_error*k_i;

                            // Calculate final angular velocity input
                            angular_input = integral + proportional;

                            ROS_INFO("ANG ERROR: %f", error);

                            // Calculating maximum possibl angular velocity request given desired linear velocity 
                            float angular_max = (2*desired_linear_velocity-2*0.016*0.075)/0.094;

                            if(angular_input > angular_max){
                            	angular_input = angular_max;
                            }
                            if(angular_input < -angular_max){
                            	angular_input = -angular_max;
                            }

                            msg.angular.z = -angular_input;

                            msg.linear.x = desired_linear_velocity;

                            ROS_INFO("Case PID");

                            break;

            case 2 :        // Avoidance manuever
                            msg.linear.x = 0.001;
                            msg.angular.z = -0.7;

                            error = 0;
                            integral_error = 0;

                            ROS_INFO("Case MANUEVER");
                            break;
            }

        }

        // Wirte message to the robot
        return msg;

    }


    void ir_right_Callback(const sensor_msgs::Range::ConstPtr& ir_msg)
    {

        right_obstacle_distance = ir_msg->range;

    }
    void ir_left_Callback(const sensor_msgs::Range::ConstPtr& ir_msg)
    {

        left_obstacle_distance = ir_msg->range;

    }
    void ir_front_Callback(const sensor_msgs::Range::ConstPtr& ir_msg)
    {
    	float sensor_val_front = static_cast<float>(msg->data);
        if(sensor_val_front < 0.15){
            ROS_WARN("Collision risk! The robot is %f meters of an obsctacle, on the front side", sensor_val_front);
        }
        front_obstacle_distance = ir_msg->range;
        front_ir_flag = true;

    }


public:
    ReactiveController(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Create a publisher object, able to push messages
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("reactive_vel", 10);


        // name_of_the_subscriber = n.subscribe("topic_name")
        this->ir_right_sensor = n.subscribe("ir_right_sensor", 10, &ReactiveController::ir_right_Callback, this);
        this->ir_left_sensor = n.subscribe("ir_left_sensor", 10, &ReactiveController::ir_left_Callback, this);
        this->ir_front_sensor = n.subscribe("ir_front_sensor", 10, &ReactiveController::ir_front_Callback, this);


        this->n.getParam("/desired_side_wall_distance", desired_side_wall_distance);
        this->n.getParam("/front_obstacle_distance_threshold", front_obstacle_distance_threshold);
        this->n.getParam("/desired_linear_velocity", desired_linear_velocity);
        this->n.getParam("/desired_wall", desired_wall);
        this->n.getParam("/k_p", k_p);
        this->n.getParam("/k_i", k_i);
    }

    void run(){

        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            if(front_ir_flag){
              auto msg = calculateCommand();
              // Publish the new command
              this->cmd_vel_pub.publish(msg);

            }
            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "reactive_controller");


    // Create our controller object and run it
    auto controller = ReactiveController();
    controller.run();
}
