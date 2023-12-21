/*
    Header file containing classes used to perform
    visual servoing control.

    Author: Adalberto Oliveira <adalberto.oliveira@solinftec.com>
    Project: Solix AgBot
    Version: 1.0.0
    Date: 10-5-2022
*/
#ifndef VISUAL_SERVOING_CONTROLLER_H
#define VISUAL_SERVOING_CONTROLLER_H

#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <thread>

using namespace std;


class Controller
{//Class begin

	/*
		Class to handle visual servoing control.
	*/

private:

	// Creating Publishers and Subscribers
    ros::Publisher pubCmdVel;
    image_transport::Publisher pubProcessedImg;
    ros::Subscriber subImageRaw;
    ros::Subscriber subReset;
    ros::Subscriber subStart;

    // Input/Output variables
    cv::Mat rawImg;
    cv_bridge::CvImagePtr cvPtr;

    // Parameter and Constants
    vector<float> GAINS{0,0,0,0,0,0,0};
    float LINVEL;

    // Control Variables
    bool isImage = false;


    // Images
    cv::Mat procImage;


    // For low-pass filter
    float prevRho = 0;
    float prevSlope = 0;
    float dt = 0.6;

    // For PI controller
    float errIntRho = 0;
    float errIntTheta = 0;
    float prevErrRho = 0;
    float prevErrTheta = 0;



public:
    // Variables
    bool reset = false;
    bool start = false;
    bool controlPI = false;

    // Constructors
    Controller(ros::NodeHandle nh);
    
    ~Controller();

    void init(ros::NodeHandle nh);

    // Getters
    float getGain(int gain);

    float getVelocity();
    
    cv::Mat getRawImg();

    bool getImgState();

    // Workers
    void callbackImgRaw(sensor_msgs::Image msg);

    void callbackReset(std_msgs::Bool msg);

    void callbackStart(std_msgs::Bool msg);

    geometry_msgs::Twist velocities(float rho,float theta);

    geometry_msgs::Twist velocitiesPI(float rho,float theta);

    void pubVelocity(geometry_msgs::Twist cmdVel);

    void pubProcessedImage(cv::Mat &procImage);

};//Class end
#endif //VISUAL_SERVOING_CONTROLLER_H
