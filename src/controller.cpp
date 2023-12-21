/*
    File containing methods used to perform visual
    serfoing control.

    Author: Adalberto Oliveira <adalberto.oliveira@solinftec.com>
    Project: Solix AgBot
    Version: 2.0
    Date: 10-5-2022
*/

#include "../include/controller.h"

/*
    Constructors
*/
Controller::Controller(ros::NodeHandle nh)
{

    this->init(nh); 
}

Controller::~Controller()
{

    // Class Destructor
}

void Controller::init(ros::NodeHandle nh)
{
    // Creating the publishers
    cout << "\nStarting Visual Servoing object...";
    pubCmdVel = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);

    // Starting the image processors
    image_transport::ImageTransport it(nh);
    pubProcessedImg = it.advertise("processed_img", 1);

    // Creating the subscribers
    subImageRaw = nh.subscribe("image_raw",1000,
                            &Controller::callbackImgRaw, this);
    subReset = nh.subscribe("reset",1000,
                             &Controller::callbackReset, this);
    subStart = nh.subscribe("start",1000,
                            &Controller::callbackStart, this);

    // Loading constants from param server
    ros::param::get("visual_servoing/control/lin_vel",LINVEL);
    ros::param::get("visual_servoing/control/gains",GAINS);
    ros::param::get("visual_servoing/control/control_pi",controlPI);
}

/*
    Getters
*/
float Controller::getGain(int gain)
{

    return GAINS[gain];
}

float Controller::getVelocity()
{

    return LINVEL;
}

cv::Mat Controller::getRawImg()
{
    cv::Mat rawImg = cvPtr->image;

    return rawImg;
}

bool Controller::getImgState()
{
    return isImage;
}
/*
    Setters
*/

// Callbacks
void Controller::callbackImgRaw(sensor_msgs::Image msg)
{
    isImage = true;
    cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
}

void Controller::callbackReset(std_msgs::Bool msg)
{
    reset = msg.data;
}

void Controller::callbackStart(std_msgs::Bool msg)
{
    start = msg.data;
    if (start)
    {
	errIntRho = 0;
	errIntTheta = 0;
        reset = true;
    }
}

geometry_msgs::Twist Controller::velocities(float rho,float theta)
{

    // Applying low-pass filter
    prevRho = prevRho + dt*(rho - prevRho);
    prevSlope = prevSlope + dt*(theta - prevSlope);

    // Controller gains
    float kRho = GAINS[0];
    float kTheta = GAINS[1];
    float kV = GAINS[2];

    // Computing the control error
    float errRho = 0 - prevRho;
    float errTheta = 0 - prevSlope;

    // Computing the control signals
    float slowDown = std::abs(errRho)*kV;
    if (slowDown > LINVEL)
    {
        slowDown = LINVEL;
    }
    float v = LINVEL - slowDown;
    float omega = - errRho*kRho - errTheta*kTheta;

    // Filling the pub object
    geometry_msgs::Twist cmdVel;
    if (start)
    {
        cmdVel.linear.x = v;
        cmdVel.angular.z = omega;

    }

    return cmdVel;
}

geometry_msgs::Twist Controller::velocitiesPI(float rho,float theta)
{

    // Applying low-pass filter
    prevRho = prevRho + dt*(rho - prevRho);
    prevSlope = prevSlope + dt*(theta - prevSlope);

    // Controller gains
    float kRho = GAINS[0];
    float kTheta = GAINS[1];
    float kV = GAINS[2];
    float kIRho = GAINS[3];
    float kITheta = GAINS[4];
    float kDRho = GAINS[5];
    float kDTheta = GAINS[6];

    // Computing the control error (for Proportional control)
    float errRho = 0 - prevRho;
    float errTheta = 0 - prevSlope;

    // Computing the integral error (for Integral control)
    float deltaRho = errRho - prevErrRho;
    prevErrRho = errRho;
    float deltaTheta = errTheta - prevErrTheta;
    prevErrTheta = errTheta;
    errIntRho = errIntRho + deltaRho;
    errIntTheta = errIntTheta + deltaTheta;
    cout << "\nIntegral errors:\n  rho: " << errIntRho << "\n  th: " << errIntTheta;
 
    // Computing the control signals
    float slowDown = std::abs(errTheta)*kV;
    if (slowDown > LINVEL)
    {
        slowDown = LINVEL;
    }

    float v = LINVEL - slowDown;
    float omega = - (errRho*kRho + deltaRho*kDRho + errIntRho*kIRho) - (errTheta*kTheta + deltaTheta*kDTheta + errIntTheta*kITheta);

    // Filling the pub object
    geometry_msgs::Twist cmdVel;
    if (start)
    {
        cmdVel.linear.x = v;
        cmdVel.angular.z = omega;

    }

    return cmdVel;
}

void Controller::pubVelocity(geometry_msgs::Twist cmdVel)
{
    pubCmdVel.publish(cmdVel);
}

void Controller::pubProcessedImage(cv::Mat &procImage)
{
    // Converting from cv image to ros image message
    sensor_msgs::ImagePtr msgProcImg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", procImage).toImageMsg();

    // Publishing ros image messages
    pubProcessedImg.publish(msgProcImg);
}
