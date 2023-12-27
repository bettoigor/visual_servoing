/*
    Main code of visual servoing control node.
    This node receives image_raw messagens, with
    raw images from navigation camera and publishes 
    a cmd_vel topic using visual servoing control 
    techniques.

    Author: Adalberto Oliveira <adalberto.oliveira@solinftec.com>
    Author: Willian Antunes <willian.antunes@solinftec.com>
    Project: Solix AgBot
    Version: 2.0
    Date: 10-5-2022
*/


#include "../include/controller.h"
#include "../include/utils.h"
#include "../include/image.h"

/*************** MAIN **************/
int main(int argc, char** argv)
{// Starting the main function for visual servoing control node

    // List of used variables
    int rateHz, roiRange, roi, ref;
    float plantRadius;

    bool showImg;
    cv::Mat rawImg;

    // Initializing the ROS structure
    ros::init(argc, argv, "visual_servoing_node");
    ros::NodeHandle nh;
    ROS_INFO("*** ROS node successfully started! ***");
    
    // Loading and setting the parameters from ros param server
    cout << "\nGetting control parameters... ";
    ros::param::get("visual_servoing/ros_control/rate",rateHz);
    ros::param::get("visual_servoing/image/show_img",showImg);
    ros::param::get("visual_servoing/image/roi_range",roiRange);
    ros::param::get("visual_servoing/image/roi_initial",roi);
    ros::param::get("visual_servoing/image/plant_radius",plantRadius);
    ref = roi;

    ros::Rate loop_rate(rateHz);

    // Creating control object
    cout << "\nCreating control object... ";
    Controller visualServoing(nh);
    cout <<"Done!";

    // Starting the ROS main loop
    ROS_INFO(" *** Velocity control node successfully activated! ***");

    // Waiting for image to be published
    cout << "\nWaiting image publisher <" << visualServoing.getImgState() << "> ...";
    while (!visualServoing.getImgState())
        ros::spinOnce();

    cout << " Received! <" << visualServoing.getImgState() << "> \n";

    std::pair<cv::Point, cv::Point> guideLine;

    while (nh.ok())
    {
        auto start = std::chrono::system_clock::now();

        // Getting the raw image from image topic
        rawImg = visualServoing.getRawImg();

        std::vector<std::vector<cv::Point>> plantCentroids = Image::getMaskFeature(rawImg);

        if (visualServoing.reset)
        {
            ros::param::get("visual_servoing/image/roi_initial",roi);
            visualServoing.reset = false;
            cout << "\nROI reseted to the center.";
        }

        std::vector<cv::Point2f> filtredCentroids = Image::filterSet(rawImg, plantCentroids, roiRange, roi, plantRadius);

        if(filtredCentroids.size() > 5) {
            Image::fitLine(rawImg, filtredCentroids, guideLine);
            Image::updateRoi(filtredCentroids, roi);
        }

        cv::line(rawImg, cv::Point(ref,0), cv::Point(ref,479), cv::Scalar(0, 255, 0), 1.5, cv::LINE_AA);

        cv::line(rawImg, cv::Point(roi-roiRange,0), cv::Point(roi-roiRange,479), cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
        cv::line(rawImg, cv::Point(roi+roiRange,0), cv::Point(roi+roiRange,479), cv::Scalar(255, 0, 0), 1, cv::LINE_AA);

        cv::line(rawImg, cv::Point(guideLine.first.y,guideLine.first.x), cv::Point(guideLine.second.y,guideLine.second.x), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

        float lineSlope = utils::calcSlope(guideLine.first, guideLine.second);
        float rho = ((utils::calcDistance(guideLine.second, guideLine.first, cv::Point(0, 0))) - ((int) rawImg.cols / 2)) / ((int) rawImg.cols / 2);

        cout << "\nNumber of points: " << filtredCentroids.size() << "\nLine slope: " << lineSlope << ", Distance: " << rho << std::endl;

        if (showImg)
        {
            cv::imshow("Processed Image",rawImg);
            cv::waitKey(1);
        }

        // Publishes cmd_vel
        if(visualServoing.controlPI)
        {
            cout << "\nUsing PI control";
            visualServoing.pubVelocity(visualServoing.velocitiesPI(rho, lineSlope));
        }
        else
        {
            visualServoing.pubVelocity(visualServoing.velocities(rho, lineSlope));
        }

        visualServoing.pubProcessedImage(rawImg);

        auto end = std::chrono::system_clock::now();
        std::cout << endl << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

        // ROS control action
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
