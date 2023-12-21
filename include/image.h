//
// Created by willian on 27/10/22.
//

#ifndef VISUAL_SERVOING_IMAGE_H
#define VISUAL_SERVOING_IMAGE_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>


class Image {
public:
    static std::vector<std::vector<cv::Point>>getMaskFeature(cv::Mat &img);
    static std::vector<cv::Point2f> getSetCenters(cv::Mat &img, std::vector<std::vector<cv::Point>> &contourns);
    static std::vector<cv::Point2f> filterSet(cv::Mat &img, std::vector<std::vector<cv::Point>>& contours, int roiRange, int roi, float plantRadius);
    static void fitLine(cv::Mat &img, std::vector<cv::Point2f>& contornsCenters, std::pair<cv::Point, cv::Point>& linePoint);
    static void updateRoi(std::vector<cv::Point2f> &points, int &roi);
};


#endif //VISUAL_SERVOING_IMAGE_H
