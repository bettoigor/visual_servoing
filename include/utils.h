//
// Created by adalberto-oliveira on 18/10/22.
//

#ifndef VISUAL_SERVOING_UTILS_H
#define VISUAL_SERVOING_UTILS_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <set>

class utils {
public:
    static float calcSlope(cv::Point initial, cv::Point final);
    static float calcSlopeRad(cv::Point initial, cv::Point final);
    static bool sortPoints(cv::Point p1, cv::Point p2);
    static bool sortLine(std::pair<cv::Point,cv::Point> l1,std::pair<cv::Point,cv::Point> l2);
    static std::pair<cv::Point,cv::Point>lineMean(std::vector<std::pair<cv::Point,cv::Point>> line);
    static int lineMedian(int size);
    static double calcDistance(const cv::Point &a, const cv::Point &b, const cv::Point &p);
    static std::pair<cv::Point,cv::Point>calcDistanceLines(std::vector<std::pair<cv::Point,cv::Point>> line);
};


#endif //VISUAL_SERVOING_UTILS_H
