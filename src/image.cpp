//
// Created by willian on 27/10/22.
//

#include "../include/image.h"

std::vector<std::vector<cv::Point>>Image::getMaskFeature(cv::Mat &img) {

    cv::Mat imgTmp;
    cv::GaussianBlur(img,imgTmp,cv::Size(5,5),5);

    cv::Mat BGR[3];
    cv::split(imgTmp, BGR);

    cv::Mat mask = (2*BGR[1] - BGR[2] - BGR[0])/2;

    for (int c=0;c<img.cols;c++) {
        for (int r = 0; r < img.rows; r++) {
            int p = mask.at<uchar>(r, c);
            float iP = (float) p;
            if (iP / 255 < 0.04 or iP / 255 > 0.08)
                mask.at<uchar>(r, c) = 0;
            else
                mask.at<uchar>(r, c) = 255;
        }
    }

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    findContours(mask, contours, hierarchy, cv::RETR_TREE,
                 cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    cv::RNG rng(12345);

    cv::Mat img_contour = cv::Mat::zeros(mask.size(), CV_8UC3);
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    for(size_t i = 0; i < contours.size(); i++){
        drawContours(img_contour, contours, i, color, 1, 8, hierarchy, 0, cv::Point());
    }

    return contours;

}

std::vector<cv::Point2f> Image::getSetCenters(cv::Mat &img, std::vector<std::vector<cv::Point>> &contours) {
    std::vector<std::vector<cv::Point>> contours_poly(contours.size());
    std::vector<cv::Point2f> center(contours.size());
    std::vector<float> radius(contours.size());

    for (size_t i = 0; i < contours.size(); i++) {
        approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 2, true);
        minEnclosingCircle((cv::Mat)contours_poly[i], center[i], radius[i]);
        cv::circle(img, cv::Point(center[i].x, center[i].y),3, cv::Scalar(0, 0, 255),cv::FILLED, 8,0);
    }

    return center;
}

std::vector<cv::Point2f> Image::filterSet(cv::Mat &img, std::vector<std::vector<cv::Point>> &contours, int roiRange, int roi, float plantRadius) {
    std::vector<std::vector<cv::Point>> contours_poly(contours.size());
    std::vector<cv::Point2f> center(contours.size());
    std::vector<float> radius(contours.size());
    std::vector<cv::Point2f> midCenters;

    for (size_t i = 0; i < contours.size(); i++) {
        approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 1, true);
        minEnclosingCircle((cv::Mat)contours_poly[i], center[i], radius[i]);
        if (center[i].x >= roi - roiRange &&
            center[i].x <= roi + roiRange &&
            radius[i] >= plantRadius) {
            midCenters.push_back(center[i]);
            cv::circle(img, cv::Point(center[i].x, center[i].y),3, cv::Scalar(0, 0, 255),cv::FILLED, 8,0);
        }

    }

    return midCenters;
}

void Image::fitLine(cv::Mat &img, std::vector<cv::Point2f> &contourCenters, std::pair<cv::Point, cv::Point>& linePoint) {
    if(!contourCenters.empty() ){
        cv::Vec4f line;
        cv::fitLine(contourCenters, line, cv::DIST_WELSCH, 0, 0.01, 0.01);

        int leftY = (-line[2]*line[1]/line[0])+line[3];
        int rightY = ((img.cols-line[2])*line[1]/line[0])+line[3];

        linePoint.first = cv::Point(rightY, img.cols-1);
        linePoint.second = cv::Point(leftY, 0);

    }
}

void Image::updateRoi(std::vector<cv::Point2f> &points, int &roi){
    int axis = 0;
    for(auto &it: points){
        axis += it.x;
    }

    roi = axis / points.size();
}
