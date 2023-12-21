//
// Created by adalberto-oliveira on 18/10/22.
//

#include "../include/utils.h"

float utils::calcSlope(cv::Point initial, cv::Point final)
{
        float deltaY =  final.y - initial.y;
        float deltaX = final.x - initial.x;

        float phi = atan(deltaY/deltaX);

        return phi * 180/M_PI;
}
bool utils::sortPoints(cv::Point p1, cv::Point p2)
{
    return (p1.y) < (p2.y);
}

float utils::calcSlopeRad(cv::Point initial, cv::Point final)
{
    float deltaY =  final.y - initial.y;
    float deltaX = final.x - initial.x;

    float phi = atan(deltaY/deltaX);

    return phi;
}
std::pair<cv::Point,cv::Point>utils::lineMean(std::vector<std::pair<cv::Point,cv::Point>> line)
{
    int mean1 = 0;
    int mean2 = 0;

    for(auto &it: line)
    {
        mean1 += it.first.y;
        mean2 += it.second.y;
    }
    size_t n = line.size();

    return std::make_pair(cv::Point(line[0].first.x,mean1/n),cv::Point(line[0].second.x,mean2/n));
}
int utils::lineMedian(int size)
{
    return (size+1)/2;
}
bool utils::sortLine(std::pair<cv::Point,cv::Point> l1,std::pair<cv::Point,cv::Point> l2)
{
    return (l1.first.y < l2.first.y);
}
double utils::calcDistance(const cv::Point &a, const cv::Point &b, const cv::Point &p)
{
    double numerator = abs((b.y - a.y)*p.x - (b.x - a.x)*p.y + b.x*a.y - b.y*a.x);
    double denominator = sqrt(pow(b.y - a.y, 2) + pow(b.x - a.x, 2));

    return numerator/denominator;
}
std::pair<cv::Point,cv::Point>utils::calcDistanceLines(std::vector<std::pair<cv::Point,cv::Point>> line)
{
    std::vector<float> distances;

    for(auto &it: line)
    {
        cv::Point a = it.first;
        cv::Point b = it.second;
        cv::Point c(240,320);

        distances.push_back(utils::calcDistance(a,b,c));
    }

    auto minElemt = std::distance(distances.begin(),std::min_element(distances.begin(), distances.end()));

    return line[minElemt];
}