#include "line.h"

namespace line {

line::Line():
            slope(0),
            c(0)
{}

std::vector<double> Line::getLineSlopeC (void)
{
    if (!_slopec_added)
    {
        ROS_WARN ("Trying to get slope before setting!");
    }
    std::vector<double> v(2);
    v[0] = _slope;
    v[1] = _c;
    return v;
}

std::vector<cv::Vec2i> Line::getLinePoints (void)
{
    std::vector<cv::Vec2i> v;
    std::vector<cv::Vec2i>::iterator it;
    for (it = _points.begin(); it != _points.end(); ++it)
    {
        v.push_back(*it);
    }

    return v;
}

void Line::addLinePoints (std::vector<cv::Vec2i> &points)
{
    if (!points.size())
    {
        ROS_WARN ("No Points to add");
        return;
    }

    std::vector<cv::Vec2i>::iterator it = points.begin();
    for (it; it != points.end(); ++it)
    {
        _points.push_back(*it);
    }

    return;
}

void Line::clearLinePoints (void)
{
    _points.clear();
    return;
}

void Line::setLineSlopeC (std::vector<double> &slopec)
{
    if (slopec.size() != 2)
    {
        ROS_WARN ("Invalid Input for setting Line Slope ");
        return;
    }

    _slope = slopec[0];
    _c     = slopec[1];
    _slopec_added++;
    return;
}

} /* line.cpp */