/*********************************************************************
 * line.h
 * Created on : 1st Jan 2018
 *      Author: Harsh Sinha
 *
 * Basic Line Element
 ********************************************************************/

#ifndef __WAREHOUSE_QUAD_LINE__
#define __WAREHOUSE_QUAD_LINE__

#include "CVInclude.h"
#include <vector>
#include <ros/console.h>

namespace LINE {
class line {

    public:
        line();

    public:
        std::vector<double> getLineSlopeC (void);
        std::vector<cv::Vec2i> getLinePoints (void);
        void addLinePoints (std::vector<cv::Vec2i> &points);
        void clearLinePoints (void);
        void setLineSlopeC (std::vector<double> &slopec);

    private:
        double _slope, _c;
        std::vector<cv::Vec2i> _points;
        int _count = 0;
};
}

#endif /* __WAREHOUSE_QUAD_LINE__ */