/*********************************************************************
 * pca.h
 * Created on : 17th Dec 2017
 *      Author: Harsh Sinha
 *
 * Principle Component Analysis using OpenCV's Matrix functions
 ********************************************************************/

#ifndef WAREHOUSE_QUAD_PCA
#define WAREHOUSE_QUAD_PCA

#include "CVInclude.h"
#include <iostream>

namespace PCA {
class pca {

    public:
        pca();
        cv::Vec4f getPrincipleComponent (std::vector<cv::Vec2i> &data_points);

    private:
        double invalid;
        double mineigenval;
};
}

#endif
