/*********************************************************************
 * pca.h
 * Created on : 1st Jan 2018
 *      Author: Harsh Sinha
 *
 * Principle Component Analysis using OpenCV's Matrix functions
 ********************************************************************/

#ifndef __WAREHOUSE_QUAD_PCA__
#define __WAREHOUSE_QUAD_PCA__

#include "CVInclude.h"

namespace warehouse_pcadetector {
class pca {

    public:
        pca();
        cv::Vec4f getPrincipleComponent (std::vector<cv::Vec2i> &data_points);

    private:
        double invalid;
        double mineigenval;
};
}

#endif /* __WAREHOUSE_QUAD_PCA__ */
