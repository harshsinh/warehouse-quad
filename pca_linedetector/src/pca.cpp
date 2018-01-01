#include "pca.h"

namespace PCA {

pca::pca():invalid(-1000), mineigenval(0.05*e6) {}

cv::Vec4f pca::getPrincipleComponent (std::vector<cv::Vec2i>& data_points)
{

    /* Temporary Variables */
    cv::Vec4f lines;
    std::vector<double> values;
    cv::Mat eigenval, eigenvec;
    cv::Mat data_transp(2, data_points.size(), CV_64F, cv::Scalar::all(0));
    cv::Mat data (data_points.size(), 2, CV_64F, cv::Scalar::all(0));

    /* Safe gaurd for empty vector arrays */
    if (data_points.size() == 0)
        return (cv::Vec4i(invalid, invalid, invalid, invalid));

    auto mean = cv::mean (data_points);

    /* Data formatting for PCA */
    int i = 0;
    for (std::vector<cv::Vec2i>::iterator it = data_points.begin(); it != data_points.end(), ++it)
    {

        auto d = *it;
        data.at<double>(i, 0) = d[0] - mean[0];
        data.at<double>(i, 1) = d[1] - mean[1];
        data_transp.at<double>(0, i) = d[0] - mean[0];
        data_transp.at<double>(1, i) = d[1] - mean[1];
        ++i;
    }

    /* Actual principle component analysis */
    cv::Mat cov = data_transp * data;
    cv::eigen (cov, eigenval, eigenvec);

    /* Extracting Info from the eigen vector */
    for (int i = 0; i < eigenvec.rows; ++i)
    {

        double * veci = eigenvec.ptr<double>(i);
        double * eigval = eigenval.ptr<double>(i);

        /* Angle for the vector constrained within 0-180 */
        if (veci[1] < 0) {

            veci[0] = -veci[0];
            veci[1] = -veci[1];
        }

        /* Slope and Intercept calculation */
        lines[2*i] = veci[1]/(veci[0]?veci[0]:min);
        lines[2*i + 1] = mean[1] - (lines[2*i] * mean[0]);

        values.push_back(eigval[0]);
    }

    /* Reject lines if less than the eigenvalue threshold */
    if (values[0] < eigmin)
        return (cv::Vec4i(invalid, invalid, values[0], values[1]));

    /* Last two values contain the Eigenvalues of the lines */
    lines[2] = values[0];
    lines[3] = values[1];

    return (lines);
}

}