/*
 * test_slic.cpp.
 *
 * Written by: Pascal Mettes.
 * Revisioned by: Satya Panuganti
 *
 * This file creates an over-segmentation of a provided image based on the SLIC
 * superpixel algorithm, as implemented in slic.h and slic.cpp.
 */
 
#include "CVInclude.h"
#include <cmath>
#include <vector>
#include <iostream>
#include <cstdlib>

#include "slic.h"

int main(int argc, char *argv[])
{
    /* Load the image and convert to Lab colour space. */
    auto image = cv::imread (argv[1], 1);
    auto lab_image = image.clone();
    
    cv::cvtColor(image, lab_image, CV_BGR2Lab);
    
    /* Yield the number of superpixels and weight-factors from the user. */
    int w = image.rows, h = image.cols;
    int nr_superpixels = std::atoi(argv[2]);
    int nc = std::atoi(argv[3]);

    double step = std::sqrt((w * h) / nr_superpixels);
    
    /* Perform the SLIC superpixel algorithm. */
    Slic slic;
    slic.generate_superpixels(lab_image, step, nc);
    slic.create_connectivity(lab_image);

    /* Display the contours and show the result. */
    slic.display_contours(image, cv::Vec3b (0,0,255)); // in BGR format
    cv::imshow ("result", image);
    cv::waitKey (0);
    cv::imwrite (argv[4], image);
}
