#pragma once

#include <vector>
#include <opencv2/core/cvdef.h>
#include <opencv2/core.hpp>


namespace temp_viz
{
    //qt creator hack
    typedef cv::Scalar Scalar;


    struct CV_EXPORTS Vertices
    {
        std::vector<unsigned int> vertices;
    };


    struct CV_EXPORTS ModelCoefficients
    {
        std::vector<float> values;
    };


    class Color : public Scalar
    {
        Color();
        Color(double red, double green, double blue);

        static Color black();
        static Color blue();
        static Color green();
        static Color cyan();

        static Color red();
        static Color magenta();
        static Color yellow();
        static Color white();

        static Color gray();
    };
}
