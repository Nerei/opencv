#pragma once

#include <vector>
#include <opencv2/core/cvdef.h>
#include <opencv2/core.hpp>


namespace temp_viz
{
    //qt creator hack
    typedef cv::Scalar Scalar;
    typedef cv::Mat Mat;
    typedef std::string String;


    struct CV_EXPORTS ModelCoefficients
    {
        std::vector<float> values;
    };


    class CV_EXPORTS Color : public Scalar
    {
    public:
        Color();
        Color(double gray);
        Color(double red, double green, double blue);

        Color(const Scalar& color);

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


    struct CV_EXPORTS Vertices
    {
        std::vector<unsigned int> vertices;
    };

    class CV_EXPORTS Mesh3d
    {
    public:
        typedef cv::Ptr<Mesh3d> Ptr;

        Mat cloud, colors;
        std::vector<Vertices> polygons;
    };

}
