#pragma once

#if !defined YES_I_AGREE_THAT_VIZ_API_IS_NOT_STABLE_NOW_AND_BINARY_COMPARTIBILITY_WONT_BE_SUPPORTED
    //#error "Viz is in beta state now. Please define macro above to use it"
#endif


#include <opencv2/core/cvdef.h>
#include <opencv2/core/affine.hpp>
#include <opencv2/core.hpp>


#include <string>
#include <opencv2/viz/types.hpp>


namespace temp_viz
{
    using cv::Scalar;
    using cv::Affine3f;
    using cv::Mat;
    typedef std::string String;


    class CV_EXPORTS Viz3d
    {
    public:

        //syntax colorrizer hack for qtcraetor
        typedef temp_viz::Scalar Scalar;
        typedef temp_viz::Mat Mat;
        typedef temp_viz::Affine3f Affine3f;
        typedef cv::Point3f Point3f;
        // end if hack


        typedef cv::Ptr<Viz3d> Ptr;

        Viz3d(const std::string& name = "Viz");
        ~Viz3d();

        void setBackgroundColor(const Color& color = Color::black());

        void addCoordinateSystem(double scale, const Affine3f& t, const String &id = "coordinate");

        void addPointCloud(const Mat& cloud, const Mat& colors, const String& id = "cloud", const Mat& mask = Mat());

        bool addPointCloudNormals (const Mat &cloud, const Mat& normals, int level = 100, float scale = 0.02f, const String &id = "cloud");




        bool addPlane (const ModelCoefficients &coefficients, const String &id = "plane");
        bool addPlane (const ModelCoefficients &coefficients, double x, double y, double z, const String &id = "plane");
        bool removeCoordinateSystem (const String &id = "coordinate");


        bool updatePointCloud (const Mat& cloud, const Mat& colors, const String& id = "cloud", const Mat& mask = Mat());


        bool addPolygonMesh (const Mesh3d& mesh, const String &id = "polygon");
        bool updatePolygonMesh (const Mesh3d& mesh, const String &id = "polygon");

        bool addPolylineFromPolygonMesh (const Mat& cloud, const std::vector<temp_viz::Vertices> &vertices, const String &id = "polyline");


        bool addText (const String &text, int xpos, int ypos, const Scalar& color = Scalar(255, 255, 255), int fontsize = 10, const String &id = "");


        bool addPolygon(const Mat& cloud, const Scalar& color = Scalar(255, 255, 255), const String &id = "polygon");

        bool addSphere (const Point3f &center, double radius, double r, double g, double b, const String &id = "sphere");


        void spin ();
        void spinOnce (int time = 1, bool force_redraw = false);

    private:
        Viz3d(const Viz3d&);
        Viz3d& operator=(const Viz3d&);

        struct VizImpl;
        VizImpl* impl_;
    };
}



