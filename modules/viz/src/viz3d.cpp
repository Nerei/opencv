#include <opencv2/viz/viz3d.hpp>
#include <opencv2/viz/viz3d_impl.hpp>


temp_viz::Viz3d::Viz3d(const std::string& name) : impl_(new VizImpl(name))
{

}

temp_viz::Viz3d::~Viz3d()
{

}


void temp_viz::Viz3d::setBackgroundColor(const Scalar& color)
{
    impl_->setBackgroundColor(color);
}

void temp_viz::Viz3d::addCoordinateSystem(double scale, const Affine3f& t, const String &id)
{
    impl_->addCoordinateSystem(scale, t, id);
}

void temp_viz::Viz3d::addPointCloud(const Mat& cloud, const Mat& colors, const String& id, const Mat& mask)
{
    impl_->addPointCloud(cloud, colors, id, mask);
}

bool temp_viz::Viz3d::addPointCloudNormals (const Mat &cloud, const Mat& normals, int level, float scale, const String& id)
{
    return impl_->addPointCloudNormals(cloud, normals, level, scale, id);
}

bool temp_viz::Viz3d::updatePointCloud(const Mat& cloud, const Mat& colors, const String& id, const Mat& mask)
{
    return impl_->updatePointCloud(cloud, colors, id, mask);
}

bool temp_viz::Viz3d::addPolygonMesh(const Mat& cloud, const Mat& colors, const Mat& mask, const std::vector<temp_viz::Vertices> &vertices, const String &id)
{
    return impl_->addPolygonMesh(cloud, colors, mask, vertices, id);
}

bool temp_viz::Viz3d::updatePolygonMesh(const Mat& cloud, const Mat& colors, const Mat& mask, const std::vector<temp_viz::Vertices> &vertices, const String& id)
{
    return impl_->updatePolygonMesh(cloud, colors, mask, vertices, id);
}

bool temp_viz::Viz3d::addPolylineFromPolygonMesh (const Mat& cloud, const std::vector<temp_viz::Vertices> &vertices, const String &id)
{
    return impl_->addPolylineFromPolygonMesh(cloud, vertices, id);
}

bool temp_viz::Viz3d::addText (const String &text, int xpos, int ypos, const Scalar& color, int fontsize, const String &id)
{
    return impl_->addText(text, xpos, ypos, color, fontsize, id);
}

bool temp_viz::Viz3d::addPolygon(const Mat& cloud, const Scalar& color, const String& id)
{
    return impl_->addPolygon(cloud, color, id);
}


bool temp_viz::Viz3d::addSphere (const cv::Point3f &center, double radius, double r, double g, double b, const std::string &id)
{
    return impl_->addSphere(center, radius, r, g, b, id);
}

void temp_viz::Viz3d::spin()
{
    impl_->spin();
}

void temp_viz::Viz3d::spinOnce (int time, bool force_redraw)
{
    impl_->spinOnce(time, force_redraw);
}

bool temp_viz::Viz3d::addPlane (const ModelCoefficients &coefficients, const String &id)
{
    return impl_->addPlane(coefficients, id);
}

bool temp_viz::Viz3d::addPlane (const ModelCoefficients &coefficients, double x, double y, double z, const String& id)
{
    return impl_->addPlane(coefficients, x, y, z, id);
}

bool temp_viz::Viz3d::removeCoordinateSystem (const String &id)
{
    return impl_->removeCoordinateSystem(id);
}
