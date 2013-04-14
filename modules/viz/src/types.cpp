#include <opencv2/viz/types.hpp>



//////////////////////////////////////////////////////////////////////////////////////////////////////
/// cv::Color

temp_viz::Color::Color() : Scalar(0, 0, 0) {}
temp_viz::Color::Color(double red, double green, double blue) : Scalar(blue, green, red) {}

temp_viz::Color temp_viz::Color::black()   { return Color(0,   0,   0); }
temp_viz::Color temp_viz::Color::blue()    { return Color(0,   0, 255); }
temp_viz::Color temp_viz::Color::green()   { return Color(0, 255,   0); }
temp_viz::Color temp_viz::Color::cyan()    { return Color(0, 255, 255); }

temp_viz::Color temp_viz::Color::red()     { return Color(255,   0,   0); }
temp_viz::Color temp_viz::Color::magenta() { return Color(255,   0, 255); }
temp_viz::Color temp_viz::Color::yellow()  { return Color(255, 255,   0); }
temp_viz::Color temp_viz::Color::white()   { return Color(255, 255, 255); }

temp_viz::Color temp_viz::Color::gray()   { return Color(128, 128, 128); }

