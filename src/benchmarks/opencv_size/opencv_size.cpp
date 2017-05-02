#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class ALoop : public cv::ParallelLoopBody
{
public:
    ALoop() {}
    void operator()(const cv::Range& r) const {}
};

int main()
{
    // ### Types ###
    cv::Size size(100, 100);
    cv::Point p;
    cv::Point2d p2d;
    cv::Point2f p2f;
    cv::Point3d p3d;
    cv::Point3f p3f;
    cv::RNG rng;
    cv::Range range;
    cv::Rect rect;
    cv::Rect2f rect2f;
    cv::RotatedRect rr;
    cv::Scalar color(1.0, 1.0, 1.0);
    cv::Size sz;
    cv::Size2d size2d;
    cv::Size2f size2f;
    cv::Vec2d vec2d;
    cv::Vec2f vec2f;
    cv::Vec3b vec3b;
    cv::Vec3d vec3d;
    cv::Vec3f vec3f;
    cv::Vec4b vec4b;
    cv::Vec4f vec4f;
    cv::Vec6d vec6d;

    // ### Images ###
    cv::Mat image(size, CV_8UC3, cv::Scalar::all(0)), image2;
    cv::Mat1b mat1b(size);
    cv::Mat1d mat1d(size, 1.0);
    cv::Mat1f mat1f(size, 1.f);
    cv::Mat3b mat3b(size);
    cv::Mat3f mat3f(size);
    cv::Mat4b mat4b(size);
    cv::Matx22f matx22f;
    cv::Matx31d matx31d;
    cv::Matx33d matx33d;
    cv::Matx33f matx33f;
    cv::Matx41d matx41d;
    cv::Matx44f matx44f;
    cv::PCA pca;

    // ### arithmetic ###
    double d = cv::norm(cv::Point2f(0.0, 0.0) - cv::Point2f(1.0, 1.0));

    cv::Scalar total = cv::sum(mat1f);
    cv::add(mat1f, mat1f, mat1f);
    cv::subtract(mat1f, mat1f, mat1f);
    cv::divide(mat1f, mat1f, mat1f);
    cv::pow(mat1f, 1.0, mat1f);
    cv::normalize(mat1f, mat1f, 0.f, 1.f, cv::NORM_MINMAX, CV_32F);

    cv::countNonZero(image);
    cv::findNonZero(image, mat1b);

    // ### drawing ###
    cv::applyColorMap(image, image, cv::COLORMAP_RAINBOW);
    cv::arrowedLine(image, { 0, 0 }, { image.cols, image.rows }, { 0, 255, 0 }, 1, 8);
    cv::line(image, { 0, 0 }, { image.cols, image.rows }, { 0, 255, 0 }, 1, 8);
    cv::rectangle(image, { 0, 0 }, { 1, 1 }, { 0, 255, 0 }, 1, 8);
    cv::ellipse(image, { { 0.f, 0.f }, { 10.f, 10.f }, 0.f }, { 0, 255, 0 }, -1);
    cv::circle(image, { image.cols / 2, image.rows / 2 }, image.cols / 4, { 0, 255, 0 }, 1, 8);

    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

    // ### linear algebra ###
    cv::Moments mom = cv::moments(image);

    cv::Matx22f C(mom.mu20 / mom.m00, mom.mu11 / mom.m00, mom.mu11 / mom.m00, mom.mu02 / mom.m00);
    cv::Matx22f vecs;
    cv::Vec2f vals;
    cv::eigen(C, vals, vecs);

    cv::Mat1f A(2, 2, 1.f), b(2, 1, 1.f), sol(2, 1);
    cv::solve(A, b, sol, cv::DECOMP_NORMAL | cv::DECOMP_LU);

    cv::multiply(mat1f, mat1f, mat1f);
    cv::gemm(mat1f, mat1f, 1.0, mat1f, 1.0, mat1f);
    cv::invert(mat1f, mat1f);

    // ### polygon ###
    std::vector<std::vector<cv::Point>> points{ { { 0, 0 }, { 0, 1 }, { 1, 1 } } };
    cv::Rect bb = cv::boundingRect(points[0]);
    cv::fillPoly(image, points, { 0, 255, 0 });
    cv::polylines(image, points, true, { 0, 255, 0 });
    cv::pointPolygonTest(points, { 0.5f, 0.5f }, true);

    // ### channels ###
    std::vector<cv::Mat1b> channels;
    cv::split(image, channels);
    cv::merge(channels, image);
    cv::extractChannel(image, mat1b, 0);

    cv::Scalar mu, sigma;
    cv::meanStdDev(mat1f, mu, sigma);
    cv::minMaxLoc(mat1f, &vec2d[0], &vec2d[1]);

    // ### parallel ####
    ALoop loop;
    cv::parallel_for_({ 0, 10 }, loop);

    // ### image/shape ###
    cv::Mat1f result;
    cv::reduce(mat1f, result, 1, cv::REDUCE_SUM);
    cv::copyMakeBorder(image, image2, 1, 1, 1, 1, cv::BORDER_CONSTANT);
    cv::resize(image, image2, image2.size());
    cv::transpose(image, image);
    cv::vconcat(image, image, image2);
    cv::hconcat(image, image, image2);
    cv::flip(image, image, 0);
    cv::dilate(image, image2, {}, { -1, -1 }, 2);
    cv::repeat(image, 2, 2, image2);

    // ### image warp ####
    cv::warpAffine(image, image2, cv::Matx23f::ones(), image.size());
    cv::remap(image, image, mat1f, mat1f, cv::INTER_LINEAR);

    cv::swap(image, image2);
    cv::swap(image2, image);

    // ### image processing ###
    cv::Matx13f fx(1.f, 1.f, 1.f);
    cv::sepFilter2D(mat1f, result, CV_32FC1, cv::Mat1f(fx), cv::Mat1f(fx.t()));

    // ### IO ###
    cv::FileStorage fs("/tmp/file.xml", cv::FileStorage::WRITE);
    fs << "size" << size;
    cv::FileNode node = fs["thing"];

    cv::imwrite("/tmp/out.png", image);

    cv::imshow("image", image);
    cv::waitKey(1);
}
