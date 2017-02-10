#ifndef __ImageUtils_HPP__
#define __ImageUtils_HPP__

/* ImageUtils
 * --------------------------
 * Nice software tools to work with images and point clouds together
 *
 * */

//--Eigen
#include <Eigen/Eigen>
//--OpenCV
#include <opencv2/opencv.hpp>


void eigen2mat(Eigen::MatrixXf& red, Eigen::MatrixXf& green, Eigen::MatrixXf& blue, cv::Mat &dst);
void eigen2mat(Eigen::MatrixXd &grayscale, cv::Mat& dst);

void eigen2file(Eigen::MatrixXf& red, Eigen::MatrixXf& green, Eigen::MatrixXf& blue, const std::string& filename);
void eigen2file(Eigen::MatrixXd& grayscale, const std::string& filename);

#endif // __ImageUtils_HPP__
