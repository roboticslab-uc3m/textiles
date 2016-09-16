#include "ImageUtils.hpp"

void eigen2mat(Eigen::MatrixXf &red, Eigen::MatrixXf &green, Eigen::MatrixXf &blue, cv::Mat& dst)
{
    if (red.cols() != blue.cols() || red.cols() != green.cols() || blue.cols() != green.cols())
    {
        std::cerr << "Error: Different number of columns in source channels" << std::endl;
        return;
    }

    if (red.rows() != blue.rows() || red.rows() != green.rows() || blue.rows() != green.rows())
    {
        std::cerr << "Error: Different number of rows in source channels" << std::endl;
        return;
    }

    //-- Eigen to OpenCV to convert RGB image as image (Quick and dirty)
    //------------------------------------------------------------------------------
    int width = red.cols();
    int height = red.rows();
    dst= cv::Mat(height, width, CV_8UC3);
    uint8_t* image_ptr = (uint8_t*)dst.data;

    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
        {
            image_ptr[i*dst.cols*3 + j*3 + 0] = blue(i, j); // B
            image_ptr[i*dst.cols*3 + j*3 + 1] = green(i, j);// G
            image_ptr[i*dst.cols*3 + j*3 + 2] = red(i, j);  // R
        }

}

void eigen2mat(Eigen::MatrixXd &grayscale, cv::Mat &dst)
{
    int width = grayscale.cols();
    int height = grayscale.rows();
    dst = cv::Mat(height, width, CV_8UC1);
    uint8_t* image_ptr = (uint8_t*)dst.data;

    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
            image_ptr[i*dst.cols + j] = grayscale(i, j);
}

void eigen2file(Eigen::MatrixXf &red, Eigen::MatrixXf &green, Eigen::MatrixXf &blue, const std::string &filename)
{
    cv::Mat image;
    eigen2mat(red, green, blue, image);
    cv::imwrite(filename, image);
}

void eigen2file(Eigen::MatrixXd &grayscale, const std::string &filename)
{
    cv::Mat image;
    eigen2mat(grayscale, image);
    cv::imwrite(filename, image);
}
