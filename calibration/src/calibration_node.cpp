#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibration");

    int image_count = 0;        //图片个数

    cv::Size image_size;        //图片大小
    cv::Size board_size = cv::Size(9, 6);   //角点个数


    std::vector<cv::Point2f> image_points_buf;  //一张图片中的所有角点
    std::vector<std::vector<cv::Point2f> > image_points_seq;    //所有图片中的角点

    //实验中只有５张图片
    for(int i = 1; i < 6; i ++)
    {
        //读图片
        std::stringstream ss;
        ss << i;
        ss << ".jpg";
        cv::Mat imageInput = cv::imread(ss.str());
        cv::cvtColor(imageInput, imageInput, CV_RGB2GRAY);  //CV_RGB --> CV_GRAY

        //图片大小
        image_size.width = imageInput.cols;
        image_size.height = imageInput.rows;

        //提取棋盘格角点
        if(!cv::findChessboardCorners(imageInput, board_size, image_points_buf))
        {
            std::cout << "cannot find chessboard corners" << std::endl;
            continue;
        }
        else
        {
            //亚像素精确化
            cv::find4QuadCornerSubpix(imageInput, image_points_buf, cv::Size(100, 100));
            
            image_points_seq.push_back(image_points_buf);

            image_count ++;
        }
    }


    cv::Size square_size = cv::Size(26, 26);  //格子长和宽(单位毫米)
    std::vector<std::vector<cv::Point3f> > object_points;
    cv::Mat cameraMatrix = cv::Mat(3, 3, 0, cv::Scalar::all(0)); //内参矩阵
    cv::Mat distCoeffs = cv::Mat(1, 5, 0, cv::Scalar::all(0));   //畸变参数

    std::vector<cv::Mat> tvecsMat;  //每副图片的平移向量
    std::vector<cv::Mat> rvecsMat;  //每副图片的旋转向量

    //世界坐标系下的角点坐标
    for(int t = 0; t < image_count; t ++)
    {
        std::vector<cv::Point3f> points;
        for(int i = 0; i < board_size.height; i ++)
        {
            for(int j = 0; j < board_size.width; j ++)
            {
                cv::Point3f point;
                point.x = i * square_size.width;
                point.y = j * square_size.height;
                point.z = 0;

                points.push_back(point);
            }
        }

        object_points.push_back(points);
    }

    //计算相机内参和畸变参数
    cv::calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);

    std::cout << "K : " << cameraMatrix << std::endl;
    std::cout << "dist : " << distCoeffs << std::endl;
    
    double total_err = 0;
    for(int i = 0; i < image_count; i ++)
    {
        std::vector<cv::Point3f> tempPointSet = object_points[i];   //第i张图片中角点对应的世界坐标系下坐标
        
        //世界坐标 ---> 像素坐标
        //std::vector<cv::Point3f> --> std::vector<cv::Point2f>
        std::vector<cv::Point2f> image_points2;
        cv::projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
    
        //提取到的像素坐标
        std::vector<cv::Point2f> tempImagePoint = image_points_seq[i];

        /*
        //类型转换
        cv::Mat tempImagePointMat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);
        cv::Mat image_points2Mat = cv::Mat(1, image_points2.size(), CV_32FC2);
        for(int j = 0; j < tempImagePoint.size(); j ++)
        {
            image_points2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points2[j].x, image_points2[j].y);
            tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
        }

        //计算单张图片重投影误差
        double err = norm(image_points2Mat, tempImagePointMat, cv::NORM_L2);
        */

        double err = 0;
        for(int j = 0; j < tempImagePoint.size(); j ++)
        {
            err += sqrt(pow(tempImagePoint[j].x - image_points2[j].x, 2) + pow(tempImagePoint[j].y - image_points2[j].y, 2));
        }

        err /= tempImagePoint.size();
        std::cout << "第 " << i << "张图片重投影误差: " << err << std::endl;
            
        total_err += (err / image_count);

    }
    std::cout << "重投影误差: " << total_err << std::endl;

    return 0;
}

