#include "pch.h"
#include "BallCalibration.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>

#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include"Ransac3D.h"

Eigen::VectorXf BallCalibration::fitBallRansac(CP cloud, string id)
{
    // 创建 RANSAC 球模型
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr sphere_model(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));

    // RANSAC 拟合
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(sphere_model);
    ransac.setDistanceThreshold(0.003); // 设置点到球的最大距离
    //ransac.setMaxIterations(10000);
    ransac.computeModel();
    //ransac.sa

    // 获取拟合结果
    Eigen::VectorXf coefficients;
    ransac.getModelCoefficients(coefficients);

    //std::cout << "拟合球参数:" << std::endl;
    //std::cout << "中心: (" << coefficients[0] << ", " << coefficients[1] << ", " << coefficients[2] << ")" << std::endl;
    std::cout << id<<"的RAN半径: " << coefficients[3] << ", 误差: "<<abs(10- coefficients[3]) << std::endl;



    // 获取球内点的索引
    std::vector<int> inliers;
    ransac.getInliers(inliers);
    CP sphere_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, inliers, *sphere_points);
    //if (abs(coefficients[3] - 10) > 0.1)
    //{
    //    Tool::showComparison(cloud, sphere_points, 1, 3);
    //    //fitBallMLS(sphere_points);
    //}
    //// 可视化原始点云和拟合球
    //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Sphere Fitting"));
    //viewer->addPointCloud<pcl::PointXYZ>(cloud, "original cloud");
    //viewer->addPointCloud<pcl::PointXYZ>(sphere_points, "sphere points");


    //// 添加拟合的球模型
    //viewer->addSphere(center, coefficients[3], 1.0, 0.0, 0.0, "fitted sphere");

    //viewer->spin();
    return coefficients;
}

Eigen::VectorXf BallCalibration::fitBallRansac3D(CP cloud, string id)
{
    RansacBall rb(cloud);
    rb.sample_num = 100;
    //rb.itera_num = 10000;
    rb.ref_r = 10;
    rb.run();
    cout << id<<"的rb: " << rb.r << endl;
    Eigen::VectorXf ret(4);
    ret[0] = rb.a;
    ret[1] = rb.b;
    ret[2] = rb.c;
    ret[3] = rb.r;
    return ret;
}

//TODO
Eigen::VectorXf BallCalibration::fitBallMLS(CP cloud)
{
    Tool::show(cloud);
    // 设置矩阵和向量进行最小二乘法拟合
    Eigen::MatrixXf A(cloud->size(), 4);
    Eigen::VectorXf B(cloud->size());

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        A(i, 0) = 2 * cloud->points[i].x;
        A(i, 1) = 2 * cloud->points[i].y;
        A(i, 2) = 2 * cloud->points[i].z;
        A(i, 3) = 1;

        B(i) = cloud->points[i].x * cloud->points[i].x +
            cloud->points[i].y * cloud->points[i].y +
            cloud->points[i].z * cloud->points[i].z;
    }

    // 求解最小二乘问题 A * [a, b, c, r^2] = B
    Eigen::VectorXf coeffs = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

    // 提取球心 (a, b, c) 和半径 r
    float a = coeffs[0];
    float b = coeffs[1];
    float c = coeffs[2];
    float r_squared = coeffs[3] + a * a + b * b + c * c;
    float r = std::sqrt(r_squared);
    cout << "MLS半径： " << r << endl;
    coeffs[0] = a;
    coeffs[1] = b;
    coeffs[2] = c;
    coeffs[3] = r;
    return coeffs;
}

Eigen::VectorXf BallCalibration::fitBall(CP cloud, string id)
{
    //fitBallMLS(cloud);
    return fitBallRansac(cloud, id);
    //auto ret = fitBallRansac3D(cloud, id);
    //if (ret[3] == 0)
    //    return fitBallRansac(cloud, id);
    //else
    //    return ret;
}

void BallCalibration::addBall(VCP balls)
{
    if (ball_center.size() == 0)
    {
        for (int i = 0; i < balls.size(); i++)
        {
            CP cloud(new CloudT());
            ball_center.push_back(cloud);
        }
    }
    if (balls.size() != ball_center.size())
        throw logic_error("点云数目前后不一致");

    #pragma omp parallel for
    for (int i = 0; i < balls.size(); i++)
    {
        Eigen::VectorXf coff = fitBall(balls[i], to_string(i));
        PointT center(coff[0], coff[1], coff[2]);
        ball_center[i]->push_back(center);
    }
}

// 计算两幅点云之间的RMSE
float calculateRMSE(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
    // 检查点云大小是否一致
    if (cloud1->size() != cloud2->size()) {
        std::cerr << "Error: Point clouds must have the same size!" << std::endl;
        return -1.0f;
    }

    float sumSquaredDistances = 0.0f;

    for (size_t i = 0; i < cloud1->size(); ++i) {
        const auto& p1 = cloud1->points[i];
        const auto& p2 = cloud2->points[i];
        float dx = p1.x - p2.x;
        float dy = p1.y - p2.y;
        float dz = p1.z - p2.z;

        // 计算平方距离并累加
        sumSquaredDistances += dx * dx + dy * dy + dz * dz;
        cout << "dis one: " << sqrt(dx * dx + dy * dy + dz * dz) << endl;
    }

    // 计算平均平方误差并开方
    float rmse = std::sqrt(sumSquaredDistances / cloud1->size());
    return rmse;
}

Eigen::Matrix4f BallCalibration::registrationTwo(CP source_cloud, CP target_cloud)
{
    Eigen::Matrix4f transformation_matrix;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(*source_cloud, *target_cloud, transformation_matrix);
    
    CP src_copy(new CloudT);
    *src_copy = *source_cloud;
    pcl::transformPointCloud(*src_copy, *src_copy, transformation_matrix);
    Tool::showComparison(src_copy, target_cloud);
    cout << "RMSE: " << calculateRMSE(src_copy, target_cloud) << endl;;

    return transformation_matrix;
}


void BallCalibration::updateCalibration()
{
    transforms.clear();
    //for(int i =0;i< ball_center.size(); i++)
    //    transforms.push_back(Eigen::Matrix4f::Identity());//第一个点云不变

    transforms.resize(ball_center.size());
    //#pragma omp parallel for
    for (int i = 0; i < ball_center.size(); i++)
    {
        auto tr = registrationTwo(ball_center[i], ball_center[1]);

        #pragma omp critical
        {
            transforms[i] = tr;
        }
    }
}


CP BallCalibration::Calibration(VCP clouds)
{
    if (clouds.size() != transforms.size())
        throw logic_error("输入点云与标定数目不一致");
    CP ret(new CloudT());
    
    for (int i=0;i<clouds.size();i++)
    {
        pcl::transformPointCloud(*clouds[i], *clouds[i], transforms[i]);
        pcl::io::savePCDFileBinary(to_string(i) + ".pcd", *clouds[i]);
        //Tool::showComparison(clouds[i], ret);
        *ret += *clouds[i];
    }

    return ret;
}

void BallCalibration::save(int num, string file_name)
{
    czxTool::saveMatrix4f(transforms, num, file_name);
}

void BallCalibration::load(int num, string file_name)
{
    transforms = czxTool::loadMatrix4f(num, file_name);
}
