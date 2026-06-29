#include "pch.h"
#include "Ransac3D.h"

#pragma region RANSAC
Ransac3D::Ransac3D(CP all_data_)
{
    for (int i = 0; i < all_data_->size(); i++)
    {
        valid_num.push_back(pair<int, int>(i, 0));
    }
    all_data = all_data_;
}

void Ransac3D::run()
{
    for (int _ = 0; _ < itera_num; _++)
    {
        fit();
        if (evaluate()) return;
    }
    useBest();
}

#pragma endregion



void RansacBall::fit()
{
    CP cloud = sample();
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
    a = coeffs[0];
    b = coeffs[1];
    c = coeffs[2];
    float r_squared = coeffs[3] + a * a + b * b + c * c;
    r = std::sqrt(r_squared);
    
    //if(r<10.1&&r>9.9)
    //{
    //    // 可视化原始点云和拟合球
    //    cout << r << endl;
    //    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Sphere Fitting"));
    //    viewer->addPointCloud<pcl::PointXYZ>(cloud, "original cloud");
    //    viewer->addSphere(PointT(a, b, c), r, 1.0, 0.0, 0.0, "fitted sphere");
    //    viewer->spin();
    //}
    //Tool::showComparison


}

bool RansacBall::evaluate()
{
    if (ref_r > 0 && abs(r - ref_r) > threshold)
    {
        for (auto& i : select_samples)
        {
            valid_num[i].second--;
            std::sort(valid_num.begin(), valid_num.end(), [](const std::pair<int, int>& a, const std::pair<int, int>& b) {return a.second > b.second; });
        }
        return false;
    }

    int success_num = 0;
    for (int i=0;i< valid_num.size();i++)
    {
        PointT p = all_data->points[valid_num[i].first];
        float dx = p.x - a;
        float dy = p.y - b;
        float dz = p.z - c;
        //cout << sqrt(abs(r * r - dx * dx + dy * dy + dz * dz)) << endl;
        //// 可视化原始点云和拟合球
        //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Sphere Fitting"));
        //viewer->addPointCloud<pcl::PointXYZ>(all_data, "original cloud");
        //// 添加拟合的球模型
        //viewer->addSphere(PointT(a,b,c), r, 1.0, 0.0, 0.0, "fitted sphere");
        //viewer->spin();
        //if (r < 10.1 && r>9.9)
        //{
        //    cout << abs(r * r - dx * dx - dy * dy - dz * dz) << endl;
        //}
        if (abs(r * r - dx * dx - dy * dy - dz * dz) < threshold * threshold)
        {
            success_num++;
            valid_num[i].second++;
        }
    }
    if (success_num > max_score)
    {
        max_score = success_num;
        best_a = a;
        best_b = b;
        best_c = c;
        best_r = r;
    }
    //if(success_num>0)
    //    cout << "valid_ratio: " << (float)success_num / all_data->size() << endl;
    std::sort(valid_num.begin(), valid_num.end(), [](const std::pair<int, int>& a, const std::pair<int, int>& b) {return a.second > b.second; });
    return success_num > all_data->size() * 0.8;
}

void RansacBall::useBest()
{
    a = best_a;
    b = best_b;
    c = best_c;
    r = best_r;
}

//TODO
void RansacBall::runM()
{
    #pragma omp parallel for
    for (int _ = 0; _ < itera_num; _++)
    {
        auto ret = fitM();
        #pragma omp critical
        {
            bool done = evaluateM(ret);
            if (done)
                end = true;
        }
    }
    useBest();
}

Eigen::VectorXf RansacBall::fitM()
{
    if (end) return Eigen::VectorXf();
    CP cloud = sample();
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

    //// 提取球心 (a, b, c) 和半径 r
    //a = coeffs[0];
    //b = coeffs[1];
    //c = coeffs[2];
    //float r_squared = coeffs[3] + a * a + b * b + c * c;
    //r = std::sqrt(r_squared);

    coeffs[3] = sqrt(coeffs[3] + coeffs[0] * coeffs[0] + coeffs[1] * coeffs[1] + coeffs[2] * coeffs[2]);

    return coeffs;
}

bool RansacBall::evaluateM(Eigen::VectorXf ball)
{
    if (end) return false;
    if (ref_r > 0 && abs(ball[3] - ref_r) > threshold)
    {
        for (auto& i : select_samples)
        {
            #pragma omp critical
            {
                valid_num[i].second--;
            }
        }
        #pragma omp critical
        {
            std::sort(valid_num.begin(), valid_num.end(), [](const std::pair<int, int>& a, const std::pair<int, int>& b) {return a.second > b.second; });
        }
        return false;
    }

    int success_num = 0;
    for (int i = 0; i < valid_num.size(); i++)
    {
        PointT p = all_data->points[valid_num[i].first];
        float dx = p.x - a;
        float dy = p.y - b;
        float dz = p.z - c;
        //cout << sqrt(abs(r * r - dx * dx + dy * dy + dz * dz)) << endl;
        //// 可视化原始点云和拟合球
        //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Sphere Fitting"));
        //viewer->addPointCloud<pcl::PointXYZ>(all_data, "original cloud");
        //// 添加拟合的球模型
        //viewer->addSphere(PointT(a,b,c), r, 1.0, 0.0, 0.0, "fitted sphere");
        //viewer->spin();
        //if (r < 10.1 && r>9.9)
        //{
        //    cout << abs(r * r - dx * dx - dy * dy - dz * dz) << endl;
        //}
        if (abs(r * r - dx * dx - dy * dy - dz * dz) < threshold * threshold)
        {
            success_num++;
            #pragma omp critical
            {
                valid_num[i].second++;
            }
        }
    }
    if (success_num > max_score)
    {
        max_score = success_num;
        best_a = a;
        best_b = b;
        best_c = c;
        best_r = r;
    }
    //if(success_num>0)
    //    cout << "valid_ratio: " << (float)success_num / all_data->size() << endl;
    #pragma omp critical
    {
        std::sort(valid_num.begin(), valid_num.end(), [](const std::pair<int, int>& a, const std::pair<int, int>& b) {return a.second > b.second; });
    }
    return success_num > all_data->size() * 0.8;
}
