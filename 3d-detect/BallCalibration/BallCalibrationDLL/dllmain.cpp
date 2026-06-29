// dllmain.cpp : 定义 DLL 应用程序的入口点。
#include "pch.h"
#include"BallCalibration.h"
#include <pcl/common/pca.h>


BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
                     )
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
        break;
    }
    return TRUE;
}

////创建标定对象
//void* newBallCalibration()
//{
//    BallCalibration* bc = new BallCalibration();
//    return bc;
//}

/// <summary>
/// 更新标定对象
/// </summary>
/// <param name="ball_camera">外层vector是不同时序的球，内层vector是同时序内的不同相机的球，各时序的相机点云顺序要一致</param>
/// 输出是一个标定对象指针
void* updateCalibrationObj(vector<vector<CP>> ball_camera)
{
    //BallCalibration* bc = static_cast<BallCalibration*>(ball_calibration);
    BallCalibration* bc = new BallCalibration();
    //bc->fitBall(ball_camera[2][3]);

    int ball_num = ball_camera.size();
    int camera_num = ball_camera[0].size();
    VCP ret;
    for (int i = 0; i < ball_camera[0].size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->resize(ball_num); // 长度为3的点云
        ret.push_back(cloud);
    }

    VCP ball_clouds;
    for (int i = 0; i < ball_camera.size(); i++)
    {
        for (int j = 0; j < ball_camera[i].size(); j++)
        {
            ball_clouds.push_back(ball_camera[i][j]);
        }
    }

    #pragma omp parallel for
    for (int i = 0; i < ball_clouds.size(); i++)
    {
        auto coff = bc->fitBall(ball_clouds[i], to_string(i));
        PointT center(coff[0], coff[1], coff[2]);
        ret[i % camera_num]->points[i / camera_num] = center;
    }
    bc->ball_center = ret;
    

    //for (int i = 0; i < ball_camera.size(); i++)
    //{
    //    cout << "ball: " << i << endl;
    //    bc->addBall(ball_camera[i]);
    //}
    bc->updateCalibration();
    return bc;
}

/// <summary>
/// 合并不同相机的点云
/// </summary>
/// <param name="obj">标定对象</param>
/// <param name="clouds">不同相机的点云， 顺序和标定时一致</param>
/// <returns>返回合并的点云</returns>
CP mergeCloud(void* obj, vector<CP> clouds)
{
    BallCalibration* bc = static_cast<BallCalibration*>(obj);
    return bc->Calibration(clouds);
}

/// <summary>
/// 保存标定对象到本地
/// </summary>
/// <param name="obj">标定对象指针</param>
/// <param name="num">相机数目</param>
/// <param name="file_name">保存的文件名</param>
void saveObj(void* obj, int num, string file_name)
{
    BallCalibration* bc = static_cast<BallCalibration*>(obj);
    bc->save(num, file_name);
}

void* loadObj(int num, string file_name)
{
    BallCalibration* bc = new BallCalibration();
    bc->load(num, file_name);
    return bc;
}

//ax + by + cz + d = 0
Eigen::Vector4f fitPlaneLSM(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if (cloud->size() < 3) return Eigen::Vector4f();

    Eigen::Vector4f plane_coefficients;

    // 1. 计算质心
    Eigen::Vector3f centroid(0, 0, 0);
    for (const auto& point : cloud->points)
    {
        centroid += Eigen::Vector3f(point.x, point.y, point.z);
    }
    centroid /= cloud->size();

    // 2. 构建协方差矩阵
    Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero();
    for (const auto& point : cloud->points)
    {
        Eigen::Vector3f p(point.x, point.y, point.z);
        Eigen::Vector3f diff = p - centroid;
        covariance_matrix += diff * diff.transpose();
    }

    // 3. 特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance_matrix);
    Eigen::Vector3f normal = solver.eigenvectors().col(0); // 最小特征值对应的特征向量

    // 4. 计算平面方程 ax + by + cz + d = 0
    float d = -normal.dot(centroid);
    plane_coefficients.head<3>() = normal;
    plane_coefficients[3] = d;

    return plane_coefficients;
}


/// <summary>
/// 给定点拟合平面，至少要三个不共线的点
/// </summary>
/// <param name="cloud">选好的平面点云</param>
/// <returns>平面方程ax + by + cz + d = 0，返回abcd四个值，  输入的cloud会被换成拟合出的平面点云</returns>
Eigen::Vector4f fitPlane(CP& cloud)
{
    if (cloud->size() < 3) return Eigen::Vector4f();

    //pcl::PCA<pcl::PointXYZ> pca;
    //pca.setInputCloud(cloud);
    //Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
    //cloud->getMatrixXfMap(3, 4, 0) = eigenvectors.transpose() * cloud->getMatrixXfMap(3, 4, 0);
    
    auto plane = fitPlaneLSM(cloud);

    Eigen::Vector3f centroid(0, 0, 0);
    for (const auto& point : cloud->points)
    {
        centroid += Eigen::Vector3f(point.x, point.y, point.z);
    }
    centroid /= cloud->size();

    CP plane_sample(new CloudT);
    for (int off_x = -100; off_x < 100; off_x++)
    {
        for (int off_y = -100; off_y < 100; off_y++)
        {
            PointT p(centroid[0] + off_x, centroid[1] + off_y, 0);
            p.z = -plane[3] - plane[0] * p.x - plane[1] * p.y;
            p.z = p.z / plane[2];
            plane_sample->push_back(p);
        }
    }
    //Tool::showComparison(cloud, plane_sample);
    cloud = plane_sample;
    //Tool::showComparison(cloud);
    return plane;
}

/// <summary>
/// 计算点到直线的距离
/// </summary>
/// <param name="plane">平面方程ax + by + cz + d = 0，abcd四个值</param>
/// <param name="point">点</param>
/// <returns>返回点到直线的距离</returns>
float pointDisPlane(Eigen::Vector4f plane, PointT point)
{
    // 平面法向量 (a, b, c)
    float a = plane[0], b = plane[1], c = plane[2];
    // 平面的常数项 d
    float d = plane[3];

    // 点 P(x0, y0, z0)
    float x0 = point.x, y0 = point.y, z0 = point.z;

    // 计算距离
    return std::abs(a * x0 + b * y0 + c * z0 + d) / std::sqrt(a * a + b * b + c * c);
}