#include"../../czxDependence/czxTool.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>

pcl::ModelCoefficients::Ptr fitCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // 法向量估计
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setKSearch(30);
    ne.compute(*cloud_normals);

    // RANSAC 分割
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.5);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits(10, 20); // 设置半径范围
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers, *coefficients);

    return coefficients;
}

void showCylinder(CP cloud, const pcl::ModelCoefficients::Ptr& coefficients)
{
    pcl::visualization::PCLVisualizer viewer;
    viewer.addPointCloud(cloud);
    viewer.addCylinder(*coefficients,"cy");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cy");
    viewer.spin();
}

void saveCylinderParameters(const pcl::ModelCoefficients::Ptr& coefficients, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

    // 写入圆柱参数
    for (const auto& value : coefficients->values) {
        file << value << " ";
    }

    file.close();
    std::cout << "圆柱参数已保存到: " << filename << std::endl;
}

bool loadCylinderParameters(pcl::ModelCoefficients::Ptr& coefficients, const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }

    coefficients->values.clear();
    std::string line;

    // 读取参数
    if (std::getline(file, line)) {
        std::istringstream iss(line);
        float value;
        while (iss >> value) {
            coefficients->values.push_back(value);
        }
    }

    file.close();
    std::cout << "圆柱参数已从: " << filename << " 读取" << std::endl;
    return true;
}

CP noiseFilter(CP cloud)
{
    CP cloud_filtered(new CloudT);
    // 创建半径滤波器
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.06); // 搜索半径
    outrem.setMinNeighborsInRadius(13); // 半径内最少需要10个邻域点
    outrem.filter(*cloud_filtered);
    return cloud_filtered; 
}

//前三个表示轴心上的点，后面三个是轴心方向，最后一个是半径
//x0,y0,z0,a,b,c,r
//x是角度，y是点到与轴垂直面的距离，z是点到轴线的距离
CP cylinderFlatten(pcl::ModelCoefficients::Ptr& coefficients, CP cloud)
{
    auto coeff = coefficients->values;
    Eigen::Vector3f direction(coeff[3], coeff[4], coeff[5]);
    direction.normalize();
    PointT center(coeff[0], coeff[1], coeff[2]);
    float R = coeff[6];

    //参考面方程 ax+by+cz=0
    Eigen::Vector3f ref_x(1, 0, 0);
    ref_x = ref_x.cross(direction);


    CP ret(new CloudT);
    for (auto& p_cloud : *cloud)
    {
        PointT p;
        Eigen::Vector3f p2c = p_cloud.getVector3fMap() - center.getVector3fMap();
        float p2c_dis = p2c.norm();

        p.y = p2c.dot(direction); //方向单位化后不必除

        p.z = sqrt(p2c_dis * p2c_dis - p.y * p.y);
        //p.z = 0;

        Eigen::Vector3f p2cProjDirection = p.y * direction;
        Eigen::Vector3f p2cProjPlane = p2c - p2cProjDirection;
        p2cProjPlane.normalize();

        double x = ref_x.dot(p2cProjPlane);
        p.x = acos(x) * R;//0-pai 映射到0-paiR
        if (ref_x.cross(p2cProjPlane)[0] / direction[0] < 0)
            p.x = -p.x;
        ret->push_back(p);
    }
    return ret;
}

vector<CP> splitCloudByX(CP cloud, int sub_num)
{
    pcl::PointXYZ min_point, max_point;
    // 使用 getMinMax3D 函数
    pcl::getMinMax3D(*cloud, min_point, max_point);

    float sub_size = (max_point.x - min_point.x) / sub_num +  1e-7;//加上1e-7避免边界点云超出数组

    vector<CP> ret;
    for (int i = 0; i < sub_num; i++)
    {
        CP sub_cloud(new CloudT);
        ret.push_back(sub_cloud);
    }

    for (auto& p : *cloud)
    {
        float bias = p.x - min_point.x;
        ret[bias / sub_size]->push_back(p);
    }
    return ret;
}

CP getMaxIntervalPointByY(CP cloud)
{
    if (cloud->size() < 10) return CP(new CloudT());
    sort(cloud->begin(), cloud->end(), [](PointT a, PointT b) {return a.y > b.y; });
    float max_interval = -1;
    CP ret(new CloudT());
    
    for (int i = 0; i < cloud->size() - 1; i++)
    {
        float interval = cloud->points[i].y - cloud->points[i + 1].y;
        if (interval > max_interval)
        {
            max_interval = interval;

            ret->clear();
            ret->push_back(cloud->points[i]);
            ret->push_back(cloud->points[i+1]);
        }
    }
    return ret;
}


float computDis(CP cloud)
{
    auto sub_clouds = splitCloudByX(cloud, 1000);
    CP bound(new CloudT);
    float dis_all = 0;
    float dis_num = 0;
    for (auto& sub_cloud : sub_clouds)
    {
        CP bound_1 = getMaxIntervalPointByY(sub_cloud);
        *bound += *bound_1;

        //-37, -13
        if (bound_1->size() != 2) continue;
        if (bound_1->points[0].x > -37 && bound_1->points[0].x < -13)
        {
            dis_all += bound_1->points[0].y - bound_1->points[1].y;
            dis_num++;
        }

    }
    Tool::showComparison(cloud, bound, 1, 5);
    if (dis_num == 0) return -1;
    return dis_all / dis_num;
}
//CP generateCylinderPointCloud(const pcl::ModelCoefficients::Ptr& coefficients,
//    float height)
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new CloudT);
//    // 获取圆柱的参数
//    float x0 = coefficients->values[0];  // 圆柱轴心点的 x 坐标
//    float y0 = coefficients->values[1];  // 圆柱轴心点的 y 坐标
//    float z0 = coefficients->values[2];  // 圆柱轴心点的 z 坐标
//    float dx = coefficients->values[3];  // 圆柱轴的方向向量的 x 分量
//    float dy = coefficients->values[4];  // 圆柱轴的方向向量的 y 分量
//    float dz = coefficients->values[5];  // 圆柱轴的方向向量的 z 分量
//    float radius = coefficients->values[6];  // 圆柱的半径
//
//    // 圆柱的高度范围
//    float z_min = z0 - height / 2.0f;
//    float z_max = z0 + height / 2.0f;
//
//    // 点云的角度分布数量和高度分布数量
//    int num_points_angle = 360; // 在圆周上生成多少个点
//    int num_points_height = 10; // 高度方向上生成多少个点
//
//    // 生成点云
//    for (int i = 0; i < num_points_height; ++i) {
//        float z = z_min + i * (z_max - z_min) / (num_points_height - 1);  // 高度方向上的坐标
//
//        // 对于每个高度，沿着圆周生成点
//        for (int j = 0; j < num_points_angle; ++j) {
//            // 计算角度
//            float theta = j * 2 * M_PI / num_points_angle;
//
//            // 计算圆周上的点
//            float x = x0 + radius * cos(theta);
//            float y = y0 + radius * sin(theta);
//
//            // 计算当前点在圆柱轴上的投影，并计算旋转
//            // 将点沿轴向量旋转到合适的位置
//            pcl::PointXYZ point;
//            point.x = x;
//            point.y = y;
//            point.z = z;
//
//            // 将点添加到点云中
//            cloud->points.push_back(point);
//        }
//    }
//
//    cloud->width = cloud->points.size();
//    cloud->height = 1;
//
//    std::cout << "Generated " << cloud->points.size() << " points for the cylinder." << std::endl;
//    return cloud;
//}
void main()
{
	CP cloud(new CloudT);

	pcl::io::loadPCDFile("水管.pcd", *cloud);

    cloud->getMatrixXfMap(3, 4, 0).colwise() -= cloud->getMatrixXfMap(3, 4, 0).rowwise().mean();

    //auto coeff = fitCylinder(cloud);
    //saveCylinderParameters(coeff, "cylinderCoeff.txt");
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
    loadCylinderParameters(coeff, "cylinderCoeff.txt");
    showCylinder(cloud, coeff);
    cout << "R: " << coeff->values[6] << endl;
    CP flattened = cylinderFlatten(coeff, cloud);
    //pcl::io::savePCDFileBinary("flat.pcd", *flattened);
    pcl::io::loadPCDFile("flat.pcd", *flattened);
    CP filtered = noiseFilter(flattened);

    float dis = computDis(filtered);
    cout << "间隙: " << dis << endl;

    Tool::showComparison(flattened, filtered);
    //auto fit_cloud = generateCylinderPointCloud(coeff, 10);
    //Tool::showComparison(cloud, fit_cloud);

}