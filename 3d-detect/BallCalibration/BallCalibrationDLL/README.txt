/// <summary>
/// 更新标定对象
/// </summary>
/// <param name="ball_camera">外层vector是不同时序的球，内层vector是同时序内的不同相机的球，各时序的相机点云顺序要一致</param>
/// 输出是一个标定对象指针
void* updateCalibrationObj(vector<vector<CP>> ball_camera)

/// <summary>
/// 合并不同相机的点云
/// </summary>
/// <param name="obj">标定对象</param>
/// <param name="clouds">不同相机的点云， 顺序和标定时一致</param>
/// <returns>返回合并的点云</returns>
CP mergeCloud(void* obj, vector<CP> clouds)

/// <summary>
/// 保存标定对象到本地
/// </summary>
/// <param name="obj">标定对象指针</param>
/// <param name="num">相机数目</param>
/// <param name="file_name">保存的文件名</param>
void saveObj(void* obj, int num, string file_name)


void* loadObj(int num, string file_name)


/// <summary>
/// 给定点拟合平面，至少要三个不共线的点
/// </summary>
/// <param name="cloud">选好的平面点云</param>
/// <returns>平面方程ax + by + cz + d = 0，返回abcd四个值，  输入的cloud会被换成拟合出的平面点云</returns>
Eigen::Vector4f fitPlane(CP& cloud)

/// <summary>
/// 计算点到直线的距离
/// </summary>
/// <param name="plane">平面方程ax + by + cz + d = 0，abcd四个值</param>
/// <param name="point">点</param>
/// <returns>返回点到直线的距离</returns>
float pointDisPlane(Eigen::Vector4f plane, PointT point)