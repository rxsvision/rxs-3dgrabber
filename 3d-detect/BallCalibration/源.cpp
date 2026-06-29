#include<Windows.h>
#include"../../czxDependence/czxTool.h"


void main()
{
	HMODULE hDLL = LoadLibrary(L"output\\BallCalibrationDLL.dll");
	typedef CP(__cdecl* MYTYPE)(void* obj, vector<CP> clouds);
	MYTYPE mergeCloud = (MYTYPE)GetProcAddress(hDLL, "mergeCloud");
	typedef void* (__cdecl* MYTYPE_)(vector<vector<CP>> ball_camera);
	MYTYPE_ updateCalibrationObj = (MYTYPE_)GetProcAddress(hDLL, "updateCalibrationObj");
	typedef void (__cdecl* MYTYPE_1)(void* obj, int num, string file_name);
	MYTYPE_1 saveObj = (MYTYPE_1)GetProcAddress(hDLL, "saveObj");
	typedef void* (__cdecl* MYTYPE_2)(int num, string file_name);
	MYTYPE_2 loadObj = (MYTYPE_2)GetProcAddress(hDLL, "loadObj");
	auto fitPlane = reinterpret_cast<Eigen::Vector4f(*)(CP&)>(GetProcAddress(hDLL, "fitPlane"));

	auto conf = czx_file::readProfile("conf.czx");
	string path = conf["path"];
	//string path = "D:\\download\\Cab_Root\\";
	auto dics = czx_file::getSubdictories(path);
	vector<vector<CP>> ball_camera;
	for (int i = 0; i < dics.size(); i++)
	{
		vector<CP> cameras;
		auto path_cloud = czx_file::pathGather(dics[i], "*.pcd");
		for (auto& p : path_cloud)
		{
			CP cloud(new CloudT);
			pcl::io::loadPCDFile(p, *cloud);
			//fitPlane(cloud);
			//Tool::show(cloud);
			cameras.push_back(cloud);
		}
		ball_camera.push_back(cameras);
	}
	//string fan_root = "D:\\download\\20241209174547-130_Grp\\";
	//string fan_root = "D:\\code\\3D\\BallCalibration\\BallCloud\\20241209142228-680_Grp\\";
	//string fan_root = "D:\\download\\交大扇叶\\";
	string fan_root = conf["fan_root"];
	auto fan_path = czx_file::pathGather(fan_root, "*.pcd");
	vector<CP> fan_clouds;
	for (auto& fp : fan_path)
	{
		CP cloud(new CloudT);
		pcl::io::loadPCDFile(fp, *cloud);
		fan_clouds.push_back(cloud);
	}
	//void* bc = updateCalibrationObj(ball_camera);
	//saveObj(bc, 4, "cal.txt");
	void* bc = loadObj(4, "cal.txt");
	CP ret = mergeCloud(bc, fan_clouds);
	pcl::io::savePCDFileBinary("merge.pcd", *ret);
	//Tool::show(ret);
	delete bc;
}