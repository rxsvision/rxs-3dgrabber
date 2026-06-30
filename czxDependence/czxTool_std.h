//#pragma once
#ifndef CZXTOOLSTD_HPP
#define CZXTOOLSTD_HPP
#include<map>
#include<string>
#include<fstream>
#include <mutex>
#include<iostream>
#include <time.h>
#include <unordered_map>
#include <chrono>
#include<vector>
#include<unordered_map>
#include <sstream>
#include <direct.h> // 包含 _mkdir 函数
#include <pcl/console/time.h>
#include <iomanip>


//#include <ccLog.h>
//#include <QTextStream>
//#include <QBuffer>
//#include <QDebug>
using namespace std;


using CONFTYPE = std::map<string, string>;


namespace czx_file 
{
	//root是目录路径,file是文件名,记得最后带\ 
	vector<string> pathGather(string root, string file = "*");
	std::vector<double> readDoubleFromFile(const std::string& filename);

	bool copyFile_czx(const std::string& sourcePath, const std::string& destinationPath);

	std::string getFileName(const std::string& filePath);

	vector<string> getSubdictories(string root);

	std::string createFolderWithCurrentDate();

	void createFolder(string folder);

	static CONFTYPE readProfile(string path)
	{
		CONFTYPE map;

		std::ifstream configFile(path); // 打开参数文件
		if (!configFile.is_open())
		{
			cout << "没有读取到配置文件" << path << std::endl;
			throw runtime_error("没有读取到配置文件" + path);
			return map;
		}
		std::string line;
		while (std::getline(configFile, line)) {
			// 使用适当的方法解析每一行，这里假设每行的格式为 "属性名=属性值"
			size_t delimiterPos = line.find('=');
			if (delimiterPos != std::string::npos) {
				std::string key = line.substr(0, delimiterPos);
				std::string value = line.substr(delimiterPos + 1);
				map[key] = value;
			}
		}

		configFile.close(); // 关闭文件
		if (map.size() == 0)
			cout << "没有读取到配置文件:" << path << "\n";
		return map;
	}

	static void saveProfile(string path, CONFTYPE map)
	{
		std::ofstream outfile(path);

		if (outfile.is_open()) {
			// 遍历字典，将每个键值对写入文件
			for (const auto& pair : map) {
				outfile << pair.first << "=" << pair.second << "\n";
			}
			outfile.close();  // 关闭文件
			std::cout << "写入成功" << std::endl;
		}
		else {
			std::cout << "无法打开文件" << std::endl;
		}
	}

	//std::vector<std::string> getSubdirectories(const std::string& directoryPath);

	static std::string getCurrentTimestamp() {
		// 获取当前时间点
		auto now = std::chrono::system_clock::now();

		// 转换为时间_t
		std::time_t time = std::chrono::system_clock::to_time_t(now);

		// 将时间转换为字符串
		std::ostringstream oss;
		oss << std::put_time(std::localtime(&time), "%Y-%m-%d-%H-%M-%S");

		return oss.str();
	}

	static std::string createPathByTime()
	{
		string time = getCurrentTimestamp();
		string root = "CZX_DEBUG_PATH";
		_mkdir(root.c_str());
		string path = root + "/" + time;
		_mkdir(path.c_str());
		return path;
	}
};

class CzxLog {
public:
	CzxLog(string path = "")
	{
		if (!path.empty())
		{
			fileStream.open(path, std::ios::app);

			// 检查文件是否成功打开
			if (fileStream.is_open()) {
			}
		}
	}

	~CzxLog() {
		fileStream.close();
	}

	template <typename T>
	CzxLog& operator<<(const T& value) {
		//std::stringstream stream;
		//stream << value;
		//printf("%s", stream.str());
		//ccLog::Print(QString::fromStdString(stream.str()));

		//控制台应用
		cout << value;

		if (fileStream.is_open())
			fileStream << value;
		return *this;
	}

	// 添加换行符并刷新缓冲区
	CzxLog& operator<<(std::ostream& (*manip)(std::ostream&)) {
		//if (manip != static_cast<std::ostream & (*)(std::ostream&)>(std::endl))
		//{
		//	ccLog::Print(QStringLiteral("未实现流的指针"));
		//}
		//else {
		//	ccLog::Print("\n");
		//}


		//控制台窗口
		manip(cout);

		if (fileStream.is_open()) {
			manip(fileStream);
		}
		return *this;
	}

private:
	ofstream fileStream;
};

class CzxTimer {
public:
	CzxTimer(string me) :log(path)
	{
		// 在构造函数中获取初始时间点
		start = std::chrono::system_clock::now();
		this->method = me;
	}

	~CzxTimer() {
		// 在析构函数中获取当前时间点
		auto ends = std::chrono::system_clock::now();

		std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(ends - start);

		//if (duration.count() < 10000) return;
		// 打印时间差
		log << "执行" << method << "经过的时间: " << duration.count() << " 毫秒" << std::endl;
	}

	static string path;

private:
	std::chrono::system_clock::time_point start;
	string method;
	CzxLog log;
};

#endif