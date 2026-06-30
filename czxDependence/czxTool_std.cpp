#include"czxTool_std.h"
#include <io.h> 
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <windows.h>


string CzxTimer::path = "";


namespace czx_file
{
	vector<string> pathGather(string root, string file)
	{
		if (!root.empty() && root.back() != '\\') {
			root = root + '\\';
		}
		vector<string> path_list;
		{
			string inPath = root + file;
			__int64 handle;
			struct _finddata_t fileinfo;
			handle = _findfirst(inPath.c_str(), &fileinfo);
			if (handle == -1)
				return path_list;

			do
			{
				string szPath;
				string file_name = fileinfo.name;
				if (file_name == "." || file_name == "..") continue;
				szPath = root + file_name;
				path_list.push_back(szPath);
			} while (!_findnext(handle, &fileinfo));
		}
		return path_list;
	}

	//std::vector<std::string> getSubdirectories(const std::string& directoryPath) {
	//	std::vector<std::string> subdirectories;

	//	boost::filesystem::path directory(directoryPath); // 将输入的字符串路径转换为 Boost 路径对象

	//	if (boost::filesystem::exists(directory) && boost::filesystem::is_directory(directory)) {
	//		boost::filesystem::directory_iterator end_itr; // 迭代器结束标志
	//		for (boost::filesystem::directory_iterator itr(directory); itr != end_itr; ++itr) {
	//			if (boost::filesystem::is_directory(itr->status())) {
	//				subdirectories.push_back(itr->path().string()); // 将绝对路径字符串添加到 vector 中
	//			}
	//		}
	//	}

	//	return subdirectories;
	//}

	std::vector<double> readDoubleFromFile(const std::string& filename)
	{
		std::vector<double> result;

		// 打开文件
		std::ifstream inputFile(filename);

		// 检查文件是否成功打开
		if (!inputFile.is_open())
		{
			throw std::runtime_error(filename + "no exist");
		}

		// 逐行读取文件内容
		std::string line;
		while (std::getline(inputFile, line)) {
			std::istringstream iss(line);
			double temp;

			// 逐个读取double并添加到向量中
			while (iss >> temp) {
				result.push_back(temp);
			}
		}

		// 关闭文件
		inputFile.close();

		return result;
	}


	bool copyFile_czx(const std::string& sourcePath, const std::string& destinationPath) {
		std::ifstream sourceFile(sourcePath, std::ios::binary);
		std::ofstream destinationFile(destinationPath, std::ios::binary);

		if (sourceFile && destinationFile) {
			// 使用迭代器流将文件内容拷贝
			destinationFile << sourceFile.rdbuf();
			return true;
		}
		else {
			return false;
		}
	}

	std::string getFileName(const std::string& filePath) {
		// 查找最后一个路径分隔符的位置
		size_t pos = filePath.find_last_of("/\\");
		if (pos == std::string::npos) {
			// 没有找到路径分隔符，返回整个路径
			return filePath;
		}
		else {
			// 提取文件名
			return filePath.substr(pos + 1);
		}
	}

	vector<string> getSubdictories(string root)
	{
		vector<string> path_list = pathGather(root);
		vector<string> ret;
		for (auto& path : path_list)
		{
			if (path.find('.') == std::string::npos)
			{
				ret.push_back(path);
			}
		}
		return ret;
	}

	std::string createFolderWithCurrentDate() {
		// 获取当前时间
		auto now = std::chrono::system_clock::now();
		std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
		std::tm localTime = *std::localtime(&currentTime);

		// 构建文件夹名称
		std::ostringstream folderNameStream;
		folderNameStream << std::put_time(&localTime, "%Y-%m-%d");
		std::string folderName = folderNameStream.str();

		// 获取当前目录
		char currentPath[MAX_PATH];
		GetCurrentDirectoryA(MAX_PATH, currentPath);

		// 拼接完整路径
		std::string folderPath = std::string(currentPath) + "\\" + folderName;

		// 创建文件夹
		if (CreateDirectoryA(folderPath.c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {
			return folderPath;
		}
		else {
			std::cerr << "创建文件夹失败。" << std::endl;
			return "";
		}
	}

	void createFolder(string folder) 
	{
		if (CreateDirectoryA(folder.c_str(), NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {
			return ;
		}
	}

	
}

