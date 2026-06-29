#include"../czxDependence/czxTool_std.h"
#include<Windows.h>

void main()
{
	auto conf = czx_file::readProfile("conf.czx");
	//string big_root = conf
	auto sub_dic = czx_file::getSubdictories(conf["root"]);
	int board_num = stoi(conf["board_num"]);
	int cur_board = 0;
	cout << conf["root"] << endl;
	cout << sub_dic.size() << endl;
	string save_root = conf["file_name"].substr(0, conf["file_name"].size() - 4);
	//string save_root = conf["file_name"].substr(1, 3);
	CreateDirectoryA(save_root.c_str(), NULL);
	for (int i = 0; i < board_num; i++)
	{

		CreateDirectoryA((save_root+"/"+to_string(i)).c_str(), NULL);
	}

	//auto files = czx_file::pathGather(conf["root"], conf["file_name"]);
	//auto sub_dic = czx_file::getSubdictories(conf["root"]);

	for (int i=0;i< sub_dic.size();i++)
	{
		string path = sub_dic[i] + "/" + conf["file_name"];
		//auto path = files[i];
		cout << path << endl;
		// 创建输入文件流，以二进制模式打开源文件
		std::ifstream src(path, std::ios::binary);
		// 创建输出文件流，以二进制模式打开目标文件
		//std::ofstream dst(save_root+"/"+to_string(i%board_num)+"/"+to_string(i/board_num)+".pcd", std::ios::binary);
		std::ofstream dst(save_root+"/"+to_string(i%board_num)+"/"+to_string(i/board_num)+".bmp", std::ios::binary);

		// 使用streambuf迭代器将输入文件流的内容复制到输出文件流
		dst << src.rdbuf();

		// 关闭文件流
		src.close();
		dst.close();
	}
	cin >> cur_board;
}