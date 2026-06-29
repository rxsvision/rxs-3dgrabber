#pragma once
#include <random> 
#include <algorithm>
#include <utility>
#include <vector>
#include <iostream>
#include"../../../czxDependence/czxTool.h"

using namespace std;
//template<typename T>
class Ransac3D
{
public:
    Ransac3D(CP all_data_);

    float sample_ratio = 0.5;
    int sample_num = -1;
    CP sample()
    {
        int valid_range = valid_num.size() * 0.8;
        if (valid_range < 20) valid_range = valid_num.size();

        // 创建一个从0到n的整数序列
        std::vector<int> numbers;
        for (int i = 0; i < valid_range; ++i) {
            numbers.push_back(i);
        }

        // 随机生成随机数生成器
        //std::random_device rd;
        //std::mt19937 gen(rd());
        std::mt19937 gen;
        unsigned int seed = 12345;
        gen.seed(seed);


        // 使用 random_shuffle 打乱 numbers
        std::shuffle(numbers.begin(), numbers.end(), gen);
        int sample_count = valid_range * sample_ratio;
        if (sample_count < 5) sample_count = 5;
        if (sample_count > numbers.size()) sample_count = numbers.size();
        if (sample_num > 0) sample_count = sample_num;

        select_samples = std::vector<int>(numbers.begin(), numbers.begin() + sample_count);

        CP ret(new CloudT);
        for (int i = 0; i < select_samples.size(); i++)
            ret->push_back(all_data->points[select_samples[i]]);
        sample_points = ret;
        return ret;
    }

    void run();
    CP getSample() { return sample_points; };

    virtual void fit() = NULL;
    
    //需要评估拟合结果，并更新valid_num，按照贴合计数重新排序
    virtual bool evaluate() = NULL;
    virtual void useBest() = NULL;

    //float max_score = std::numeric_limits<float>::min();
    float max_score = -1;
    int itera_num = 1000;
protected:
    vector<pair<int, int>> valid_num;//第一个数是索引，第二个数是贴合计数
    CP all_data;
    CP sample_points;
    std::vector<int> select_samples;
};



#pragma region BALL
class RansacBall :public Ransac3D
{
public:
    RansacBall(CP ad) :Ransac3D(ad) {};

    virtual void fit();
    //float pointToLineDistance(PointT);
    virtual bool evaluate();
    virtual void useBest();

    void runM();
    Eigen::VectorXf fitM();
    bool evaluateM(Eigen::VectorXf ball);

    bool end = false;

    float a, b, c, r;
    float best_a, best_b, best_c, best_r;

    float ref_r=-1;

    float threshold = 0.1;
};
#pragma endregion
