//
// 「重さと大体の比率で近似値出るんじゃね」理論展開中．
// main.cppと合体させることも検討．
//

#include <iostream>
#include <boost/format.hpp>
#include <boost/foreach.hpp>

const double average_weight[3] = { 5.902, 1.35, 0.274 };
const double standard_dev[3] = {0.13265, 0.059161, 0.043863};

const double dev = 0.078558;

int main()
{
	//全体の重さ 大サイコロの比率 中サイコロの比率 小サイコロの比率
	//と入力を待つ．

    double weight, big, middle, small;
    double ratio[3];
    std::cin >> weight;
    std::cin >> ratio[0] >> ratio[1] >> ratio[2];
    const double ratio_sum = ratio[0] + ratio[1] + ratio[2];

    std::vector<std::tuple<int,int,int>> data;
    for(int i=0; (big=average_weight[0]*i) < weight; ++i)
    {
        for(int j=0; big + (middle = average_weight[1]*j) < weight; ++j)
        {
            double sum;
            int k;
            for(k=0; (sum = (big + middle + (small = average_weight[2]*k))) < weight; ++k);

            const int num = i + j + k;
            if(std::abs((double)i/num - ratio[0]/ratio_sum) > 0.05) continue;
            if(std::abs((double)j/num - ratio[1]/ratio_sum) > 0.05) continue;
            if(std::abs((double)k/num - ratio[2]/ratio_sum) > 0.05) continue;

            data.push_back(std::make_tuple(i,j,k));
        }
    }
    
    for(auto it = data.cbegin(); it != data.cend(); ++it)
    {
        std::cout << std::get<0>(*it) << " " << std::get<1>(*it) << " " << std::get<2>(*it) << std::endl;
    }

    return 0;
}

