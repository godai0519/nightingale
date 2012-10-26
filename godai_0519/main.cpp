#include <iostream>
#include <cmath>
#include <tuple>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/minmax_element.hpp>
#include <boost/filesystem.hpp>

#ifdef _DEBUG
    #pragma comment(lib,"opencv_core242d.lib")
    #pragma comment(lib,"opencv_imgproc242d.lib")
    #pragma comment(lib,"opencv_highgui242d.lib")
    #pragma comment(lib,"opencv_features2d242d.lib")
#else
    #pragma comment(lib,"opencv_core242.lib")
    #pragma comment(lib,"opencv_imgproc242.lib")
    #pragma comment(lib,"opencv_highgui242.lib")
    #pragma comment(lib,"opencv_features2d242.lib")
#endif

#include "constant.hpp"

//Windows.hの罠
#undef small

#include "util.hpp"
#include "detector.hpp"
#include "callback.hpp"
#include "contraster.hpp"
#include "sizer.hpp"
#include "grouper.hpp"


void unsharp(cv::Mat& dst, const cv::Mat& src, cv::Size window)
{
	const float k = 2.0;
	float kernel_data[] ={
		-k/9.0f, -k/9.0f, -k/9.0f,
		-k/9.0f, 1 + (8*k)/9.0f, -k/9.0f,
		-k/9.0f, -k/9.0f, -k/9.0f
	};

	const cv::Mat kernel(3, 3, CV_32F, kernel_data);
	cv::filter2D(src, dst, -1, kernel);

    //cv::Mat src_f, dst_f, sub_f;

    //src.convertTo(src_f, CV_32F);
    //cv::blur(src_f, dst_f, window);
    //cv::subtract(src_f, dst_f, sub_f);
    //cv::add(src_f,sub_f,dst_f);

    //dst_f.convertTo(dst, CV_8U);
    return;
}
//
//void detector(std::vector<cv::KeyPoint>& dst, const cv::Mat& src)
//{
//    cv::Mat gray_img;
//    cv::cvtColor(src, gray_img, CV_BGR2GRAY);
//    cv::normalize(gray_img, gray_img, 0, 255, cv::NORM_MINMAX);
//  
//    // SimpleBlob 検出器に基づく特徴点検出
//    // thresholdStep=20, other params=default
//    cv::SimpleBlobDetector::Params params;
//    params.thresholdStep = 20;
//    cv::SimpleBlobDetector detector(params);
//    cv::Scalar color(200,255,100);
//    detector.detect(gray_img, dst);
//
//    return;
//}


int calc_from_weight(const double weight, const double big_ratio, const double middle_ratio, const double small_ratio)
{
    const double ratio_sum = big_ratio + middle_ratio + small_ratio;

    std::vector<std::tuple<int,int,int,double>> data;
    double big, middle, small;
    for(int i=0; (big=average_weight[0]*i) < weight || std::abs(big_ratio)<0.01; ++i)
    {
        for(int j=0; big + (middle = average_weight[1]*j) < weight || std::abs(middle_ratio)<0.01; ++j)
        {
            double sum;
            int k;
            for(k=0; (sum = (big + middle + (small = average_weight[2]*k))) < weight || std::abs(small_ratio)<0.01; ++k);

            const int num = i + j + k;
            if(std::abs((double)i/num - big_ratio/ratio_sum) > 0.05) continue;
            if(std::abs((double)j/num - middle_ratio/ratio_sum) > 0.05) continue;
            if(std::abs((double)k/num - small_ratio/ratio_sum) > 0.05) continue;

            data.push_back(std::make_tuple(i, j, k, std::abs(sum-weight)));
        }
    }

    std::sort(
        data.begin(), data.end(),
        [](const std::tuple<int,int,int,double>& lhs, const std::tuple<int,int,int,double>& rhs)
        {
            return std::get<3>(lhs) < std::get<3>(rhs);
        }
    );
    
    for(auto it = data.cbegin(); it != data.cend(); ++it)
    {
        std::cout << std::get<0>(*it) << " " << std::get<1>(*it) << " " << std::get<2>(*it) << " " << std::get<3>(*it) << std::endl;
    }

    return 0;
}

std::tuple<double,double,double> get_ratio(const std::string& path)
{
    cv::Mat src = cv::imread(path.c_str(), 1);
    if(src.empty()) throw std::exception();

    cv::resize(src, src, cv::Size(1024, 768));
    unsharp(src, src, cv::Size(7,7));

	cv::Mat image;
	contraster(image, src);

	std::vector<cv::KeyPoint> red_boxes, black_boxes;
	detector(red_boxes, black_boxes, image);


    //大きいのとかを，ここで消す
	erase_small_eye(red_boxes, 3);
	erase_small_eye(black_boxes, 3);
	erase_big_eye(red_boxes, 100);
	erase_big_eye(black_boxes, 100);
    
    std::vector<std::vector<cv::KeyPoint>> first_grouped;
    face_group(first_grouped, black_boxes);
    
    std::vector<std::vector<cv::KeyPoint>> second_grouped;
    face_merge(second_grouped, first_grouped);

	std::vector<std::pair<cv::Point2f,float>> faces;
	int big, mid;
	std::tie(big, mid) = sizer(faces, src, second_grouped);

    int big_num, mid_num, small_num;
    big_num = mid_num = small_num = 0;
    for(auto it = faces.cbegin(); it != faces.cend(); ++it)
    {
        if(it->second > big) ++big_num;
        else if(it->second > mid) ++mid_num;
        else ++small_num;
    }

    std::cout << big_num << " " << mid_num << " " <<  small_num << std::endl;

    return std::make_tuple((double)big_num, (double)mid_num, (double)small_num);
}

int main(int argc, char* argv[])
{
    std::string full_path;
    if(argc > 1)
    {
        full_path.append(argv[1]);
    }
    else
    {
        boost::filesystem::path root_path("D:\\DCIM\\100CANON");
        //boost::filesystem::path root_path("S:\\DCIM\\100CANON");
        //boost::filesystem::path root_path("E:\\Repository\\24th-kosen-procon\\24th-kosen-procon");
        std::string file_from_root;
        std::cout << "File Path(from " << root_path.branch_path() << "\\IMG_****.JPG): ";
        std::cin >> file_from_root;
        file_from_root = "IMG_" + file_from_root + ".JPG";

        if(!boost::filesystem::exists(root_path/file_from_root))
        {
            std::cout << "File Not Found." << std::endl;
            std::exit(1);
        }

        full_path.append((root_path/file_from_root).string());
    }

    //いわゆるスレッド
    boost::packaged_task<std::tuple<double,double,double>> p(boost::bind(get_ratio, full_path));
    boost::unique_future<std::tuple<double,double,double>> f(p.get_future());
    boost::thread th(boost::ref(p));
    
    double weight;
    std::cout << "\nWeight: ";
    std::cin >> weight;
    std::cout << std::endl;
    
    try
    {
        const auto ratio = f.get();
        std::cout << std::endl;
        calc_from_weight(weight, std::get<0>(ratio), std::get<1>(ratio), std::get<2>(ratio));
    }
    catch(...)
    {
        double big, middle, small;
        std::cout << "Bad Ratio Found." << std::endl;
        std::cout << "Ratio(Big Middle Small): ";
        std::cin >> big >> middle >> small;
        std::cout << std::endl;
        calc_from_weight(weight, big, middle, small);
    }
	
	std::system("Pause");
	


    return 0;
}

