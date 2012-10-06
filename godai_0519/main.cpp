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

//Windows.hの罠
#undef small

const double PI = 6*asin(0.5);
const cv::Scalar colors[] = {CV_RGB(0,0,255),CV_RGB(0,255,0),CV_RGB(255,0,0),CV_RGB(255,255,0),CV_RGB(0,255,255),CV_RGB(255,0,255),CV_RGB(255,255,255)};
const double average_weight[3] = { 5.902, 1.35, 0.274 };
const double standard_dev[3] = {0.13265, 0.059161, 0.043863};

template<class T = float>
class line_segment{
    const cv::Point_<T> orient_vector_;
    const cv::Point_<T> fixed_point1_;
    const cv::Point_<T> fixed_point2_;
    
public:
    line_segment(const T& x1,const T& y1,const T& x2,const T& y2)
    : orient_vector_(x2-x1,y2-y1),
        fixed_point1_(x1,y1),
        fixed_point2_(x2,y2)
    {
    }
    line_segment(const cv::Point_<T>& p1,const cv::Point_<T>& p2)
    : orient_vector_(p2.x-p1.x,p2.y-p1.y),
        fixed_point1_(p1),
        fixed_point2_(p2)
    {
    }
    
    bool is_parallel(const line_segment& other,const T& tolerance_angle = PI/36) const
    {
        const double x1 = this->orient_vector_.x, y1 = this->orient_vector_.y;
        const double x2 = other.orient_vector_.x, y2 = other.orient_vector_.y;
        const double angle = acos((x1*x2 + y1*y2) / (hypot(x1,y1)*hypot(x2,y2)));

        return (-tolerance_angle <= angle && angle <= tolerance_angle);
    }
    bool is_normal(const line_segment& other,const T& tolerance_angle = PI/36) const
    {
        const double x1 = this->orient_vector_.x, y1 = this->orient_vector_.y;
        const double x2 = other.orient_vector_.x, y2 = other.orient_vector_.y;
        const double angle = acos((x1*x2 + y1*y2) / (hypot(x1,y1)*hypot(x2,y2)));
    
        const double right_angle = PI/2;
        return (right_angle-tolerance_angle <= angle && angle <= right_angle+tolerance_angle);
    }

    T distance(const cv::Point_<T>& point) //適当
    {
        const cv::Point_<T> vec_ab(fixed_point2_ - fixed_point1_);
        const cv::Point_<T> vec_ac(point - fixed_point1_);
        if(vec_ab.dot(vec_ac) < 0.0) return hypot(vec_ac.x ,vec_ac.y);

        const cv::Point_<T> vec_ba(fixed_point1_ - fixed_point2_);
        const cv::Point_<T> vec_bc(point - fixed_point2_);
        if(vec_ba.dot(vec_bc) < 0.0) return hypot(vec_bc.x, vec_bc.y);

        return std::abs(vec_ab.cross(vec_ac)) / hypot(vec_ab.x, vec_ab.y);
    }

    bool operator|| (const line_segment& other) const
    {
        return is_parallel(other);
    }
    bool operator+ (const line_segment& other) const
    {
        return is_normal(other);
    }
};


template<typename Ty>
inline double point_to_far(const cv::Point_<Ty>& point)
{
    return hypot(point.x, point.y);
}
template<typename Type>
inline double calc_distance(const cv::Point_<Type> &p1,const cv::Point_<Type> &p2)
{
    return point_to_far(p1 - p2);
}

void detector(std::vector<cv::KeyPoint>& dst, const cv::Mat& src)
{
    cv::Mat gray_img;
    cv::cvtColor(src, gray_img, CV_BGR2GRAY);
    cv::normalize(gray_img, gray_img, 0, 255, cv::NORM_MINMAX);
  
    // SimpleBlob 検出器に基づく特徴点検出
    // thresholdStep=20, other params=default
    cv::SimpleBlobDetector::Params params;
    params.thresholdStep = 10;
    cv::SimpleBlobDetector detector(params);
    cv::Scalar color(200,255,100);
    detector.detect(gray_img, dst);

    return;
}

template<typename Ty>
inline cv::Point_<Ty> get_externally_point(
    const cv::Point_<Ty>& i,
    const cv::Point_<Ty>& j
    )
{
    return cv::Point_<Ty>(2.0*j - i);
}

void face_group(
    std::vector<std::vector<cv::KeyPoint>>& dst,
    std::vector<cv::KeyPoint>& src
    )
{
    for(size_t i=0; i<src.size(); ++i)
    {
        //距離全探索，短い順にソート
        std::vector<std::pair<size_t,double>> distances;
        for(size_t j=i+1; j<src.size(); ++j) distances.push_back(std::make_pair(j,calc_distance(src[i].pt, src[j].pt)));
        std::sort(
            distances.begin(), distances.end(),
            [](const std::pair<size_t,double>& lhs, const std::pair<size_t,double>& rhs)
            {
                return lhs.second < rhs.second;
            }
        );

        //近すぎもなく遠過ぎもないところをイテレータにのこす
        auto it = distances.cbegin();
        while(it != distances.cend())
        {
            const auto sum_size = src[i].size + src[it->first].size;
            if(sum_size*1.5 < it->second && it->second < sum_size*3.5) break;
            else ++it;
        }
        if(it == distances.cend()) continue; //該当がなければスルー

        //2つの特徴点を詰める
        std::vector<cv::KeyPoint> group;
        group.push_back(src[i]);
        group.push_back(src[it->first]);
        src.erase(src.begin() + it->first);
        src.erase(src.begin() + i);

        //3つ目があるか探してみる
        const auto externally_point = get_externally_point(group.front().pt, group.back().pt);
        for(size_t j=0; j<src.size(); ++j)
        {
            const auto distance = calc_distance(externally_point, src[j].pt);
            const auto radius   = src[j].size;

            if(distance <= radius)
            {
                group.push_back(src[j]);
                src.erase(src.begin() + j);
                if(j < i) --i;

                ////3つが見つかったんで5つ
                //for(size_t k=0; k<src.size() && group.size()!=5; ++k)
                //{
                //    const auto distance_k_center = calc_distance(group[1].pt, src[k].pt);
                //    const auto rabius_k_center   = group[1].size + src[k].size;
                //    if(distance_k_center < rabius_k_center*3.5) continue;
                //    
                //    const line_segment<> k_zero_line(src[k].pt, group[0].pt);
                //    for(size_t l=k+1; l<src.size() && group.size()!=5; ++l)
                //    {
                //        const line_segment<> l_two_line(src[l].pt, group[2].pt);

                //        if(k_zero_line.is_parallel(l_two_line))
                //        {
                //            group.push_back(src[k]);
                //            group.push_back(src[l]);
                //            src.erase(src.begin() + l);
                //            src.erase(src.begin() + k);

                //            if(k <  i) --i;
                //            if(l <  i) --i;
                //        }
                //    }
                //}

                break;
            }
        }

        dst.push_back(std::move(group));
        --i;
    }

    std::cout << "Remain: " << src.size() << std::endl;

    return;
}

void face_merge(
    std::vector<std::vector<cv::KeyPoint>>& dst,
    std::vector<std::vector<cv::KeyPoint>>& src
    )
{
    while(src.size() != 0)
    {
        std::vector<cv::KeyPoint> group;
        if(src[0].size() == 3)
        {
            for(size_t i=1; i<src.size(); ++i)
            {
                if(src[i].size() == 3)
                {
                    const line_segment<> line_base(src[0].front().pt, src[0].back().pt);
                    const line_segment<> line_other(src[i].front().pt, src[i].back().pt);
                    const float distance1 = calc_distance(src[0].front().pt, src[i].front().pt);
                    const float distance2 = calc_distance(src[0].back().pt , src[i].back().pt );
                    const float averadius = (src[0].front().size + src[0].back().size + src[i].front().size + src[i].back().size) / 4.0;

                    if(
                        line_base.is_parallel(line_other) && 
                        (averadius*3.0<distance1) && (distance1<averadius*7.0) && 
                        (averadius*3.0<distance2) && (distance2<averadius*7.0)
                        )
                    {
                        group.insert(group.end(), src[0].begin(), src[0].end());
                        group.insert(group.end(), src[i].begin(), src[i].end());
                        src.erase(src.begin() + i);
                        src.erase(src.begin());
                        break;
                    }
                }
                //else if(src[i].size() == 2)
                //{
                //    const line_segment<> line_one(src[0].front().pt, src[i].front().pt);
                //    const line_segment<> line_two(src[0].back().pt , src[i].back().pt );
                //    const cv::Point2f center((src[i].front().pt + src[i].back().pt)*0.50);
                //    if(calc_distance(center, src[0][1].pt) < src[0].at(1).size && line_one.is_parallel(line_two))
                //    {
                //        group.insert(group.end(), src[0].begin(), src[0].end());
                //        group.insert(group.end(), src[i].begin(), src[i].end());                        
                //        src.erase(src.begin() + i);
                //        src.erase(src.begin());
                //        break;
                //    }

                //}
            }
        }
        else if(src[0].size() == 2)
        {
            for(size_t i=1; i<src.size(); ++i)
            {
                if(src[i].size() != 2) continue;

                const line_segment<> line_base(src[0].front().pt, src[0].back().pt);
                const line_segment<> line_other(src[i].front().pt, src[i].back().pt);
                const float distance1 = calc_distance(src[0].front().pt, src[i].front().pt);
                const float distance2 = calc_distance(src[0].back().pt , src[i].back().pt );
                const float averadius = (src[0].front().size + src[0].back().size + src[i].front().size + src[i].back().size) / 4.0;

                if(
                    line_base.is_parallel(line_other) && 
                    (averadius*3.0<distance1) && (distance1<averadius*7.0) && 
                    (averadius*3.0<distance2) && (distance2<averadius*7.0)
                    )
                {
                    group.insert(group.end(), src[0].begin(), src[0].end());
                    group.insert(group.end(), src[i].begin(), src[i].end());
                    src.erase(src.begin() + i);
                    src.erase(src.begin());
                    
                    for(size_t j=0; j<src.size(); ++j)
                    {
                        if(src[j].size() != 2) continue;
                        const line_segment<> line_third(src[j].front().pt, src[j].back().pt);

                        //並行*3
                        {
                            const float distance3 = calc_distance(group[2].pt, src[j].front().pt);
                            const float distance4 = calc_distance(group[3].pt, src[j].back().pt);

                            if(
                                line_third.is_parallel(line_other) &&
                                (averadius*3.0<distance3) && (distance3<averadius*7.0) && 
                                (averadius*3.0<distance4) && (distance4<averadius*7.0)
                                )
                            {
                                group.insert(group.end(), src[j].begin(), src[j].end());
                                src.erase(src.begin() + j);
                                if(j < i) --i;
                                break;
                            }
                        }

                        //1つ直交
                        {                          
                            const line_segment<> line_02(group[0].pt, group[2].pt);
                            const float distance3 = calc_distance(group[0].pt, src[j].front().pt);
                            const float distance4 = calc_distance(group[2].pt, src[j].back().pt);
                            
                            if(
                                line_02.is_parallel(line_third) &&
                                (averadius*3.0<distance3) && (distance3<averadius*7.0) && 
                                (averadius*3.0<distance4) && (distance4<averadius*7.0)
                                )
                            {
                                group.insert(group.end(), src[j].begin(), src[j].end());
                                src.erase(src.begin() + j);
                                if(j < i) --i;
                                break;
                            }

                            
                            const line_segment<> line_13(group[1].pt, group[3].pt);
                            const float distance5 = calc_distance(group[1].pt, src[j].front().pt);
                            const float distance6 = calc_distance(group[3].pt, src[j].back().pt);
                            
                            if(
                                line_13.is_parallel(line_third) &&
                                (averadius*3.0<distance5) && (distance5<averadius*7.0) && 
                                (averadius*3.0<distance6) && (distance6<averadius*7.0)
                                )
                            {
                                group.insert(group.end(), src[j].begin(), src[j].end());
                                src.erase(src.begin() + j);
                                if(j < i) --i;
                                break;
                            }
                        }
                    }

                    break;
                }
            }
        }

        if(group.size() == 0)
        {
            group.insert(group.end(), src[0].begin(), src[0].end());
            src.erase(src.begin());
        }

        dst.push_back(std::move(group));
    }

    return;

}


void big_small_callback(int pos, void* userdata)
{
    auto data = reinterpret_cast<std::tuple<int*, int*, cv::Mat*, std::vector<std::pair<cv::Point2f,float>>*>*>(userdata);
    cv::Mat temp = std::get<2>(*data)->clone();

    auto big = std::get<0>(*data);
    auto mid = std::get<1>(*data);

    const auto circles = std::get<3>(*data);
    for(auto it = circles->cbegin(); it != circles->cend(); ++it)
    {
        int color_num = (it->second > (float)*big) ? 0 : ((it->second > (float)*mid) ? 1 : 2);
        cv::circle(temp, it->first, it->second, colors[color_num], 3, CV_AA);
    }
    
    cv::imshow("SimpleBlob Features",temp);
    return;
}

//四隅でなく，上下左右．適当
void circle_to_four_point(std::vector<cv::Point2f>& dst, const cv::Point2f& center, const float radius)
{
    dst.push_back(cv::Point2f(center.x + radius, center.y         ));
    dst.push_back(cv::Point2f(center.x - radius, center.y         ));
    dst.push_back(cv::Point2f(center.x         , center.y + radius));
    dst.push_back(cv::Point2f(center.x         , center.y - radius));

    return;
}


int calc_from_weight(const double weight, const double big_ratio, const double middle_ratio, const double small_ratio)
{
    const double ratio_sum = big_ratio + middle_ratio + small_ratio;

    std::vector<std::tuple<int,int,int,double>> data;
    double big, middle, small;
    for(int i=0; (big=average_weight[0]*i) < weight; ++i)
    {
        for(int j=0; big + (middle = average_weight[1]*j) < weight; ++j)
        {
            double sum;
            int k;
            for(k=0; (sum = (big + middle + (small = average_weight[2]*k))) < weight; ++k);

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

std::tuple<double,double,double> get_ratio(const std::string path)
{
    //cv::Mat img = cv::imread("./black_one_normal.jpg", 1);
    cv::Mat img = cv::imread(path.c_str(), 1);
    if(img.empty()) return std::make_tuple(0.0, 0.0, 0.0);
  
    cv::resize(img,img,cv::Size(1200,900));

    std::vector<cv::KeyPoint> keypoints, red_points;
    std::vector<std::vector<cv::KeyPoint>> sized_point;
    detector(keypoints, img);
    

    std::sort(
        keypoints.begin(),
        keypoints.end(),
        [](const cv::KeyPoint& lhs,const cv::KeyPoint& rhs){ return lhs.size < rhs.size; }
    );    
    std::vector<int> distance_vec;
    distance_vec.reserve( keypoints.size()-1);
    for(size_t i = 1; i < keypoints.size(); ++i) distance_vec.push_back(keypoints[i].size - keypoints[i-1].size);
    const int position = std::distance(distance_vec.begin(), boost::first_max_element(distance_vec.begin(), distance_vec.end())) + 1;
    while(position != keypoints.size())
    {
        red_points.push_back(keypoints[position]);
        keypoints.erase(keypoints.begin() + position);
    }

    for(int i = 0; keypoints.size() != 0; ++i)
    {
        std::vector<cv::KeyPoint> same_sizes;
        for(auto it = keypoints.begin(); it != keypoints.end();)
        {
            if((int)it->size/3 == i)
            {
                same_sizes.push_back(*it);
                it = keypoints.erase(it);
            }
            else ++it;
        }

        if(!same_sizes.empty()) sized_point.push_back(std::move(same_sizes));
    }

    std::vector<std::vector<cv::KeyPoint>> first_grouped;
    std::vector<cv::KeyPoint> remain;
    std::for_each(
        sized_point.begin(), sized_point.end(),
        [&img,&first_grouped,&remain](std::vector<cv::KeyPoint>& vec)
        {
            std::sort(
                vec.begin(), vec.end(),
                [](const cv::KeyPoint& lhs, const cv::KeyPoint& rhs)
                {
                    return (lhs.pt.y == rhs.pt.y) ? (lhs.pt.x < rhs.pt.x) : (lhs.pt.y < rhs.pt.y);
                }
            );
                        
            std::vector<std::vector<cv::KeyPoint>> grouped;
            face_group(grouped, vec);
            first_grouped.insert(first_grouped.end(), grouped.begin(), grouped.end());
            remain.insert(remain.end(), vec.begin(), vec.end());

            return;
        }
    );

    
    std::vector<std::vector<cv::KeyPoint>> grouped;
    face_group(grouped, remain);
    first_grouped.insert(first_grouped.end(), grouped.begin(), grouped.end());
    
    std::cout << "Remain Sum: " << remain.size() << std::endl;
    for(auto it = remain.begin(); it!=remain.end(); ++it) {
        cv::circle(img, it->pt, it->size, cv::Scalar(0,0,0), 3, CV_AA);
    }

    std::vector<std::vector<cv::KeyPoint>> second_grouped;
    face_merge(second_grouped, first_grouped);

    std::vector<std::pair<cv::Point2f,float>> faces;
    cv::Mat tmp = img.clone();
    for(size_t i=0; i<second_grouped.size(); ++i)
    {
        std::vector<cv::Point2f> points;
        for(auto it = second_grouped[i].begin(); it != second_grouped[i].end(); ++it)
        {
            circle_to_four_point(points, it->pt, it->size);
            cv::circle(tmp, it->pt, it->size, colors[i%7], 3, CV_AA);
        }

        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(points, center, radius);

        faces.push_back(std::make_pair(std::move(center), radius));
    }

    cv::namedWindow("SimpleBlob Features", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
    cv::imshow("SimpleBlob Features", tmp);
        
    int big = 30, mid = 15;
    auto data = std::make_tuple<int*, int*, cv::Mat*, std::vector<std::pair<cv::Point2f,float>>*>(&big, &mid, &img, &faces);
    cv::createTrackbar("big", "SimpleBlob Features", &big, 1200, big_small_callback, (void*)&data);
    cv::createTrackbar("mid", "SimpleBlob Features", &mid, 1200, big_small_callback, (void*)&data);

    cv::waitKey(0);
    cv::destroyWindow("SimpleBlob Features");


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
        boost::filesystem::path root_path("S:\\DCIM\\100CANON");
        //boost::filesystem::path root_path("E:\\Repository\\24th-kosen-procon\\24th-kosen-procon");
        std::string file_from_root;
        std::cout << "File Path(from S:\\): ";
        std::cin >> file_from_root;

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
    
    const auto ratio = f.get();
    calc_from_weight(weight, std::get<0>(ratio), std::get<1>(ratio), std::get<2>(ratio));

    return 0;
}

