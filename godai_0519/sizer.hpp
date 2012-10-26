
void big_small_callback(int pos, void* userdata)
{
    auto data = reinterpret_cast<std::tuple<int*, int*, const cv::Mat*, std::vector<std::pair<cv::Point2f,float>>*>*>(userdata);
    cv::Mat temp = std::get<2>(*data)->clone();

    auto big = std::get<0>(*data);
    auto mid = std::get<1>(*data);

    const auto circles = std::get<3>(*data);
    for(auto it = circles->cbegin(); it != circles->cend(); ++it)
    {
        int color_num = (it->second > (float)*big) ? 0 : ((it->second > (float)*mid) ? 1 : 2);
        cv::circle(temp, it->first, it->second, colors[color_num], 3, CV_AA);
    }
    
    cv::imshow("size",temp);
    return;
}

std::pair<int,int> sizer(std::vector<std::pair<cv::Point2f,float>>& dst_faces, const cv::Mat& src, const std::vector<std::vector<cv::KeyPoint>>& group)
{
    cv::Mat tmp = src.clone();
    for(size_t i=0; i<group.size(); ++i)
    {
        std::vector<cv::Point2f> points;
        for(auto it = group[i].begin(); it != group[i].end(); ++it)
        {
            circle_to_four_point(points, it->pt, it->size);
            cv::circle(tmp, it->pt, it->size, colors[i%7], 3, CV_AA);
        }

        cv::Point2f center; float radius;
        cv::minEnclosingCircle(points, center, radius);

        dst_faces.push_back(std::make_pair(std::move(center), radius));
    }

	erase_big_faces(dst_faces);
	delete_overlap(dst_faces);

    cv::namedWindow("size", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
    cv::imshow("size", tmp);
        
    int big = 30, mid = 15;
    auto data = std::make_tuple<int*, int*, const cv::Mat*, std::vector<std::pair<cv::Point2f,float>>*>(&big, &mid, &src, &dst_faces);
    cv::createTrackbar("big", "size", &big, 1024, big_small_callback, (void*)&data);
    cv::createTrackbar("mid", "size", &mid, 1024, big_small_callback, (void*)&data);

    cv::waitKey(0);
    cv::destroyWindow("size");

	return std::make_pair(big, mid);
}
