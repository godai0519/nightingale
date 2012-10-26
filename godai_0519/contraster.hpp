
//コントラスト・明度調整用callback関数
void contrast_callback(int pos, void* userdata)
{
    auto data = reinterpret_cast<std::tuple<const cv::Mat*,cv::Mat*,int*,int*>*>(userdata);

    cv::Mat* dst_image = std::get<1>(*data);
    std::get<0>(*data)->convertTo(
        *dst_image,
        dst_image->type(),
        *std::get<2>(*data) / 100.0,
        *std::get<3>(*data) - 255.0
        );

    cv::imshow("contrast_adjust",*dst_image);
    return;
}

void contraster(cv::Mat& dst, const cv::Mat& src)
{
	cv::namedWindow("contrast_adjust");
    cv::imshow("contrast_adjust", src);
    {
        int contrast = 400, bright = 0;
        std::tuple<const cv::Mat*,cv::Mat*,int*,int*> contrast_data = std::make_tuple(&src,&dst,&contrast,&bright);
        cv::createTrackbar("contrast","contrast_adjust",&contrast,800,&contrast_callback,(void*)&contrast_data);
        cv::createTrackbar("bright","contrast_adjust",&bright,510,&contrast_callback,(void*)&contrast_data);
        
        while((char)cv::waitKey(0) != 'q'); //qが入力されるまで待機
        cv::destroyWindow("contrast_adjust");
    }

	return;
}
