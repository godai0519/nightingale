

//�����Ȗڂ�I������Ƃ���callback�֐�
void disenable_rect_input(int event, int x, int y, int flags, void* param)
{
    static short down_x = -1, down_y = -1;
    if(event == CV_EVENT_LBUTTONDOWN)
    {
        //�N���b�N�J�n���̍��W���i�[
        down_x = x; down_y = y;
    }
    else if(event == CV_EVENT_LBUTTONUP)
    {
        if(down_x == (short)x && down_y == (short)y)
        {
            std::cout << "Clicked: (" << x << "," << y << ")" << std::endl;

            const cv::Point2f click_point(down_x,down_y);
            auto contours = static_cast<std::tuple<cv::Mat*,std::vector<cv::RotatedRect>*,std::vector<cv::RotatedRect>*>*>(param);
            auto image = std::get<0>(*contours);
            auto red_contours = std::get<1>(*contours);
            auto black_contours = std::get<2>(*contours);

            //�ԁ����ƒT�����C�ŏ��Ɍ����������̂��폜
            bool match = false;
            for(auto it = red_contours->begin(); it != red_contours->end(); ++it)
            {
                if(cv::pointPolygonTest(rect_to_points(*it),click_point,false) >= 0.0)
                {
                    match = true;
                    red_contours->erase(it);
                    break;
                }
            }
            if(!match)
            {
                for(auto it = black_contours->begin(); it != black_contours->end(); ++it)
                {
                    if(cv::pointPolygonTest(rect_to_points(*it),click_point,false) >= 0.0)
                    {
                        black_contours->erase(it);
                        break;
                    }
                }
            }

            //�㏑���`��
            cv::Mat temp = image->clone();
            for(auto it = red_contours->cbegin(), end = red_contours->cend(); it != end; ++it)
            {
                cv::Point2f vtx[4];
                it->points(vtx);
                for(int i=0; i<4; ++i) cv::line(temp, vtx[i], vtx[i<3?i+1:0], cv::Scalar(200,100,100), 2, CV_AA);
            }
            for(auto it = black_contours->cbegin(), end = black_contours->cend(); it != end; ++it)
            {
                cv::Point2f vtx[4];
                it->points(vtx);
                for(int i=0; i<4; ++i) cv::line(temp, vtx[i], vtx[i<3?i+1:0], cv::Scalar(100,100,200), 2, CV_AA);
            }
            cv::imshow("rect_temp",temp);
        }
    }
    return;
}

