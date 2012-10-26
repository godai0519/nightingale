
//�Ԃ��G���A�𔲂��o���֐�
void red_area(cv::Mat& dst,const cv::Mat& hsv)
{
    //HSV�̗v�f����
    cv::Mat channels[3],h_mask,h_mask_low,h_mask_high,c_mask;
    cv::split(hsv,channels);

    cv::threshold(channels[0],h_mask_low,10,255,cv::THRESH_BINARY); //�F�����ԕt�߂��ǂ���
    cv::threshold(channels[0],h_mask_high,170,255,cv::THRESH_BINARY); //�F�����ԕt�߂��ǂ���
    cv::threshold(channels[1],c_mask,180,255,cv::THRESH_BINARY); //�ʓx(�M�p��)���������ǂ���

    cv::bitwise_not(h_mask_low,h_mask_low);
    cv::bitwise_or(h_mask_low,h_mask_high,h_mask);

    //�F�����ԕt�߂ŁC�ʓx����������(->dst)
    cv::bitwise_and(h_mask,c_mask,dst);

    //�����߂̔������������c��������
    cv::dilate(dst,dst,cv::Mat());
    return;
}

//�����G���A�𔲂��o��
void black_area(cv::Mat& dst,const cv::Mat& image)
{
    //RGB�ɕ���
    cv::Mat channels[3],r,g,b;
    cv::split(image,channels);
  
    //(�e�v�f��)�����Ȃ��Ƃ�
    cv::threshold(channels[0],r,70,255,cv::THRESH_BINARY);
    cv::threshold(channels[1],g,70,255,cv::THRESH_BINARY);
    cv::threshold(channels[2],b,70,255,cv::THRESH_BINARY);

    //(�e�v�f��)���]�����č����Ƃ�
    bitwise_not(r,r);
    bitwise_not(g,g);
    bitwise_not(b,b);

    //����(->dst)
    cv::bitwise_and(r,g,dst);
    cv::bitwise_and(b,dst,dst);

    //�����߂̔������������c��������
    cv::dilate(dst,dst,cv::Mat());
}

void contour_out(const std::string& window_name, const cv::Mat& img, const cv::Mat& red_map, const cv::Mat& black_map)
{
    std::vector<std::vector<cv::Point>> red_contours, black_contours;
    cv::findContours(red_map, red_contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    cv::findContours(black_map, black_contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    
    cv::Mat tmp = img.clone();
    cv::drawContours(tmp,   red_contours, -1, CV_RGB(0,0,255), 2);
    cv::drawContours(tmp, black_contours, -1, CV_RGB(0,0,255), 2);

    cv::imshow(window_name, tmp);
	return;
}

//�}�E�X�N���b�N�p�֐�(�N���b�N����x�ɌĂ΂�܂�)
//cv::setMouseCallback�̃h�L�������g�ɒ���
void inactive_area_input(int event, int x, int y, int flags, void* param)
{
    static short start_x,start_y,end_x,end_y; // VC10��GCC��short��2�o�C�g�ŃR���p�C�������̂����p�D
    static bool  first_call = true; //�o�͐��`�p

    if(event == CV_EVENT_LBUTTONDOWN)
    {
        //�N���b�N�J�n���̍��W���i�[
        start_x = x; start_y = y;
    }
    else if(event == CV_EVENT_LBUTTONUP)
    {
        //�h���b�O�I�����̍��W���i�[
        end_x = x; end_y = y;

        //�h���b�O�󋵂̕\��
        if(first_call) first_call = false;
        else std::cout << "\n";
        std::cout << "Drag And Droped:  (" << start_x << "," << start_y << ") -> (" << end_x << "," << end_y << ")" << std::endl;

        //main��senddata�Ƃ���param�ɗ���̂ŁC�|�C���^�ϊ�
        const auto data = reinterpret_cast<std::tuple<const cv::Mat*,cv::Mat*,cv::Mat*>*>(param);
        const cv::Mat* image = std::get<0>(*data);
        cv::Mat* red   = std::get<1>(*data);
        cv::Mat* black = std::get<2>(*data);

        //start������Cend���E���ɗ���悤�ɏC��
        if(start_x > end_x)
        {
            short tmp = start_x;
            start_x = end_x;
            end_x = tmp;
        }
        if(start_y > end_y)
        {
            short tmp = start_y;
            start_y = end_y;
            end_y = tmp;
        }

        //��ʊO�ւ̃N���b�N���C��
        start_x = std::max(static_cast<short>(0),start_x);
        start_y = std::max((short)0,start_y);
        end_x   = std::min((short)black->cols,end_x);
        end_y   = std::min((short)black->rows,end_y);

        //�C������
        std::cout << "After Correction: (" << start_x << "," << start_y << ") -> (" << end_x << "," << end_y << ")" << std::endl;

        //�I��͈͂̓h��Ԃ�
        for(int i = start_y; i < end_y; ++i)
        {
            for(int j = start_x; j < end_x; ++j)
            {
                black->at<unsigned char>(i,j) = 0;
                red->at<unsigned char>(i,j) = 0;
            }
        }

        cv::Mat temp = image->clone();
		contour_out("in-active", temp, *red, *black);
    }
    return;
}


void detector(std::vector<cv::KeyPoint>& red_dst, std::vector<cv::KeyPoint>& black_dst, const cv::Mat& src_gbr)
{
    // hsv�����
    cv::Mat hsv;
    cv::cvtColor(src_gbr, hsv, CV_BGR2HSV);

    //�ԍ��G���A
	cv::Mat red_map, black_map;
	red_area(red_map, hsv);
    black_area(black_map, src_gbr);
    
    cv::namedWindow("in-active");
    contour_out("in-active", src_gbr, red_map, black_map);

    std::cout << "Choose In Active Area" << std::endl;

    std::tuple<const cv::Mat*,cv::Mat*,cv::Mat*> senddata = std::tuple<const cv::Mat*,cv::Mat*,cv::Mat*>(&src_gbr,&red_map,&black_map); //setMouseCallback�œn�����f�[�^
    cv::setMouseCallback("in-active",&inactive_area_input,&senddata); //�}�E�X�N���b�N����ݒ�
    while((char)cv::waitKey(0) != 'q'); //q�����͂����܂őҋ@

    cv::destroyWindow("in-active");
    std::cout << "--End--\n" << std::endl;

    std::vector<std::vector<cv::Point>> red_contours, black_contours;
    cv::findContours(  red_map,   red_contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    cv::findContours(black_map, black_contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	{
		float radius; cv::Point2f center;
		for(auto it = red_contours.cbegin(); it != red_contours.cend(); ++it)
		{
			cv::minEnclosingCircle(*it, center, radius);
			red_dst.push_back(cv::KeyPoint(center, radius));
		}
		for(auto it = black_contours.cbegin(); it != black_contours.cend(); ++it)
		{
			cv::minEnclosingCircle(*it, center, radius);
			black_dst.push_back(cv::KeyPoint(center, radius));
		}
	}
	delete_overlap(red_contours, black_dst);

	return;
}

