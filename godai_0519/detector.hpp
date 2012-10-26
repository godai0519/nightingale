
//赤いエリアを抜き出す関数
void red_area(cv::Mat& dst,const cv::Mat& hsv)
{
    //HSVの要素分解
    cv::Mat channels[3],h_mask,h_mask_low,h_mask_high,c_mask;
    cv::split(hsv,channels);

    cv::threshold(channels[0],h_mask_low,10,255,cv::THRESH_BINARY); //色相が赤付近かどうか
    cv::threshold(channels[0],h_mask_high,170,255,cv::THRESH_BINARY); //色相が赤付近かどうか
    cv::threshold(channels[1],c_mask,180,255,cv::THRESH_BINARY); //彩度(信用性)が高いかどうか

    cv::bitwise_not(h_mask_low,h_mask_low);
    cv::bitwise_or(h_mask_low,h_mask_high,h_mask);

    //色相が赤付近で，彩度が高い部分(->dst)
    cv::bitwise_and(h_mask,c_mask,dst);

    //厳しめの判定をしたから膨張させる
    cv::dilate(dst,dst,cv::Mat());
    return;
}

//黒いエリアを抜き出す
void black_area(cv::Mat& dst,const cv::Mat& image)
{
    //RGBに分割
    cv::Mat channels[3],r,g,b;
    cv::split(image,channels);
  
    //(各要素で)黒くないとこ
    cv::threshold(channels[0],r,70,255,cv::THRESH_BINARY);
    cv::threshold(channels[1],g,70,255,cv::THRESH_BINARY);
    cv::threshold(channels[2],b,70,255,cv::THRESH_BINARY);

    //(各要素で)反転させて黒いとこ
    bitwise_not(r,r);
    bitwise_not(g,g);
    bitwise_not(b,b);

    //合体(->dst)
    cv::bitwise_and(r,g,dst);
    cv::bitwise_and(b,dst,dst);

    //厳しめの判定をしたから膨張させる
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

//マウスクリック用関数(クリックする度に呼ばれます)
//cv::setMouseCallbackのドキュメントに注目
void inactive_area_input(int event, int x, int y, int flags, void* param)
{
    static short start_x,start_y,end_x,end_y; // VC10もGCCもshortが2バイトでコンパイルされるのを悪用．
    static bool  first_call = true; //出力整形用

    if(event == CV_EVENT_LBUTTONDOWN)
    {
        //クリック開始時の座標を格納
        start_x = x; start_y = y;
    }
    else if(event == CV_EVENT_LBUTTONUP)
    {
        //ドラッグ終了時の座標を格納
        end_x = x; end_y = y;

        //ドラッグ状況の表示
        if(first_call) first_call = false;
        else std::cout << "\n";
        std::cout << "Drag And Droped:  (" << start_x << "," << start_y << ") -> (" << end_x << "," << end_y << ")" << std::endl;

        //mainのsenddataとやらはparamに来るので，ポインタ変換
        const auto data = reinterpret_cast<std::tuple<const cv::Mat*,cv::Mat*,cv::Mat*>*>(param);
        const cv::Mat* image = std::get<0>(*data);
        cv::Mat* red   = std::get<1>(*data);
        cv::Mat* black = std::get<2>(*data);

        //startが左上，endが右下に来るように修正
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

        //画面外へのクリックを修正
        start_x = std::max(static_cast<short>(0),start_x);
        start_y = std::max((short)0,start_y);
        end_x   = std::min((short)black->cols,end_x);
        end_y   = std::min((short)black->rows,end_y);

        //修正結果
        std::cout << "After Correction: (" << start_x << "," << start_y << ") -> (" << end_x << "," << end_y << ")" << std::endl;

        //選択範囲の塗りつぶし
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
    // hsv作って
    cv::Mat hsv;
    cv::cvtColor(src_gbr, hsv, CV_BGR2HSV);

    //赤黒エリア
	cv::Mat red_map, black_map;
	red_area(red_map, hsv);
    black_area(black_map, src_gbr);
    
    cv::namedWindow("in-active");
    contour_out("in-active", src_gbr, red_map, black_map);

    std::cout << "Choose In Active Area" << std::endl;

    std::tuple<const cv::Mat*,cv::Mat*,cv::Mat*> senddata = std::tuple<const cv::Mat*,cv::Mat*,cv::Mat*>(&src_gbr,&red_map,&black_map); //setMouseCallbackで渡されるデータ
    cv::setMouseCallback("in-active",&inactive_area_input,&senddata); //マウスクリック動作設定
    while((char)cv::waitKey(0) != 'q'); //qが入力されるまで待機

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

