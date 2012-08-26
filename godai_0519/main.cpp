//
// black_one_normal.jpgでのみ上手くグループ化でき，その後の数え上げ処理は未定
// 基本的にCUIとの対話形式で処理を行う
//

#include <iostream>
#include <tuple>
#include <opencv2/opencv.hpp>

//ライブラリのリンク宣言だけど気にしなくてもいいと思われ
#ifdef _DEBUG
    #pragma comment(lib,"opencv_core240d.lib")
    #pragma comment(lib,"opencv_imgproc240d.lib")
    #pragma comment(lib,"opencv_highgui240d.lib")
#else
    #pragma comment(lib,"opencv_core240.lib")
    #pragma comment(lib,"opencv_imgproc240.lib")
    #pragma comment(lib,"opencv_highgui240.lib")
#endif

static const double PI = 6*asin(0.5);

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

  //cv::Mat channels[3],v_mask;
  //cv::split(image,channels);

  //cv::threshold(channels[2],v_mask,100,255,cv::THRESH_BINARY);
  //bitwise_not(v_mask,v_mask);
  //dst = v_mask;

  //厳しめの判定をしたから膨張させる
  cv::dilate(dst,dst,cv::Mat());
}

std::vector<cv::Point2f> rect_to_points(const cv::RotatedRect& rect)
{
  cv::Point2f points[4];
  rect.points(points);

  std::vector<cv::Point2f> result;
  for(int i=0; i<4; ++i) result.push_back(points[i]);
  return result;
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
    const auto data = reinterpret_cast<std::tuple<cv::Mat*,cv::Mat*,cv::Mat*>*>(param);
    cv::Mat* image = std::get<0>(*data);
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
    end_x   = std::min((short)red->cols,end_x);
    end_y   = std::min((short)red->rows,end_y);

    //修正結果
    std::cout << "After Correction: (" << start_x << "," << start_y << ") -> (" << end_x << "," << end_y << ")" << std::endl;

    //選択範囲の塗りつぶし
    for(int i = start_y; i < end_y; ++i)
    {
      for(int j = start_x; j < end_x; ++j)
      {
        red->at<unsigned char>(i,j) = 0;
        black->at<unsigned char>(i,j) = 0;
      }
    }

    //赤・黒合成
    cv::Mat out;
    bitwise_or(*red,*black,out); //赤か黒のところ(->out)
    
    //輪郭情報の削除と再探索
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(out,contours,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);

    //上書き表示
    cv::Mat temp = image->clone();
    cv::drawContours(temp,contours,-1,CV_RGB(0,0,255),2);
    cv::imshow("temp",temp);
  }
  return;
}

void disenable_rect_input(int event, int x, int y, int flags, void* param)
{
  static short down_x = -1, down_y = -1;
  if(event == CV_EVENT_LBUTTONDOWN)
  {
    //クリック開始時の座標を格納
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

      //赤→黒と探索し，最初に見つかったものを削除
      bool match = false;
      for(auto it = red_contours->cbegin(); it != red_contours->cend(); ++it)
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
        for(auto it = black_contours->cbegin(); it != black_contours->cend(); ++it)
        {
          if(cv::pointPolygonTest(rect_to_points(*it),click_point,false) >= 0.0)
          {
            black_contours->erase(it);
            break;
          }
        }
      }

      //上書き描画
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

std::vector<cv::RotatedRect>& erase_big_rect(std::vector<cv::RotatedRect>& rects)
{
  for(auto it=rects.cbegin(); it != rects.cend();)
  {
    if(it->size.width>50 || it->size.height>50) it = rects.erase(it);
    else ++it;
  }
  return rects;
}
std::vector<cv::RotatedRect>& erase_small_rect(std::vector<cv::RotatedRect>& rects)
{
  for(auto it=rects.cbegin(); it != rects.cend();)
  {
    if(it->size.width<5 || it->size.height<5) it = rects.erase(it);
    else ++it;
  }
  return rects;
}

void erase_overlap(std::vector<cv::RotatedRect>& red_sources,std::vector<cv::RotatedRect>& black_sources)
{
  for(size_t i=0; i<black_sources.size(); ++i)
  {
    const std::vector<cv::Point2f> contour = rect_to_points(black_sources[i]);
    for(size_t j=i+1; j<black_sources.size();) //Not Increment
    {
      if(cv::pointPolygonTest(contour,black_sources[j].center,false) >= 0.0) black_sources.erase(black_sources.begin() + j);
      else ++j;
    }
    for(size_t j=0; j<red_sources.size();) //Not Increment
    {
      if(cv::pointPolygonTest(contour,red_sources[j].center,false) >= 0.0) red_sources.erase(red_sources.begin() + j);
      else ++j;
    }
  }
  
  return;
}

template<class T = float>
class line_segment{
  const cv::Point_<T> orient_vector_;
  const cv::Point_<T> fixed_point_;

public:
  line_segment(const T& x1,const T& y1,const T& x2,const T& y2)
    : orient_vector_(x2-x1,y2-y1),
      fixed_point_(x1,y1)
  {
  }
  line_segment(const cv::Point_<T>& p1,const cv::Point_<T>& p2)
    : orient_vector_(p2.x-p1.x,p2.y-p1.y),
      fixed_point_(p1)
  {
  }
  
  bool is_parallel(const line_segment& other,const T& tolerance_angle = PI/18) const
  {
    const double x1 = this->orient_vector_.x, y1 = this->orient_vector_.y;
    const double x2 = other.orient_vector_.x, y2 = other.orient_vector_.y;
    const double angle = acos((x1*x2 + y1*y2) / (hypot(x1,y1)*hypot(x2,y2)));

    return (-tolerance_angle <= angle && angle <= tolerance_angle);
  }
  bool is_normal(const line_segment& other,const T& tolerance_angle = PI/18) const
  {
    const double x1 = this->orient_vector_.x, y1 = this->orient_vector_.y;
    const double x2 = other.orient_vector_.x, y2 = other.orient_vector_.y;
    const double angle = acos((x1*x2 + y1*y2) / (hypot(x1,y1)*hypot(x2,y2)));
    
    const double right_angle = PI/2;
    return (right_angle-tolerance_angle <= angle && angle <= right_angle+tolerance_angle);
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

template<typename Type>
inline double calc_distance(const cv::Point_<Type> &p1,const cv::Point_<Type> &p2)
{
  return std::sqrt(
    std::pow(static_cast<double>(std::abs(p1.x-p2.x)),2) + std::pow(static_cast<double>(std::abs(p1.y-p2.y)),2)
    );
}

void grouping(
  std::vector<std::vector<cv::RotatedRect>>& groups,
  std::vector<cv::RotatedRect>& sources
  )
{
  for(size_t i=0; i<sources.size(); ++i)
  {
    for(size_t j=i+1; j<sources.size(); ++j)
    {
      const float angle_distance = std::fabs(sources[i].angle - sources[i].angle);
      const double center_distance = calc_distance(sources[i].center, sources[j].center);
      const double average_diameter = (sources[i].size.width + sources[i].size.height + sources[j].size.width + sources[j].size.height) / 4.0; //要検討

      if((center_distance > average_diameter*2.0) || (center_distance < average_diameter*3.0)) //要検討
      {
        //外分点を漁る
        const cv::Point2f ext_i(2.0*sources[i].center - sources[j].center);
        const cv::Point2f ext_j(2.0*sources[j].center - sources[i].center);

        //とりま全探索
        //<PointPolygonTest,n番目,ext_i(true) or ext_j(false)>
        std::vector<std::tuple<double,int,bool>> matching_list;
        for(size_t k=0; k<sources.size(); ++k)
        {
          if(k==i || k==j) continue;
          
          const std::vector<cv::Point2f> contours(rect_to_points(sources[k]));
          {
            const double measure = cv::pointPolygonTest(contours,ext_i,true);
            if(measure >= 0.0) matching_list.push_back(std::make_tuple(measure,k,true));
          }
          {
            const double measure = cv::pointPolygonTest(contours,ext_j,true);
            if(measure >= 0.0) matching_list.push_back(std::make_tuple(measure,k,false));
          }
        }
        
        if(!matching_list.empty())
        {
          //1つ以上マッチングしていれば
          //ソートと最大マッチング取り出し(最初の一個だけ探索してもおｋだけど面倒くさいからSTLで)
          std::partial_sort(matching_list.begin(),matching_list.begin()+1,matching_list.end(),
            [](const std::tuple<double,int,bool>& lhs,const std::tuple<double,int,bool>& rhs)->bool{
              return std::get<0>(lhs) > std::get<0>(rhs);
            });
          const std::tuple<double,int,bool>& most_match_info = matching_list.front();
          const int matched_order = std::get<1>(most_match_info);

          std::vector<cv::RotatedRect> group;
          if(std::get<2>(most_match_info))
          {
			//matched_order - i - j と存在
            group.push_back(sources[matched_order]);
            group.push_back(sources[i]);
            group.push_back(sources[j]);
            sources.erase(sources.begin()+j);
            sources.erase(sources.begin()+i);
            sources.erase(sources.begin()+matched_order);
          }
          else
          {
			//i - j - matched_order と存在
            group.push_back(sources[i]);
            group.push_back(sources[j]);
            group.push_back(sources[matched_order]);
            sources.erase(sources.begin()+matched_order);
            sources.erase(sources.begin()+j);
            sources.erase(sources.begin()+i);
          }
          groups.push_back(std::move(group));

        }
        else
        {
          //マッチングが無いということは ( このふたつのグループ | これを含む四角形(あとでくっつける) )
          std::vector<cv::RotatedRect> group;
          group.push_back(sources[i]);
          group.push_back(sources[j]);
          sources.erase(sources.begin()+j);
          sources.erase(sources.begin()+i);
          groups.push_back(std::move(group));
        }
      }
    }
  }

  std::cout << "Remain Source: " << sources.size() << std::endl;
  if(sources.size() > 0) grouping(groups,sources);

  return;
}

//2～3のグループを更に合体したりしなかったり
void merge(std::vector<std::vector<cv::RotatedRect>>& groups)
{
  for(size_t i=0; i<groups.size(); ++i)
  {
    for(size_t j=i+1; j<groups.size();)
    {
      const line_segment<float> i_line(groups[i].front().center, groups[i].back().center);
      const line_segment<float> j_line(groups[j].front().center, groups[j].back().center);

      if(i_line || j_line)
      {
        std::vector<cv::RotatedRect> v;
        for(auto it = groups[i].cbegin(), end = groups[i].cend(); it != end; ++it) v.push_back(*it);
        for(auto it = groups[j].cbegin(), end = groups[j].cend(); it != end; ++it) v.push_back(*it);
        groups[i] = std::move(v);
        groups.erase(groups.begin()+j);

        ++i;
        j=i+1;
        return;
      }
      else ++j;
    }
  }
}

int main()
{
  //imageにBGR画像取り込み，hsvにHSV形式に変換したものを格納
  //一般にコンピュータで扱われるJPEG/BMP等はBGRで保存されていて，RGBではない
  cv::Mat image = cv::imread("./0731_pic/black_one_normal.jpg",1),hsv;
  image.convertTo(image,image.type(),2,-100);

  cv::cvtColor(image,hsv,CV_BGR2HSV);
  
  //実装済みの関数をコール
  cv::Mat red,black,out;
  red_area(red,hsv);
  black_area(black,image);
  bitwise_or(red,black,out); //赤か黒のところ(->out)
  
  //輪郭探索
  cv::Mat temp = image.clone();
  {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(out,contours,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
    cv::drawContours(temp,contours,-1,CV_RGB(0,0,255),2); //tempに輪郭書き出し
  }
  
  //画面に表示
  cv::namedWindow("src");
  cv::namedWindow("temp");
  cv::imshow("src",image);
  cv::imshow("temp",temp);
  
  //今回の雑エリアの表示
  std::cout << "--Choose \"InActive Area\"--" << std::endl;
  {
    std::tuple<cv::Mat*,cv::Mat*,cv::Mat*> senddata = std::tuple<cv::Mat*,cv::Mat*,cv::Mat*>(&image,&red,&black); //setMouseCallbackで渡されるデータ
    cv::setMouseCallback("temp",&inactive_area_input,&senddata); //マウスクリック動作設定
  
    char inputkey;
    while(inputkey = cv::waitKey(0), inputkey != 'q'); //qが入力されるまで待機
  }
  cv::destroyWindow("temp");
  std::cout << "--End--\n" << std::endl;

  //
  // 処理開始
  //
  //std::cout << "Process Start" << std::endl;

  // 四角形と角度(RotatedRect)で全ての点を表す
  std::vector<cv::RotatedRect> black_boxes, red_boxes;
  {
    std::vector<std::vector<cv::Point>> black_contours, red_contours;
    cv::findContours(black,black_contours,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
    cv::findContours(  red,  red_contours,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
    for(size_t i=0; i<black_contours.size(); ++i) black_boxes.push_back(cv::minAreaRect(black_contours[i]));
    for(size_t i=0; i<red_contours.size();   ++i)   red_boxes.push_back(cv::minAreaRect(  red_contours[i]));
  }
  
  //大きいRectと小さいRectの削除
  erase_big_rect(red_boxes);
  erase_small_rect(red_boxes);
  erase_big_rect(black_boxes);
  erase_small_rect(black_boxes);
  
  //
  // (要検討)ここで中心が他の輪郭内に入るRectを消す処理を入れたほうが良さそうだよね
  //
  erase_overlap(red_boxes, black_boxes);

  //表示
  temp = image.clone();
  for(auto it = black_boxes.cbegin(), end = black_boxes.cend(); it != end; ++it)
  {
    cv::Point2f vtx[4];
    it->points(vtx);
    for(int i=0; i<4; ++i) cv::line(temp, vtx[i], vtx[i<3?i+1:0], cv::Scalar(100,100,200), 2, CV_AA);
  }
  for(auto it = red_boxes.cbegin(), end = red_boxes.cend(); it != end; ++it)
  {
    cv::Point2f vtx[4];
    it->points(vtx);
    for(int i=0; i<4; ++i) cv::line(temp, vtx[i], vtx[i<3?i+1:0], cv::Scalar(200,100,100), 2, CV_AA);
  }
  cv::namedWindow("rect_temp");
  cv::imshow("rect_temp",temp);

  //無駄な四角形を選択させ削除
  std::cout << "--Choose \"Disenable Rect\"--" << std::endl;
  {
    std::tuple<cv::Mat*,std::vector<cv::RotatedRect>*,std::vector<cv::RotatedRect>*> send_data(&image,&red_boxes,&black_boxes);
    cv::setMouseCallback("rect_temp",disenable_rect_input,&send_data);

    char inputkey;
    while(inputkey = cv::waitKey(0), inputkey != 'q'); //qが入力されるまで待機
  }
  cv::destroyWindow("rect_temp");
  std::cout << "--End--\n" << std::endl;

  //左上基準でソート．左上->右上->左下->右下(実装でλ式使ってるけど怖くないですよ？)
  std::sort(black_boxes.begin(),black_boxes.end(),
    [](const cv::RotatedRect& lhs,const cv::RotatedRect& rhs)->bool{
      return (lhs.center.y == rhs.center.y) ? (lhs.center.x < rhs.center.x) : (lhs.center.y < rhs.center.y);
    });

  //グループ分け
  std::cout << "--Start \"Grouping\"--" << std::endl;
  std::vector<std::vector<cv::RotatedRect>> groups;
  grouping(groups,black_boxes);
  merge(groups);
  std::cout << "--End--\n" << std::endl;

  //赤を突っ込む
  for(auto it=red_boxes.cbegin(), end=red_boxes.cend(); it != end; ++it)
  {
    std::vector<cv::RotatedRect> group;
    group.push_back(*it);
    groups.push_back(std::move(group));
  }
  
  //グループ書き出し．
  temp = image.clone();
  const CvScalar colors[] = {CV_RGB(0,0,255),CV_RGB(0,255,0),CV_RGB(255,0,0),CV_RGB(255,255,0),CV_RGB(0,255,255),CV_RGB(255,0,255),CV_RGB(255,255,255)};
  for(size_t i=0; i<groups.size(); ++i)
  {
    for(auto it = groups[i].cbegin(); it != groups[i].cend(); ++it)
    {
      cv::Point2f vtx[4];
      it->points(vtx);
      for(int j=0; j<4; ++j) cv::line(temp, vtx[j], vtx[j<3?j+1:0], colors[i%7], 2, CV_AA);
    }
  }
  cv::namedWindow("grouped_temp");
  cv::imshow("grouped_temp",temp);
  {
    char inputkey;
    while(inputkey = cv::waitKey(0), inputkey != 'q'); //qが入力されるまで待機
  }
  cv::destroyWindow("grouped_temp");

  //
  // 綺麗な矩形にならないですよね
  //
  temp = image.clone();
  std::vector<cv::RotatedRect> group_area;
  for(size_t i = 0; i < groups.size(); ++i)
  {
    std::vector<cv::Point2f> points;
    for(auto it = groups[i].begin(); it != groups[i].end(); ++it)
    {
      cv::Point2f vtx[4];
      it->points(vtx);
      for(int j=0; j<4; ++j) points.push_back(vtx[j]);
    }
    group_area.push_back(cv::minAreaRect(points));
  }
  for(size_t i = 0; i<group_area.size(); ++i)
  {
    //group_area[i].angle -= 40.0;
    cv::Point2f vtx[4];
    group_area[i].points(vtx);
    for(int j=0; j<4; ++j) cv::line(temp, vtx[j], vtx[j<3?j+1:0], colors[i%7], 2, CV_AA);
  }
  cv::namedWindow("grouped");
  cv::imshow("grouped",temp);
  {
    char inputkey;
    while(inputkey = cv::waitKey(0), inputkey != 'q'); //qが入力されるまで待機
  }
  cv::destroyWindow("grouped");


  //cv::waitKey(0);


  return 0;
}
