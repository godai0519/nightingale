#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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

//赤いエリアを抜き出す関数
void red_area(cv::Mat& dst,const cv::Mat& hsv)
{
  cv::Mat channels[3],h_mask,c_mask;
  cv::split(hsv,channels);

  cv::threshold(channels[0],h_mask,160,255,cv::THRESH_BINARY); //色相が赤付近かどうか
  cv::threshold(channels[1],c_mask,150,255,cv::THRESH_BINARY); //彩度(信用性)が高いかどうか

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
  cv::threshold(channels[0],r,50,255,cv::THRESH_BINARY);
  cv::threshold(channels[1],g,50,255,cv::THRESH_BINARY);
  cv::threshold(channels[2],b,50,255,cv::THRESH_BINARY);

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

std::vector<std::vector<cv::Point>> contours;

//マウスクリック用関数(クリックする度に呼ばれます)
//cv::setMouseCallbackのドキュメントに注目
void inactive_area_input(int event, int x, int y, int flags, void* param){
  static short start_x,start_y,end_x,end_y; // VC10もGCCもshortが2バイトでコンパイルされるのを悪用．

  if(event == CV_EVENT_LBUTTONDOWN)
  {
    //クリック開始時の座標を格納
    start_x = x; start_y = y;
    std::cout << "Click" << std::endl;
  }
  if(event == CV_EVENT_LBUTTONUP)
  {
    //ドラッグ終了時の座標を格納
    end_x = x; end_y = y;

    //ドラッグ状況の表示
    std::cout << "UnClick" << std::endl;
    std::cout << "(" << start_x << "," << start_y << ") - (" << end_x << "," << end_y << ")" << std::endl;

    //mainのsenddataとやらはparamに来るので，ポインタ変換
    const std::pair<cv::Mat*,cv::Mat*> *data = reinterpret_cast<std::pair<cv::Mat*,cv::Mat*>*>(param);
    cv::Mat* out = data->second;

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
    start_x = std::max((short)0,start_x);
    start_y = std::max((short)0,start_y);
    end_x   = std::min((short)out->cols,end_x);
    end_y   = std::min((short)out->rows,end_y);

    //修正結果
    std::cout << "(" << start_x << "," << start_y << ") - (" << end_x << "," << end_y << ")\n" << std::endl;

    //選択範囲の塗りつぶし
    for(int i = start_y; i < end_y; ++i)
      for(int j = start_x; j < end_x; ++j)
        out->at<unsigned char>(i,j) = 0;
    
    //輪郭情報の削除と再探索
    contours.clear();
    cv::findContours(*out,contours,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);

    //上書き表示
    cv::Mat temp = data->first->clone();
    cv::drawContours(temp,contours,-1,CV_RGB(0,0,255),2);
    cv::imshow("temp",temp);
  }
}

int main()
{
  //imageにBGR画像取り込み，hsvにHSV形式に変換したものを格納
  //一般にコンピュータで扱われるJPEG/BMP等はBGRで保存されていて，RGBではない
  cv::Mat image = cv::imread("img.jpg",1),hsv;
  cv::cvtColor(image,hsv,CV_BGR2HSV);
  
  //実装済みの関数をコール
  cv::Mat red,black,out;
  red_area(red,hsv);
  black_area(black,image);
  bitwise_or(red,black,out); //赤か黒のところ(->out)
  
  //輪郭探索
  cv::findContours(out,contours,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);

  //tmpにimageを完全コピー(クローン)
  cv::Mat temp = image.clone();
  cv::drawContours(temp,contours,-1,CV_RGB(0,0,255),2); //tempに輪郭書き出し
  
  //画面に表示
  cv::namedWindow("src");
  cv::namedWindow("temp");
  cv::imshow("src",image);  
  cv::imshow("temp",temp);
   
  std::pair<cv::Mat*,cv::Mat*> senddata = std::pair<cv::Mat*,cv::Mat*>(&image,&out); //setMouseCallbackで渡されるデータ
  cv::setMouseCallback("temp",&inactive_area_input,&senddata); //マウスクリック動作設定
  
  char inputkey;
  while(inputkey = cv::waitKey(0), inputkey != 'q'); //qが入力されるまで待機

  //
  // 処理！！
  //

  return 0;
}
