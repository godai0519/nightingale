#include <iostream>
#include <string>
#include <cv.h>
#include <highgui.h>

/*
note:
	g++ GUIsample.cpp -g `pkg-config --cflags opencv` `pkg-config --libs opencv`
*/

cv::Mat src;
cv::Mat src_gray;
int thresh = 64;
int max_thresh = 255;
cv::RNG rng(12345);
int mode = 1;
int max_mode = 1;

// 今回の主役関数のプロトタイプ宣言です。
void thresh_callback(int, void* );

int main( int argc, char** argv )
{
	// 実行時の第二引数のイメージを取り込みます。
	src = cv::imread(argv[1]);
	// src を CV_BGR2GRAY型(グレースケール)に変換して、平滑化させます。
	cv::cvtColor(src, src_gray, CV_BGR2GRAY);
	cv::blur(src_gray, src_gray, cv::Size(3,3));

	// ウィンドウ表示部分の初期化です。
	std::string source_window = "Source";
	cv::namedWindow( source_window, CV_WINDOW_AUTOSIZE );
	cv::imshow( source_window, src );
	
	// GUI部分です。
	// トラックバーを動かすたびに、thresh_callbackが呼ばれます
	cv::createTrackbar( " Threshold:", "Source", &thresh, max_thresh, thresh_callback );
	cv::createTrackbar( " Mode:", "Source", &mode, max_mode, thresh_callback );
	// とりあえず(トラックバーを動かしていない)初期状態での表示を行わせます。
	thresh_callback( 0, 0 );

	cv::waitKey(0);
	return 0;
}

void thresh_callback(int, void* )
{
	cv::Mat threshold_output;
	cv::vector<std::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> hierarchy;

	// トラックバーの値に応じて敷居処理を行います。
	cv::threshold( src_gray, threshold_output, thresh, 255, cv::THRESH_BINARY );
	// 輪郭を抽出します。
	cv::findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	// 表示に必要な変数の初期化をします。
	cv::vector<std::vector<cv::Point> > contours_poly( contours.size() );
	cv::vector<cv::Rect> boundRect( contours.size() );
	cv::vector<cv::Point2f>center( contours.size() );
	cv::vector<float>radius( contours.size() );
	for( int i = 0; i < contours.size(); i++ ){
		// 輪郭線をより直線に近い形に近似します。いらない情報をカットする目的としても良いかもしれません。
		cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true ); 
		// 近似された輪郭線を構成する点の傾いていない外接短形を求めます。
		boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) ); 
		// 近似された輪郭線を構成する点を囲む最小の円を求めます。
		cv::minEnclosingCircle( contours_poly[i], center[i], radius[i] );
	} 


	int count = 0;
	/// 表示部分です。必要でなさそうな輪郭線(例:半径が小さすぎる)はif文でカットしています。
	cv::Mat drawing = cv::Mat::zeros( threshold_output.size(), CV_8UC3 );
	for( int i = 0; i< contours.size(); i++ ){ 
		cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		switch(mode){
		case 0:
			if(drawing.cols/32 < (int)radius[i] && (int)radius[i] < drawing.cols/2 ){
				cv::circle( drawing, center[i], (int)radius[i], color, 2, CV_AA);
				cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
				if(drawing.cols/32 < (int)radius[i] && (int)radius[i] < drawing.cols/2){ ++count; }
			}
			break;

		case 1:
			cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
			cv::circle( drawing, center[i], (int)radius[i], color, 2, CV_AA);
			++count;
			break;
		default:
			break;
	
		}

	}
	cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
	cv::imshow( "Contours", drawing );
	
	//CUI表示部分
	std::cout << "count = " << (mode==0 ? count : (int)(count/3.5 + .5)) << std::endl;
}
