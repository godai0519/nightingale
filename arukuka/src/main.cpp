
/*
 * C++11 を使っています、 g++ (GCC) 4.7.1 で。
 * もしかしたら他の環境ではエラーが出るかもしれません。
 * g++-4.7 -std=c++11 main.cpp -o main -Wall `pkg-config --libs opencv` `pkg-config --cflags opencv`
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <thread>
#include <vector>

#ifdef _WIN32
#ifdef _DEBUG
	#pragma comment(lib,"opencv_core242d.lib")
	#pragma comment(lib,"opencv_imgproc242d.lib")
	#pragma comment(lib,"opencv_highgui242d.lib")
#else
	#pragma comment(lib,"opencv_core242.lib")
	#pragma comment(lib,"opencv_imgproc242.lib")
	#pragma comment(lib,"opencv_highgui242.lib")
#endif
#endif

#define INF (1>>28)

cv::Mat src;
cv::Mat out;
cv::RNG rng(12345);
int n=-1;
void draw();
const int MAX_SIZE = 1000;

std::map<int, int> _map;

void label(cv::Mat& dst, int x, int y){
  static const int data[] = {dst.data[0], dst.data[1], dst.data[2]};
  static const int ofs[4][2] = {{0,1},{1,0},{0,-1},{-1,0}};
  int a = dst.step*y+(x*3);
  for(int i=0; i<3; ++i){
	  dst.data[a+i]=0;
  }
  for(int i=0; i<4; ++i){
	  int nx = x + ofs[i][0], ny =  y + ofs[i][1];
	  if(0<=nx && nx<dst.cols && 0<=ny && ny<dst.rows){
		  bool f=true;
		  int b = dst.step*ny+(nx*3);
		  for(int j=0; j<3; ++j){
			  f &= dst.data[b+j]==data[j];
		  }
		  if(f){
			  label(dst, nx, ny);
		  }
	  }
  }
}

//赤いエリアを抜き出す関数
void red_area(cv::Mat& dst,const cv::Mat& hsv)
{
  
  //HSVの要素分解
  cv::Mat channels[3],h_mask,h_mask_low,h_mask_high,c_mask;
  cv::split(hsv,channels);

  cv::threshold(channels[0],h_mask_low,10,255,cv::THRESH_BINARY); //色相が赤付近かどうか
  cv::threshold(channels[0],h_mask_high,170,255,cv::THRESH_BINARY); //色相が赤付近かどうか
  cv::threshold(channels[1],c_mask,50,255,cv::THRESH_BINARY); //彩度(信用性)が高いかどうか

  
  cv::bitwise_not(h_mask_low,h_mask_low);
  cv::bitwise_or(h_mask_low,h_mask_high,h_mask);

  //色相が赤付近で，彩度が高い部分(->dst)
  cv::bitwise_and(h_mask,c_mask,dst);

  //厳しめの判定をしたから膨張させる
  cv::dilate(dst,dst,cv::Mat());
//  label(dst, 0, 0);
  cv::bitwise_not(dst,dst);
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
    std::cout << "(" << start_x << "," << start_y << ") - (" << end_x << "," << end_y << ")¥n" << std::endl;

    //選択範囲の塗りつぶし
    for(int i = start_y; i < end_y; ++i)
      for(int j = start_x; j < end_x; ++j)
        out->at<unsigned char>(i,j) = 0;
    
    draw();

  }
}

inline int pick_out_data(const int arr, const int req){
	return (arr>>(req*8) & 0xff);
}

// 悟大様のコードパクリ
void set_answer_area(int event, int x, int y, int flags, void* param){
	static short start_x,end_x; // VC10もGCCもshortが2バイトでコンパイルされるのを悪用．
	static const std::string target[3] = {"small", "medium", "large"};
	static const int mask[3] = {0xffff00, 0xff00ff, 0x00ffff};	//めんどいのでLUT()

	if(event == CV_EVENT_LBUTTONDOWN)
	{
		//クリック開始時の座標を格納
		start_x = x;
		std::cout << "Click" << std::endl;
	}
	if(event == CV_EVENT_LBUTTONUP)
	{
		//ドラッグ終了時の座標を格納
		end_x = x;
	
		//ドラッグ状況の表示
		std::cout << "UnClick" << std::endl;
		std::cout << start_x << "," << end_x << std::endl;

		const std::pair<int *, int *> *data = reinterpret_cast<std::pair<int *,int *>*>(param);
		int *res = data->first;
		int *req = data->second;
		
		// 今変更をいれるところ(*reqの場所)を0クリアする
		*res = (*res)&mask[*req];

		//startが左上，endが右下に来るように修正
		if(start_x > end_x)
		{
			short tmp = start_x;
			start_x = end_x;
			end_x = tmp;
		}

		//画面外へのクリックを修正
		start_x = std::max((short)0,start_x);
		end_x   = std::min((short)MAX_SIZE,end_x);	// Result 画面のサイズが1000,300なので。げろコード

		//修正結果
		std::cout << start_x << "," << end_x << std::endl;
	
		int count=0;
		for(std::map<int,int>::iterator it = _map.begin(); it!=_map.end(); ++it){
			int S = (*it).first;
			int V = (*it).second;
			if(n==1){ V = (int)((V / 3.5) + .5); }
			if(MAX_SIZE<=S)break;
			if(V>0){
				if(start_x <= S && S <= end_x){
					count += V;
				}
			}
		}
		for(int i=0; i<8; ++i){
			*res = *res | ( (((count>>i)&0x01)<<i) << ((*req)*8) );
		}

		std::cout << "Now target..." << target[*req] << std::endl;
		std::cout << "catch..." << pick_out_data(*res, *req) << std::endl;
		if(++(*req)>=3){
			*req = 0;
		}

	}
}

void draw(){
	cv::vector<std::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> hierarchy;

	cv::findContours( out, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
	
	// 表示に必要な変数の初期化をします。
	cv::vector<std::vector<cv::Point>> contours_poly( contours.size() );
	cv::vector<cv::Rect> boundRect( contours.size() );
	cv::vector<cv::Point2f>center( contours.size() );
	cv::vector<float>radius( contours.size() );
	for( uint i = 0; i < contours.size(); i++ ){
		// 輪郭線をより直線に近い形に近似します。いらない情報をカットする目的としても良いかもしれません。
		cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true ); 
		// 近似された輪郭線を構成する点の傾いていない外接短形を求めます。
		boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) ); 
		// 近似された輪郭線を構成する点を囲む最小の円を求めます。
		cv::minEnclosingCircle( contours_poly[i], center[i], radius[i] );
	}

	int count = 0;
	_map.clear();
	/// 表示部分です。
	cv::Mat drawing = cv::Mat::zeros( out.size(), CV_8UC3 );
	for( uint i = 0; i< contours.size(); i++ ){ 
		cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		int width = boundRect[i].br().x - boundRect[i].tl().x;
		int height = boundRect[i].br().y - boundRect[i].tl().y;
		cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
		cv::circle( drawing, center[i], (int)radius[i], color, 2, CV_AA);
		++count;
		if(n==0){width>>=1; height>>=1;}
		std::cout << width << ", " << height << std::endl;
		int S = width*height;
		if(_map.find(S)!=_map.end()){
			_map[S] = _map[S]+1;
		}else{
			_map[S] = 1;
		}
	}
	cv::namedWindow( "Contours", CV_WINDOW_NORMAL );
	cv::imshow( "Contours", drawing );
	
	//CUI表示部分
	std::cout << "count = " << (n==0 ? count : (int)(count/3.5 + .5)) << std::endl;
}

void sharp(cv::Mat const& src, cv::Mat& dst, float k = 2.0f)
{
	float kernelData[] = {
		-k/9.0f, -k/9.0f, -k/9.0f,
		-k/9.0f, 1 + (8*k)/9.0f, -k/9.0f,
		-k/9.0f, -k/9.0f, -k/9.0f,
	};
	const cv::Mat kernel(3, 3, CV_32F, kernelData);
	cv::filter2D(src, dst, -1, kernel);
}

void _gamma(cv::Mat const& src, cv::Mat& dst, unsigned char const LUT[256], int const S[2], int const E[2])
{
	for(int y=S[1]; y<E[1]; ++y){
		for(int x=S[0]; x<E[0]; ++x){
			const cv::Vec3b p = src.at<cv::Vec3b>(y,x);
			dst.at<cv::Vec3b>(y,x) = cv::Vec3b(LUT[p[0]], LUT[p[1]], LUT[p[2]]);
		}
	}
}

void gamma(cv::Mat const& src, cv::Mat& dst, double gamma = 2.0)
{
	static bool f = true;
	static unsigned char LUT[256];   
	
	if(f){
		f = false;         
		//ガンマ補正テーブルの作成   
		for (int i = 0; i < 256; i++){   
			LUT[i] = (int)(pow((double)i / 255.0, 1.0 / gamma) * 255.0);   
		}
	}
	
	int p[2][2][2][2];	// アドレスが変わらないように...
	std::vector<std::thread> th;
	for(int y=0; y<2; ++y){
		for(int x=0; x<2; ++x){
			int S[2] = {(src.cols/2)*x, (src.rows/2)*y};
			int E[2] = {(src.cols/2)*(x+1), (src.rows/2)*(y+1)};
			p[y][x][0][0] = S[0];  p[y][x][0][1] = S[1];
			p[y][x][1][0] = E[0];  p[y][x][1][1] = E[1];
			th.push_back(std::thread(std::bind(_gamma, src, std::ref(dst), LUT, p[y][x][0], p[y][x][1])));
		}
	}
	for(auto& i : th) i.join();
}

void normalise(cv::Mat const& src, cv::Mat& dst)
{
	double max = 0., min = 0.;
	cv::minMaxLoc(src, &min, &max);
	cv::convertScaleAbs(src, dst, 255.0/(max-min), (255.0/(max-min))*(-min));
}

void pre_pros_red(cv::Mat const& src, cv::Mat &dst, cv::Mat &red){
	cv::medianBlur(src, dst, 13);
	normalise(dst, dst);
	
	cv::Mat hsv;
	cv::cvtColor(dst, hsv, CV_BGR2HSV);
	red_area(red,hsv);
}
void pre_pros_black(cv::Mat const& src, cv::Mat &dst, cv::Mat &black){
	cv::dilate(src, dst, cv::Mat());
	normalise(dst, dst);
	sharp(dst, dst, 6.4f);
	
	black_area(black,dst);
}


int main(int argc, char** argv)
{
	if(argc>1){
		time_t start, stop;
		std::time(&start);
		int ans[3] = {0};
		for(int num=1; num<argc; ++num){
			// 画像の読み込み
			src = cv::imread(argv[num], 1);
			cv::resize(src, src, cv::Size(1200, 900));
			gamma(src, src);			// 共通前処理
			
		
			// ウィンドウ表示部分の初期化です。
			std::string source_window = "Source";
			cv::namedWindow( source_window, CV_WINDOW_NORMAL );
			std::string result_window = "Result";	// 表示したりは消したりするので
	
			// GUI部分です。
			std::pair<cv::Mat*,cv::Mat*> senddata = std::pair<cv::Mat*,cv::Mat*>(&src,&out); //setMouseCallbackで渡されるデータ
			cv::setMouseCallback(source_window, &inactive_area_input, &senddata); //マウスクリック動作設定	
			// Result については表示したときに逐一設定します。


			// 赤と黒を取り出す
			cv::Mat dst[2];
			cv::Mat red,black;
			std::thread th_red(std::bind(pre_pros_red, src, std::ref(dst[0]), std::ref(red)));
			std::thread th_black(std::bind(pre_pros_black, src, std::ref(dst[1]), std::ref(black)));

			th_red.join();
			th_black.join();
			cv::Mat img[2] = {red.clone(), black.clone()};

			for(int i=0; i<2; ++i){
				out = img[i].clone();
				cv::imshow( source_window, dst[i] );
				n=i; draw();
				for(char inputkey = 0; inputkey != 'q';inputkey = cv::waitKey(0)); //qが入力されるまで待機
				//_map -> ans 写
				std::map<int,int>::iterator it = _map.begin();
				cv::Mat drawing = cv::Mat::zeros( cv::Size(1000,300), CV_8UC3 );
				while(it!=_map.end()){
					cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
					int S = (*it).first;
					int V = (*it).second;
					if(n==1){ V = (int)((V / 3.5) + .5); }
					if(1000<=S)break;
					if(V>0){
						cv::line(drawing, cv::Point(S,0), cv::Point(S,V*15<drawing.rows ? V*15 : drawing.rows-1), color);
						std::cout << S << ":" << V << std::endl;
					}
					++it;
				}
				cv::namedWindow( result_window, CV_WINDOW_AUTOSIZE );
				cv::imshow(result_window, drawing);
				int count[3] = {0}, target = 0;
				int _count = 0;
				std::pair<int *,int *> req = std::pair<int *,int *>(&_count, &target);
				cv::setMouseCallback(result_window, &set_answer_area, &req);
				std::cout << "set answer area" << std::endl;
				for(char inputkey = 0; inputkey != 'q';inputkey = cv::waitKey(0)); //qが入力されるまで待機
				cv::destroyWindow(result_window);
				
				for(int j=0; j<3; ++j){
					count[j] = pick_out_data(_count, j);
				}
				std::cout << "Result..." << count[0] << ", " << count[1] << ", " << count[2] << std::endl;
				
				for(int j=0; j<3; ++j){
					ans[j] += count[j];
				}
			}
		}
		for(int i=0; i<3; ++i){
			ans[i] = (int)(ans[i]/((double)(argc-1)) + .5);
		}
		std::cout << std::endl;
		std::cout << "Answer..." << ans[0] << ", " << ans[1] << ", " << ans[2] << std::endl;
		std::time(&stop);
		std::cout << "Time..." << std::difftime(stop, start) << std::endl;
		
		double g=INF;
		std::cin >> g;
		g -= ans[2]*5.902;
		g -= ans[1]*1.35;
		int _small = (int)(g/0.274 + 0.5);
		ans[0] = std::min(ans[0], _small);	// ノイズ混入が考えられるのでminで
		
		std::cout << "Answer..." << ans[0] << ", " << ans[1] << ", " << ans[2] << std::endl;
	}
	
	return 0;
}

