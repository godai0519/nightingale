/*
忘れないようメモ
	g++ algo.cpp -g `pkg-config --cflags opencv` `pkg-config --libs opencv`

*/

#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

#ifdef _DEBUG
	#pragma comment(lib,"opencv_core242d.lib")
	#pragma comment(lib,"opencv_imgproc242d.lib")
	#pragma comment(lib,"opencv_highgui242d.lib")
#else
	#pragma comment(lib,"opencv_core242.lib")
	#pragma comment(lib,"opencv_imgproc242.lib")
	#pragma comment(lib,"opencv_highgui242.lib")
#endif

#define square(a) ((a)*(a))

#define OPENCV_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))
#define OPENCV_VERSION_CODE OPENCV_VERSION(CV_MAJOR_VERSION, CV_MINOR_VERSION, CV_SUBMINOR_VERSION)

#if OPENCV_VERSION_CODE < OPENCV_VERSION(2,3,1)
namespace cv
{
	enum {
		EVENT_MOUSEMOVE      =CV_EVENT_MOUSEMOVE,
		EVENT_LBUTTONDOWN    =CV_EVENT_LBUTTONDOWN,
		EVENT_RBUTTONDOWN    =CV_EVENT_RBUTTONDOWN,
		EVENT_MBUTTONDOWN    =CV_EVENT_MBUTTONDOWN,
		EVENT_LBUTTONUP      =CV_EVENT_LBUTTONUP,
		EVENT_RBUTTONUP      =CV_EVENT_RBUTTONUP,
		EVENT_MBUTTONUP      =CV_EVENT_MBUTTONUP,
		EVENT_LBUTTONDBLCLK  =CV_EVENT_LBUTTONDBLCLK,
		EVENT_RBUTTONDBLCLK  =CV_EVENT_RBUTTONDBLCLK,
		EVENT_MBUTTONDBLCLK  =CV_EVENT_MBUTTONDBLCLK
	};
	enum {
		EVENT_FLAG_LBUTTON   =CV_EVENT_FLAG_LBUTTON,
		EVENT_FLAG_RBUTTON   =CV_EVENT_FLAG_RBUTTON,
		EVENT_FLAG_MBUTTON   =CV_EVENT_FLAG_MBUTTON,
		EVENT_FLAG_CTRLKEY   =CV_EVENT_FLAG_CTRLKEY,
		EVENT_FLAG_SHIFTKEY  =CV_EVENT_FLAG_SHIFTKEY,
		EVENT_FLAG_ALTKEY    =CV_EVENT_FLAG_ALTKEY
	};
}
#endif


/**
 * XORShift
 */
class Random {
	private:
		unsigned long seed128[4];

	public:
		Random(){
			this->seed((unsigned int)std::time(NULL));
		}
		
		void seed(unsigned int s){
			for(int i=1; i<=4; i++){
				seed128[i-1] = s = 1812433253U * (s^(s>>30)) + i;
			}
		}
		unsigned long xor128(){
			unsigned long t;
			t=(seed128[0]^(seed128[0]<<11));
			seed128[0]=seed128[1];
			seed128[1]=seed128[2];
			seed128[2]=seed128[3];
			return seed128[3]=(seed128[3]^(seed128[3]>>19))^(t^(t>>8));
		}
		unsigned long getRandom(unsigned long min, unsigned long max){
			
			return min + (unsigned long)(this->xor128()*(max-min+1.)/(1.+ULONG_MAX));
		}
};

static Random *r = new Random();

/**
 * パラメータの構造体です。
 */
struct param_t {
	unsigned int angle;
	unsigned int scale;
	unsigned int x;
	unsigned int y;
	
	unsigned int _org_cols;
	unsigned int _org_rows;
	unsigned int _tmp_cols;
	unsigned int _tmp_rows;
	param_t(){}
	
	void preset(const cv::Mat org, const cv::Mat tmp){
		this->_org_cols = org.cols;
		this->_org_rows = org.rows;
		this->_tmp_cols = tmp.cols;
		this->_tmp_rows = tmp.rows;

		this->rand();
	}
	void preset(const param_t v){
		this->_org_cols = v._org_cols;
		this->_org_rows = v._org_rows;
		this->_tmp_cols = v._tmp_cols;
		this->_tmp_rows = v._tmp_rows;
		
		this->rand();
	}
	
	void getOffsets(int ofs[2]){
		ofs[0] = this->x;
		ofs[1] = this->y;
	}
	
	unsigned int getParam(int num){
		switch(num){
			case 0: return this->angle;
			case 1: return this->scale;
			case 2: return this->x;
			case 3: return this->y;
		}
	}

	void rand(){
//		static bool pre = false;
		unsigned int a=_org_cols, b=_org_rows, c=_tmp_cols, d=_tmp_rows;
		this->angle = r->getRandom(0, 360);
		this->scale = r->getRandom(4, 36);
		this->x = r->getRandom(0, a - (int)(c * this->scale/10.));
		this->y = r->getRandom(0, b - (int)(d * this->scale/10.));
/*		if(!pre){
			this->angle=0;
			this->scale=10;
			this->x = 584;
			this->y = 483;
			pre = true;
		}*/
	}
};



class Util
{
	friend class Algorithm;
	
	private:
		static void toGrayScale(cv::Mat &img){
			cv::Mat g = img.clone();
			cv::cvtColor(img, g, CV_RGB2GRAY);
			img = g.clone();
		}
		static void toHSV(cv::Mat &img){
			cv::Mat dst = img.clone();
			cv::cvtColor(img, dst, CV_RGB2HSV);
			img = dst.clone();
		}
		static void toEdge(cv::Mat &img){
			cv::Mat g = img.clone();
			cv::Canny(img, g, 50, 200);
			img = g.clone();
		}
		/**
		 * imgに保存されている画像の(x, y)座標に格納されている輝度値を返します。
		 */
		static double getLumi(const cv::Mat img, int x, int y){
			if(img.rows < y) return 0;
			cv::Scalar intensity = img.at<unsigned char>(y, x);
			return intensity.val[0];
		}
		/**
		 * 色の距離を返します。
		 */
		static double getColorDiff(const cv::Mat img, const cv::Mat tmp, int x, int y, int ofs[2]){
			return Util::getColorDiff(img, tmp, x, y, ofs[0], ofs[1]);
		}
		static double getColorDiff(const cv::Mat img, const cv::Mat tmp, int x, int y, int ox, int oy){
			if(img.cols <= x+ox || img.rows <= y+oy){ return -1.;}
			const cv::Vec3b &p_img = img.at<cv::Vec3b>( y+oy, x+ox );
			const cv::Vec3b &p_tmp = tmp.at<cv::Vec3b>( y, x );
			bool all_zero = true;
			double sum = 0.;
			for(int i=0; i<3; ++i){
				sum += square((int)(p_img[i] - p_tmp[i]));
				all_zero &= (int)p_tmp[i]==0;
			}
			if(all_zero){return -1;}
			return sum;
		}
		/**
		 * scale倍して返します。
		 */
		static cv::Mat getScaled(const cv::Mat img, float scale){
			cv::Mat scaled;
			cv::Mat m_scale = cv::getRotationMatrix2D(cv::Point2f(0, 0), 0., scale);
			cv::warpAffine(img, scaled, m_scale, cv::Size(img.cols*scale, img.rows*scale));

			return scaled.clone();
		}
		/**
		 * 角度に対する辺の長さを返します。
		 */
		static int getNEWcols(const cv::Mat img, float angle){
			angle = (int)angle%45;
			return (img.cols*std::cos( (angle*180.)/M_PI ) + img.rows*std::sin( (angle*180.)/M_PI ) );
		}
		static int getNEWrows(const cv::Mat img, float angle){
			angle = (int)angle%45;
			return (img.rows*std::cos( (angle*180.)/M_PI ) + img.cols*std::sin( (angle*180.)/M_PI ) );
		}
		/**
		 * 中心からangle[deg]回転する。
		 */
		static cv::Mat getRotated(const cv::Mat img, float angle){
			const cv::Point2f src_pt[] = {cv::Point2f(0, 0), cv::Point2f(0, 1), cv::Point2f(1, 0)};
			const int next[] = { (Util::getNEWcols(img, angle)-img.cols)/2., (Util::getNEWrows(img, angle)-img.rows)/2.};
			const cv::Point2f dst_pt[] = {cv::Point2f(next[0], next[1]), cv::Point2f(next[0], next[1]+1), cv::Point2f(next[0]+1, next[1])};
			cv::Mat m_shift = cv::getAffineTransform(src_pt, dst_pt);
			cv::Mat shifted;
			cv::warpAffine(img, shifted, m_shift, img.size());

			cv::Mat m_rotate = cv::getRotationMatrix2D(cv::Point2f(shifted.cols/2, shifted.rows/2), angle, 1.);
			cv::Mat rotated;
			cv::warpAffine(img, rotated, m_rotate, img.size());

			return rotated.clone();
		}
		/**
		 * 回転と拡大, 縮小のパック
		 */
		static cv::Mat getOperated(const cv::Mat img, float angle, float scale){
			return Util::getRotated(Util::getScaled(img, scale) ,angle);
		}
		static cv::Mat getOperated(const cv::Mat img, const param_t p){
			return Util::getRotated(Util::getScaled(img, p.scale/10.) ,(float)p.angle);
		}

		/**
		 * 階調値の距離を返します。
		 */
		static int getDistance(const cv::Mat org, const cv::Mat tmp, const int x, const int y, const int ofs[2]){
			return Util::getDistance(org, tmp, x, y, ofs[0], ofs[1]);
		}
		static double getDistance(const cv::Mat org, const cv::Mat tmp, const int x, const int y, const int ox, const int oy){
			return square(Util::getLumi(org, x+ox, y+oy) - Util::getLumi(tmp, x, y));
		}
		
		/**
		 * Vに格納されているパラメータの情報、tempを当てる範囲を
		 * trgのクローンに書き出して表示します。
		 */
		static void show(const cv::Mat trg, const cv::Mat temp, const std::vector<param_t> V){
			cv::Mat dst = trg.clone();
			for(int i=V.size()-1; 0<=i; --i){
				cv::Scalar color = i==0 ? cv::Scalar(0, 0, 160) : cv::Scalar(160, 160, 0);
				cv::rectangle(dst, cv::Point(V[i].x, V[i].y), cv::Point(V[i].x + temp.cols * V[i].scale/10., V[i].y + temp.rows * V[i].scale/10.), color, 4, 8);
			}
			cv::namedWindow("result", CV_WINDOW_NORMAL);
			cv::imshow("result", dst);
			cv::waitKey(0);
		}
	
		/**
		 * マッチング率を返します。
		 */
		static double getConcord(const cv::Mat trg, const cv::Mat tmp, const param_t design){
			cv::Mat dst = tmp.clone();;
			dst = Util::getOperated(dst, design);
			int count = 0;
			double sum = 0.;
			for(int y=0; y<dst.rows; ++y){
				for(int x=0; x<dst.cols; ++x){
					int offsets[2] = { design.x, design.y};
					double v = Util::getColorDiff(trg, dst, x, y, offsets);
					if(0<v){
						++count;
						sum += v;
					}
				}
			}
			return 1. - (sum/(square(255.) * 3.))/(double)count;
		}
		/**
		 * 一様交叉
		 * 値の幅を通り越す可能性大で対策も見つからないので使っていません。
		 */
		static unsigned int uni_cross(const unsigned int a, const unsigned int b){
			unsigned int c = 0;
			for(int i=0; i<sizeof(int)*8; ++i){
				c |= ((r->getRandom(0,1)==0 ? a>>i : b>>i)&0x01)<<i;
			}
			return c;
		}
		/**
		 * 交叉
		 *   1/2:ランダムのどれか一つのパラメータ
		 *   1/2:自分とランダムのどれか一つを足して2で割る
		 * 突然変異
		 *   1/9の確立
		 */
		static void generate(const std::vector<param_t> V, std::vector<param_t> &params){
			params.clear();
			for(int i=0; i<V.size()<<1; ++i){
				param_t next;
				next.preset(V[0]);
				next.angle = r->getRandom(0,8)==0 ? next.angle : ( r->getRandom(0,1)==0 ? V[r->getRandom(0,V.size()-1)].angle : (V[r->getRandom(0,V.size()-1)].angle + V[i%V.size()].angle)/2 );
				next.scale = r->getRandom(0,8)==0 ? next.scale : ( r->getRandom(0,1)==0 ? V[r->getRandom(0,V.size()-1)].scale : (V[r->getRandom(0,V.size()-1)].scale + V[i%V.size()].scale)/2 );
				next.x = r->getRandom(0,8)==0 ? next.x : ( r->getRandom(0,1)==0 ? V[r->getRandom(0,V.size()-1)].x : (V[r->getRandom(0,V.size()-1)].x + V[i%V.size()].x)/2 );
				next.y = r->getRandom(0,8)==0 ? next.y : ( r->getRandom(0,1)==0 ? V[r->getRandom(0,V.size()-1)].y : (V[r->getRandom(0,V.size()-1)].y + V[i%V.size()].y)/2 );
				params.push_back(next);
			}
		}
	
};

class Algorithm
{
	private:
		cv::Mat temp;
		cv::Mat target;
		int temp_num;
		std::string target_name;
		std::string display;

		cv::Mat getTemp(int num){
			std::stringstream ss;
			ss << "temp_" << num << ".png";
			return cv::imread(ss.str());
		}
	public:
		Algorithm(const std::string& filename){
			this->target = cv::imread(filename);
			this->display = "Display Image";
			this->target_name = filename;
			this->temp_num = 1;
			this->temp = this->getTemp(this->temp_num);
		}
		Algorithm(){
			*this = Algorithm("sample.jpg");
		}
		~Algorithm(){
			this->temp.release();
			this->target.release();
		}
		
		void run(){
			while(true){
				// ひとつ検出するためのブロック
				
				std::vector<param_t> params;
				for(int i=0; i<64; ++i){
					param_t p; p.preset(this->target, this->temp);
					params.push_back(p);
				}
				std::vector<param_t> V;
				for(int i=0; i<params.size()>>1; ++i){ V.push_back(params[0]); }
				std::vector<float> max;
				for(int i=0; i<V.size(); ++i){ max.push_back(0.); }
				int generation = 0;
				bool flag = false;
				int count = 0;
				while(!flag){
					double buf = max[0];
					++generation;
					
					for(int i=0; i<params.size(); ++i){
						// マッチング率の取得
						double parsent = Util::getConcord(this->target, this->temp, params[i]);					
						
						// 適当な位置に挿入
						for(int j=0; j<max.size(); ++j){
							if(max[j] < parsent){
								for(int k=max.size()-1; j<k; --k){
									max[k] = max[k-1];
									V[k] = V[k-1];
								}
								max[j] = parsent;
								V[j] = params[i];
								break;
							}
						}
					}
					std::cout << max[0] << std::endl;
					
					// 終了条件は、最大マッチング率が0.98以上のときか、同じ値が32回続いたとき
					flag |= 0.98 < max[0];
					if(max[0]==buf){
						flag |= ++count>32;
					}else{
						count = 0;
					}

					Util::generate(V, params);
					if(flag){
						std::cout << "generation = " << generation << std::endl;
						std::cout << "angle = " << V[0].angle << std::endl;
						std::cout << "scale = " << V[0].scale/10. << std::endl;
						std::cout << "[x, y] = " << "[" << V[0].x << ", " << V[0].y << "]" << std::endl;
						std::cout << max[0] << std::endl;
						std::cout << std::endl;
	
						Util::show(this->target, this->temp, V);

						// 見つけたところを黒く塗りつぶす
						for(int y=V[0].y; y<V[0].y + this->temp.rows*(V[0].scale/10.); ++y){
							for(int x=V[0].x; x<V[0].x + this->temp.rows*(V[0].scale/10.); ++x){
								cv::Vec3b &ch = this->target.at<cv::Vec3b>(y, x);
								for(int i=0; i<sizeof(ch); ++i){
									ch[i] = 0;
								}
							}
						}
					}
				}
			}
		}
};

int main(int argc, char* argv[])
{
	if(1 < argc){
		for(int i=1; i<argc; ++i){
			Algorithm* me = new Algorithm(argv[i]);
			me->run();
		}
	}else{
		Algorithm* me = new Algorithm("P_easy_top.JPG");
		me->run();
	}
	
	return 0;
};

