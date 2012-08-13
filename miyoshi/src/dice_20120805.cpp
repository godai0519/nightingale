// サイコロ検出アルゴリズム試作
// copyright(c) 2012 Shota Miyoshi

//都合により環境がVisualStudio2008+OpenCV2.1となっておりますが,
//始めの方の#ifdefから#endifまでの,ライブラリファイルの参照部を変えるとOpenCV2.2でも動作します.

#include "stdafx.h"
#include <iostream>
#include <set>
#include <stack>
#include <vector>
#include "opencv\cv.h"
#include "opencv\highgui.h"

#ifdef _DEBUG
    #pragma comment(lib,"C:\\OpenCV2.1\\lib\\cv210d.lib")
    #pragma comment(lib,"C:\\OpenCV2.1\\lib\\cxcore210d.lib")
    #pragma comment(lib,"C:\\OpenCV2.1\\lib\\cvaux210d.lib")
    #pragma comment(lib,"C:\\OpenCV2.1\\lib\\highgui210d.lib")
#else
    #pragma comment(lib,"C:\\OpenCV2.1\\lib\\cv210.lib")
    #pragma comment(lib,"C:\\OpenCV2.1\\lib\\cxcore210.lib")
    #pragma comment(lib,"C:\\OpenCV2.1\\lib\\cvaux210.lib")
    #pragma comment(lib,"C:\\OpenCV2.1\\lib\\highgui210.lib")
#endif



typedef struct Eye//サイコロの目を表す構造体(x,yは中心座標)
{
	int x;
	int y;
	int h;
	int w;
}Eye;


const char* WinConv="Converted Image";
const char* WinDet="Detected Eyes";



//函数プロトタイプ
void convert(const cv::Mat& src,cv::Mat& dst,int thblack,int thwhite);
void label(const cv::Mat& from,cv::Mat& to,int xs,int ys,int R,int G,int B);
void geteye(const cv::Mat& image,int blackmin,int blackmax,int redmin,int redmax);
Eye getsize(const cv::Mat& image,int xs,int ys);


int height,width;

int _tmain(int argc, _TCHAR* argv[])
{
	cv::Mat input,converted;
	int thb=100,thw=200;
	char flag;
	char filename[100];
	//std::cin.getline(filename,100);

	input=cv::imread(filename);
	height=input.rows;
	width=input.cols;

	//入力画像表示
	cv::namedWindow("input_img", CV_WINDOW_NORMAL);
	cv::imshow("input_img",input);
	
	//入力画像4値化
	cv::namedWindow(WinConv, CV_WINDOW_NORMAL);

	//先ずは既定の閾値で変換
	//赤の閾値は今の所省略
	std::cout<<"Threshold of Black: "<<thb<<std::endl;
	std::cout<<"Threshold of White: "<<thw<<std::endl;
	convert(input,converted,thb,thw);
	std::cout<<"\nConvert again? Yes->y,No->n  *Please Activate \"Converted Image\""<<std::endl;
	flag=cv::waitKey();
	std::cout<<flag<<std::endl;

	//閾値をユーザが設定(CUIによる)
	while(flag=='y')
	{
		std::cout<<"Threshold of Black,0-255 : ";
		std::cin>>thb;
		std::cout<<"Threshold of White,0-255 : ";
		std::cin>>thw;
		convert(input,converted,thb,thw);
		std::cout<<"\nConversion Again? Yes->y,No->n  *Please Activate \"Converted Image\""<<std::endl;
		flag=cv::waitKey(0);
		std::cout<<flag<<std::endl;
	}

	//4値画像(converted)から目を探索
	//始めは既定値探索
	int bmin=1,bmax=20,rmin=1,rmax=100;
	cv::namedWindow(WinDet);
	std::cout<<"\n\nMinimum size of Black Eye: "<<bmin<<std::endl;
	std::cout<<"Maximum size of Black Eye: "<<bmax<<std::endl;
	std::cout<<"Minimum size of Red Eye:   "<<rmin<<std::endl;
	std::cout<<"Maximum size of Red Eye:   "<<rmax<<std::endl;
	geteye(converted,bmin,bmax,rmin,rmax);
	std::cout<<"\ndetect Again? Yes->y,No->nv *Please Activate \"Detected Eyes\""<<std::endl;
	flag=cv::waitKey();
	std::cout<<flag<<std::endl;

	//条件をユーザが設定
	//設定可能は赤目,黒目の大きさ下限,上限
	while(flag=='y')
	{
		std::cout<<"Minimum size of Black Eye: ";
		std::cin>>bmin;
		std::cout<<"Maximum size of Black Eye: ";
		std::cin>>bmax;
		std::cout<<"Minimum size of Red Eye:   ";
		std::cin>>rmin;
		std::cout<<"Maximum size of Red Eye:   ";
		std::cin>>rmax;
		geteye(converted,bmin,bmax,rmin,rmax);
		std::cout<<"\ndetect Again? Yes->y,No->n  *Please Activate \"Detected Eyes\""<<std::endl;
		flag=cv::waitKey(0);
		std::cout<<flag<<std::endl;
	}

	cv::waitKey(0);

	return 0;
}

void convert(const cv::Mat& src,cv::Mat& dst,int thblack,int thwhite)//赤,黒,白,その他(灰)の4値化
{
	cv::Mat temp_img= cv::Mat(cv::Size(width,height),CV_8UC3);
	cv::Mat hsv_img;
	
	cv::cvtColor(src,hsv_img, CV_BGR2HSV);//RGB->HSVへ変換 H=hue(色相),S=saturation(彩度),V=value(明度)と言う意味
	temp_img = cv::Scalar(0,0,0);
	for(int y=0; y<src.rows;y++)
	{
		for(int x=0; x<src.cols; x++)
		{
			int a = hsv_img.step*y+(x*3);
			if((hsv_img.data[a] >=170 || hsv_img.data[a] <=10) &&hsv_img.data[a+1] >=50) //赤検出
			{
				temp_img.data[a] = 0;
				temp_img.data[a+1] = 0;
				temp_img.data[a+2] = 255;
			}
			else if(src.data[a]<thblack&&src.data[a+1]<thblack&&src.data[a+2]<thblack)//黒(RGB値が閾値より小)
			{
				temp_img.data[a] = 0;
				temp_img.data[a+1] = 0;
				temp_img.data[a+2] = 0;
			}
			else if(src.data[a]>thwhite&&src.data[a+1]>thwhite&&src.data[a+2]>thwhite)//白(RGB値が閾値より大)
			{
				temp_img.data[a] = 255;
				temp_img.data[a+1] = 255;
				temp_img.data[a+2] = 255;
			}
			else//どれでもない場合は灰
			{
				temp_img.data[a] = 128;
				temp_img.data[a+1] = 128;
				temp_img.data[a+2] = 128;
			}
		}
	}
	dst=temp_img;

	cv::imshow(WinConv,dst);//4値画像表示
}

void geteye(const cv::Mat& image,int blackmin,int blackmax,int redmin,int redmax)//目を検出
{
	cv::Mat detection=cv::Mat::zeros(height,width,CV_8UC3);
	std::vector<Eye> redeye;
	std::vector<Eye> blackeye;
	Eye eye;

	for(int y=0;y<height;y++)
	{
		for(int x=0;x<width;x++)
		{
			if(detection.data[detection.step*y+x*3]==0&&detection.data[detection.step*y+x*3+1]==0&&detection.data[detection.step*y+x*3+2]==0)
			{
				if(image.data[image.step*y+x*3]==0&&image.data[image.step*y+x*3+1]==0&&image.data[image.step*y+x*3+2]==255)//赤
				{
					eye=getsize(image,x,y);
					if(eye.h>=redmin&&eye.w>=redmin&&eye.h<=redmax&&eye.w<=redmax)
					{
						redeye.push_back(eye);
						label(image,detection,x,y,255,0+(eye.h>eye.w?255:0),0);
					}
					else
					{
						label(image,detection,x,y,0,255,0);
					}
				}
				else if(image.data[image.step*y+x*3]==0&&image.data[image.step*y+x*3+1]==0&&image.data[image.step*y+x*3+2]==0)//黒
				{
					eye=getsize(image,x,y);
					if(eye.h>=blackmin&&eye.w>=blackmin&&eye.h<=blackmax&&eye.w<=blackmax)
					{
						blackeye.push_back(eye);
						label(image,detection,x,y,0,0+(eye.h>eye.w?255:0),255);
					}
					else
					{
						label(image,detection,x,y,0,255,0);
					}
				}
			}
		}
	}

	//std::cout<<"Red Eyes:"<<std::endl;
	//for(std::vector<Eye>::iterator i=redeye.begin();i!=redeye.end();i++)
	//{
	//	std::cout<<(*i).x<<' '<<(*i).y<<' '<<(*i).h<<' '<<(*i).w<<std::endl;
	//}

	//std::cout<<"Black Eyes:"<<std::endl;
	//for(std::vector<Eye>::iterator i=blackeye.begin();i!=blackeye.end();i++)
	//{
	//	std::cout<<(*i).x<<' '<<(*i).y<<' '<<(*i).h<<' '<<(*i).w<<std::endl;
	//}

	cv::imshow(WinDet,detection);
}

Eye getsize(const cv::Mat& image,int xs,int ys)//xs,ysを含む領域サイズ取得
{
	std::set<int> visited;
	std::stack<int> stk;

	int xmin=xs,xmax=xs,ymin=ys,ymax=ys;
	int scolor[3];
	Eye eye;

	for(int i=0;i<3;i++){scolor[i]=image.data[image.step*ys+xs*3+i];}

	stk.push(xs+ys*width);
	while(!stk.empty())
	{
		bool flag=false;
		int x=stk.top()%width;
		int y=stk.top()/width;
		stk.pop();
		for(int i=0;i<3;i++){if(image.data[image.step*y+x*3+i]!=scolor[i])flag=true;}
		if(flag)continue;

		visited.insert(x+y*width);
		xmin=xmin>x?x:xmin;
		xmax=xmax<x?x:xmax;
		ymin=ymin>y?y:ymin;
		ymax=ymax<y?y:ymax;

		if(x>0&&visited.find(x-1+y*width)==visited.end())stk.push(x-1+y*width);
		if(x<width-1&&visited.find(x+1+y*width)==visited.end())stk.push(x+1+y*width);
		if(y>0&&visited.find(x+(y-1)*width)==visited.end())stk.push(x+(y-1)*width);
		if(y<height-1&&visited.find(x+(y+1)*width)==visited.end())stk.push(x+(y+1)*width);
	}
	eye.x=xmin+(xmax-xmin)/2;
	eye.y=ymin+(ymax-ymin)/2;
	eye.h=ymax-ymin+1;
	eye.w=xmax-xmin+1;
	return eye;
}

void label(const cv::Mat& from,cv::Mat& to,int xs,int ys,int R,int G,int B)//xs,ysと同じ色の領域をR,G,Bで指定される色で塗りつぶす
{
	std::stack<int> stk;
	std::set<int> visited;
	int scolor[3];

	for(int i=0;i<3;i++){scolor[i]=from.data[from.step*ys+xs*3+i];}

	stk.push(xs+ys*width);
	while(!stk.empty())
	{
		bool flag=false;
		int x=stk.top()%width;
		int y=stk.top()/width;
		stk.pop();
		for(int i=0;i<3;i++){if(from.data[from.step*y+x*3+i]!=scolor[i])flag=true;}
		if(flag)continue;

		to.data[to.step*y+x*3]=B;
		to.data[to.step*y+x*3+1]=G;
		to.data[to.step*y+x*3+2]=R;

		visited.insert(x+y*width);
		if(x>0&&visited.find(x-1+y*width)==visited.end())stk.push(x-1+y*width);
		if(x<width-1&&visited.find(x+1+y*width)==visited.end())stk.push(x+1+y*width);
		if(y>0&&visited.find(x+(y-1)*width)==visited.end())stk.push(x+(y-1)*width);
		if(y<height-1&&visited.find(x+(y+1)*width)==visited.end())stk.push(x+(y+1)*width);
	}
}
