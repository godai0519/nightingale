// サイコロ検出アルゴリズム(ほぼ)完全版

#include "stdafx.h"
#include <iostream>
#include <set>
#include <stack>
#include <vector>
#include <algorithm>
#include "opencv\cv.h"
#include "opencv\highgui.h"

#ifdef _DEBUG
    #pragma comment(lib,"C:\\OpenCV2.4.2\\opencv\\build\\x86\\vc9\\lib\\opencv_core242d.lib")
    #pragma comment(lib,"C:\\OpenCV2.4.2\\opencv\\build\\x86\\vc9\\lib\\opencv_imgproc242d.lib")
    #pragma comment(lib,"C:\\OpenCV2.4.2\\opencv\\build\\x86\\vc9\\lib\\opencv_highgui242d.lib")
#else
    #pragma comment(lib,"C:\\OpenCV2.4.2\\opencv\\build\\x86\\vc9\\lib\\opencv_core242.lib")
    #pragma comment(lib,"C:\\OpenCV2.4.2\\opencv\\build\\x86\\vc9\\lib\\opencv_imgproc242.lib")
    #pragma comment(lib,"C:\\OpenCV2.4.2\\opencv\\build\\x86\\vc9\\lib\\opencv_highgui242.lib")
#endif

typedef struct Dice
{
	int h;
	int w;
	int eyesize;
	int value;
	double aspect;
	int compsize;
	Dice();
	Dice(int h,int w,int eyesize,int value,double aspect,int compsize);
}Dice;

Dice::Dice()
{
	return;
}

Dice::Dice(int h, int w, int eyesize, int value, double aspect,int compsize)
{
	this->h=h;
	this->w=w;
	this->eyesize=eyesize;
	this->value=value;
	this->aspect=aspect;
	this->compsize=compsize;
}

typedef struct TempEye
{
	int location;
	int height;
	int width;
	double degree;
	TempEye(int location,int height,int width,double degree);

}TempEye;

TempEye::TempEye(int location,int height,int width,double degree)
{
	this->location=location;
	this->height=height;
	this->width=width;
	this->degree=degree;

}

//窓名(兼キャプション)
const char* WINDOW_EYE="Detected Eyes: Wants to detect again, then Press \'y\', else Press \'n\'";
const char* WINDOW_DICE="Detected Dice: Wants to detect again, then Press \'y\',else Press \'n\'";
const char* WINDOW_EXCLUDE="Exclude Error. Drag and Drop around the center of Wrong Eye";
const char* WINDOW_SIZE="Select size: Enter the size of selected Dice,\'l\', \'m\', or \'s\'";
const char* WINDOW_COUNT="Counted Dice: Wants to add dice, Select dice then Press the size, to finish, Press \"Enter\"";
const char* WINDOW_CROP="Crop the image: Drag and Drop, Press \"Enter\" to Finish";

//函数プロトタイプ達
void red_area(cv::Mat& dst,const cv::Mat& image);
void black_area(cv::Mat& dst,const cv::Mat& image);
void onMouse_crop(int event,int x,int y,int flag,void* param);
void crop(const cv::Mat& src,cv::Mat& dst);
void sharpen(const cv::Mat src, cv::Mat& dst, float k = 2.0f);
void gamma(const cv::Mat src, cv::Mat& dst, double gamma = 2.0);
void normalise(const cv::Mat src, cv::Mat& dst);
void ioGeteye(const cv::Mat& image,const cv::Mat& black,const cv::Mat& red,std::map<int,cv::Size>& blackeye,std::map<int,cv::Size>& redeye,bool first);
void geteye(const cv::Mat& src,cv::Mat& dst,std::map<int,cv::Size>& eyes,std::vector<TempEye>& tempeyes,bool red);
void geteye_second(cv::Mat& dst,std::map<int,cv::Size>& eyes,const std::vector<TempEye>& tempeyes,int min,int max,int degree,bool red);
std::pair<int,cv::Size> getsize(const cv::Mat& image,int xs,int ys,int& area,int& perimeter,std::set<int>& visited);
void ioGetdice(const cv::Mat& image,std::map<int,Dice>& top,std::map<int,Dice>& side,std::map<int,cv::Size>& black,std::map<int,cv::Size>& red,bool first);
void getdice(const cv::Mat& image,std::map<int,Dice>& top,std::map<int,Dice>& side,std::map<int,cv::Size>& eyes,double dist,double r_h,double r_aspect);
bool near_ellipse(int p,int p0,double height,double width,int cols);
void redeye2dice(std::map<int,Dice>& top,std::map<int,Dice>& side,std::map<int,cv::Size>& redeye);
void showdice(const char* winname,const cv::Mat* image,std::map<int, Dice>* top,std::map<int,Dice>* side);
void onMouse_exclude(int event,int x,int y,int flag,void* param);
void CountDice(const cv::Mat& image,std::map<int,Dice>& top,std::map<int,Dice>& side,int& s,int& m,int& l);
void count(const cv::Mat& image,cv::Mat& counted_img,std::multimap<char,int>& dicesize,const std::map<int,Dice>& dice,int top);
void sift(const std::map<int,Dice>& src,std::map<int,Dice>& dst,const std::map<int,Dice>& top,int cols);
double getperimeter(const cv::Mat& image,int x0,int y0);
void onMouse_count(int event,int x,int y,int flag,void* param);
void remove(std::map<int,Dice>& top,std::map<int,Dice>& side,int cols);


///////////////////////////////////////////////////////////////////////////////

//座標変換の小道具達
inline int xy2p(int x,int y,int cols)
{
	return x+y*cols;
}
inline int cvp2p(cv::Point p,int cols)
{
	return p.x+p.y*cols;
}
inline cv::Point p2cvp(int p,int cols)
{
	return cv::Point(p%cols,p/cols);
}
//距離判定の小道具
inline bool distant(int p1,int p2,double d,int cols)
{
	return hypot(p2%cols-p1%cols,p1/cols-p2/cols)>d;
}

//メイン
int _tmain(int argc, _TCHAR* argv[])
{
	cv::Mat input,black,red;
	std::map<int,cv::Size> blackeye,redeye;
	std::map<int,Dice> side;
	std::map<int,Dice> top;
	std::string filename;
	int s=0,m=0,l=0;
	bool first=true;
	while(1)
	{
		int i=1;

		//画像読込
		if(argc>i)
		{
			input=cv::imread(argv[i]);
		}
		else
		{
			std::cout<<"Enter Filename. in case of the End, Press Enter Key without input."<<std::endl;
			std::getline(std::cin,filename);
			if(filename.size()==0)break;
			input=cv::imread(filename);
		}

		crop(input,input);

		//目を抽出(東様より引用させて頂きました)
		black_area(black,input);
		red_area(red,input);

		ioGeteye(input,black,red,blackeye,redeye,first);
		ioGetdice(input,top,side,blackeye,redeye,first);
		if(!first){remove(top,side,input.cols);}
		CountDice(input,top,side,s,m,l);
		cv::destroyAllWindows();
		if(first)first=false;
	}
	cv::destroyAllWindows();
	std::cout<<"\n\nNumber of Dice: S:"<<s<<" M:"<<m<<" L:"<<l<<std::endl;
	std::cin.get();
	return 0;
}

//探索領域を絞るマウスI/O
void onMouse_crop(int event,int x,int y,int flag,void* param)
{
	static int x1,y1,x2,y2;

	if(event == CV_EVENT_LBUTTONDOWN)
	{
		x1=x;y1=y;
	}
	else if(event == CV_EVENT_LBUTTONUP)
	{
		x2=x;y2=y;

		std::pair<const cv::Mat*,cv::Rect*>* data=reinterpret_cast<std::pair<const cv::Mat*,cv::Rect*>*>(param);
		cv::Rect* size=data->second;
		cv::Mat image=data->first->clone();
		size->x=x1<x2?x1:x2;
		size->y=y1<y2?y1:y2;
		size->width=abs(x1-x2);
		size->height=abs(y1-y2);
		cv::rectangle(image,*size,cv::Scalar(0,0,0),1,CV_AA);
		cv::imshow(WINDOW_CROP,image);
	}
}

//画像切り抜き
void crop(const cv::Mat& src,cv::Mat& dst)
{
	std::pair<const cv::Mat*,cv::Rect*> senddata;
	senddata.first=&src;
	senddata.second=new cv::Rect(0,0,src.cols,src.rows);
	cv::namedWindow(WINDOW_CROP,CV_WINDOW_NORMAL);
	cv::imshow(WINDOW_CROP,src);
	cv::setMouseCallback(WINDOW_CROP,onMouse_crop,&senddata);
	while(cv::waitKey()=='\n')
	cv::destroyWindow(WINDOW_CROP);
	cv::Mat cropped(src,*senddata.second);
	dst=cropped.clone();
	cv::destroyWindow(WINDOW_CROP);
}

//尖鋭化
void sharpen(const cv::Mat src, cv::Mat& dst, float k)
{
	float kernelData[] = {
		-k/9.0f, -k/9.0f, -k/9.0f,
		-k/9.0f, 1 + (8*k)/9.0f, -k/9.0f,
		-k/9.0f, -k/9.0f, -k/9.0f,
	};
	const cv::Mat kernel(3, 3, CV_32F, kernelData);
	cv::filter2D(src, dst, -1, kernel);
}

//ガンマ補正
void gamma(const cv::Mat src, cv::Mat& dst, double gamma)
{
	static bool f = true;
	int i;   
	static unsigned char LUT[256];   
	
	if(f){
		f = false;         
		//ガンマ補正テーブルの作成   
		for (i = 0; i < 256; i++){   
			LUT[i] = (int)(pow((double)i / 255.0, 1.0 / gamma) * 255.0);   
		}
	}
	for(int y=0; y<src.rows; ++y){
		for(int x=0; x<src.cols; ++x){
			const cv::Vec3b p = src.at<cv::Vec3b>(y,x);
			dst.at<cv::Vec3b>(y,x) = cv::Vec3b(LUT[p[0]], LUT[p[1]], LUT[p[2]]);
		}
	}
}

//正規化
void normalise(const cv::Mat src, cv::Mat& dst)
{
	double max = 0., min = 0.;
	cv::minMaxLoc(src, &min, &max);
	cv::convertScaleAbs(src, dst, 255.0/(max-min), (255.0/(max-min))*(-min));
}


//赤いエリアを抜き出す関数
void red_area(cv::Mat& dst,const cv::Mat& image)
{
  //HSVの要素分解
	cv::Mat hsv;
	cv::cvtColor(image,hsv,CV_BGR2HSV);
	cv::Mat channels[3],h_mask,h_mask_low,h_mask_high,c_mask;
	cv::split(hsv,channels);

	cv::threshold(channels[0],h_mask_low,10,255,cv::THRESH_BINARY); //色相が赤付近かどうか
	cv::threshold(channels[0],h_mask_high,170,255,cv::THRESH_BINARY); //色相が赤付近かどうか
	cv::threshold(channels[1],c_mask,100,255,cv::THRESH_BINARY); //彩度(信用性)が高いかどうか

	cv::bitwise_not(h_mask_low,h_mask_low);
	cv::bitwise_or(h_mask_low,h_mask_high,h_mask);

	//色相が赤付近で，彩度が高い部分(->dst)
	cv::bitwise_and(h_mask,c_mask,dst);

	return;
}

//黒いエリアを抜き出す
void black_area(cv::Mat& dst,const cv::Mat& image)
{
	//RGBに分割

	cv::Mat channels[3],r,g,b;
	cv::split(image,channels);
	
	//(各要素で)黒くないとこ
	cv::threshold(channels[0],r,100,255,cv::THRESH_BINARY);
	cv::threshold(channels[1],g,100,255,cv::THRESH_BINARY);
	cv::threshold(channels[2],b,100,255,cv::THRESH_BINARY);

	//(各要素で)反転させて黒いとこ
	bitwise_not(r,r);
	bitwise_not(g,g);
	bitwise_not(b,b);

	//合体(->dst)
	cv::bitwise_and(r,g,dst);
	cv::bitwise_and(b,dst,dst);
}


//目を検知する際のI/O処理
void ioGeteye(const cv::Mat& image, const cv::Mat& black,const cv::Mat& red,std::map<int,cv::Size>& blackeye,std::map<int,cv::Size>& redeye,bool first)
{
	//始めは既定値探索
	cv::Mat dst=cv::Mat::zeros(black.rows,black.cols,CV_8UC3);
	std::vector<TempEye> tempblack,tempred;
	char flag;
	static int bmin=black.rows/200,bmax=black.rows/20,rmin=red.rows/200,rmax=red.rows/10,deg=50;//デフォルト閾値

	geteye(black,dst,blackeye,tempblack,false);
	geteye(red,dst,redeye,tempred,true);

	//一枚目の探索
	if(first)
	{
		cv::namedWindow(WINDOW_EYE,CV_WINDOW_NORMAL);

		//閾値設定用のトラックバー達を設置
		cv::createTrackbar("Black Min",WINDOW_EYE,&bmin,black.cols/10);
		cv::createTrackbar("Black Max",WINDOW_EYE,&bmax,black.cols/10);
		cv::createTrackbar("Red Min",WINDOW_EYE,&rmin,red.cols/10);
		cv::createTrackbar("Red Max",WINDOW_EYE,&rmax,red.cols/10);
		cv::createTrackbar("Roundness",WINDOW_EYE,&deg,100);

		//条件をユーザが設定
		//設定可能は赤目,黒目の大きさ下限,上限
		do
		{
			dst=cv::Mat::zeros(black.rows,black.cols,CV_8UC3);
			geteye_second(dst,blackeye,tempblack,bmin,bmax,deg,false);
			geteye_second(dst,redeye,tempred,rmin,rmax,deg,true);
			cv::imshow(WINDOW_EYE,dst);
			flag=cv::waitKey();
		}while(flag=='y');
		cv::destroyWindow(WINDOW_EYE);
	}
	//二枚目以降は一枚目と同じ閾値(撮影条件はほとんど変わらない...筈)
	else
	{
		geteye_second(dst,blackeye,tempblack,bmin,bmax,deg,false);
		geteye_second(dst,redeye,tempred,rmin,rmax,deg,true);
	}
}

//目を抽出
void geteye(const cv::Mat& src,cv::Mat& dst,std::map<int,cv::Size>& eyes,std::vector<TempEye>& tempeyes,bool red)
{
	//此処でのsrcは白黒の二値である事に注意
	std::set<int> visited;
	eyes.clear();

	for(int ys=0;ys<src.rows;ys++)
	{
		for(int xs=0;xs<src.cols;xs++)
		{
			if(src.data[src.step*ys+xs]==0||visited.find(xy2p(xs,ys,src.cols))!=visited.end())continue;

			int s=0;
			int xmin=xs,xmax=xs,ymin=ys,ymax=ys;
			std::pair<int,cv::Size> eye;
			std::set<int> region;
			std::stack<int> stk;
			stk.push(xy2p(xs,ys,src.cols));

			//再帰じゃないけどdfs(depth first search)です
			//dfsで塗りつぶす事で領域を決定
			while(!stk.empty())
			{
				bool flag=false;
				int x=stk.top()%src.cols;
				int y=stk.top()/src.cols;
				stk.pop();
				if(region.find(xy2p(x,y,src.cols))!=region.end()||src.data[src.step*y+x]==0)continue;

				region.insert(xy2p(x,y,src.cols));
				xmin=xmin>x?x:xmin;
				xmax=xmax<x?x:xmax;
				ymin=ymin>y?y:ymin;
				ymax=ymax<y?y:ymax;

				if(x>0){stk.push(xy2p(x-1,y,src.cols));}
				if(x<src.cols-1){stk.push(xy2p(x+1,y,src.cols));}
				if(y>0){stk.push(xy2p(x,y-1,src.cols));}
				if(y<src.rows-1){stk.push(xy2p(x,y+1,src.cols));}
				s++;
			}

			//一回目は全て目として処理
			//円形度=4πs/l^2
			int h=ymax-ymin+1,w=xmax-xmin+1;
			double l=getperimeter(src,xs,ys);
			double d=12.566370614359173*s/(l*l);
			eye.first=(xmin+xmax)/2+(ymin+ymax)/2*src.cols;//目の中心座標
			eye.second.height=h;
			eye.second.width=w;

			//塗る色設定&目の集合に挿入
			//赤目横長->赤,赤目縦長->黄,->黒目横長->青,黒目縦長->青緑
			int color[3];
			eyes.insert(eye);
			color[0]=red?0:255;color[1]=eye.second.height>eye.second.width?255:0;color[2]=red?255:0;

			for(std::set<int>::iterator it=region.begin();it!=region.end();it++)
			{
				for(int i=0;i<3;i++)
				{
					dst.data[dst.step*((*it)/src.cols)+(*it)%src.cols*3+i]=color[i];
				}
			}
			tempeyes.push_back(TempEye(eye.first,h,w,d));
			visited.insert(region.begin(),region.end());
		}
	}
}

//2回目以降の目の探索
void geteye_second(cv::Mat& dst,std::map<int,cv::Size>& eyes,const std::vector<TempEye>& tempeyes,int min,int max,int degree,bool red)
{
	//1回目に得た目の情報を使って高速化を図る
	eyes.clear();
	for(int i=0;i<tempeyes.size();i++)
	{
		if(tempeyes[i].height>=min&&tempeyes[i].height<=max&&tempeyes[i].width>=min&&tempeyes[i].width<=max&&tempeyes[i].degree>=(double)degree/100)
		{
			eyes.insert(std::pair<int,cv::Size>::pair(tempeyes[i].location,cv::Size(tempeyes[i].width,tempeyes[i].height)));
			cv::ellipse(dst,p2cvp(tempeyes[i].location,dst.cols),cv::Size(tempeyes[i].width/2,tempeyes[i].height/2),0,0,360,cv::Scalar(red?0:255,tempeyes[i].height>tempeyes[i].width?255:0,red?255:0),-1);
		}
	}
}

//周長を求める
double getperimeter(const cv::Mat& image,int x0,int y0)
{
	cv::Mat temp=image.clone();
	const int move[8][2]={{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1},{0,1},{1,1}};
	int scolor[3];
	for(int i=0;i<3;i++){scolor[i]=255;}//image.data[image.step*y0+x0*3+i];}
	int d=3;
	int x=x0,y=y0;
	int x1=-1,y1=-1;
	bool end=false;
	int perpendicular=0,diagonal=0;

	while(!end)
	{
		bool equal;

		temp.data[temp.step*y+x]=128;

		for(int i=0;i<8;i++)
		{
			//反時計回りに走査
			equal=true;
			d=(d+1)%8;
			//同じ色の画素か,範囲外でないか?
			if(x+move[d][0]<0||x+move[d][0]>=image.cols||y+move[d][1]<0||y+move[d][1]>=image.rows||image.data[image.step*(y+move[d][1])+x+move[d][0]]==0){equal=false;}

			//移動(同じ色の画素を初めて発見したら)
			if(equal)
			{
				int xb=x,yb=y;//移動前のx,y仮置場
				x+=move[d][0];//移動
				y+=move[d][1];

				
				if(x1==-1&&y1==-1){x1=x;y1=y;}
				else if(x==x1&&y==y1&&xb==x0&&yb==y0){end=true;}
				d=(d+4)%8;
				if(!end)
				{
					if(d%2==1){diagonal++;}
					else{perpendicular++;}
				}
				break;
			}
		}
		if(!equal){perpendicular=1;end=true;}
	}
	
	//周長=斜め*2^0.5+4近傍隣接*1
	return diagonal*1.41421356237309505+perpendicular;
}


//目からサイコロを検出するI/O
void ioGetdice(const cv::Mat& image,std::map<int,Dice>& top,std::map<int,Dice>& side,std::map<int,cv::Size>& black,std::map<int,cv::Size>& red,bool first)
{
	char flag;
	static int dist=40,r_h=12,r_aspect=12;
	
	//一回目
	if(first)
	{
		cv::namedWindow(WINDOW_DICE,CV_WINDOW_NORMAL);
		cv::createTrackbar("Distance(*10)",WINDOW_DICE,&dist,80);
		cv::createTrackbar("Height(*10)",WINDOW_DICE,&r_h,30);
		cv::createTrackbar("Aspect(*10)",WINDOW_DICE,&r_aspect,30);

		//条件をユーザが設定
		//設定可能は目の間の距離,高さ,縦横比の,基準となる目との比
		do
		{
			getdice(image,top,side,black,(double)dist/10,(double)r_h/10,(double)r_aspect/10);
			flag=cv::waitKey();
		}while(flag=='y');
		cv::destroyWindow(WINDOW_DICE);
	}
	//二回目以降は一回目と同じ閾値
	else
	{
		getdice(image,top,side,black,(double)dist/10,(double)r_h/10,(double)r_aspect/10);
	}

	//共通(赤目追加と誤検出排除)
	redeye2dice(top,side,red);
	cv::namedWindow(WINDOW_EXCLUDE,CV_WINDOW_NORMAL);
	showdice(WINDOW_EXCLUDE,&image,&top,&side);

	std::pair<const cv::Mat*,std::pair<std::map<int,Dice>*,std::map<int,Dice>*>*> senddata;
	senddata.first=&image;
	senddata.second=&std::pair<std::map<int,Dice>*,std::map<int,Dice>*>::pair(&top,&side);

	cv::setMouseCallback(WINDOW_EXCLUDE,onMouse_exclude,&senddata);
	cv::waitKey();

}

//楕円範囲での近傍判定
bool near_ellipse(int p,int p0,double height,double width,int cols)
{
	int dx=p%cols-p0%cols,dy=p/cols-p0/cols;
	return hypot(height*dx,width*dy)<=height*width;
}

//サイコロ検出本処理
void getdice(const cv::Mat& image,std::map<int,Dice>& top,std::map<int,Dice>& side,std::map<int,cv::Size>& eyes,double dist,double r_h,double r_aspect)
{
	//同じ位の大きさで座標が近い目を一つの面にまとめる
	cv::Mat temp_img=image.clone();
	std::set<int> visited;

	top.clear();side.clear();
	for(std::map<int,cv::Size>::iterator ibase=eyes.begin();ibase!=eyes.end();ibase++)
	{
		std::vector<std::pair<int,cv::Size> > temp;//サイコロの目集合
		std::vector<cv::Point> coordinate;//サイコロの目座標
		if(visited.find((*ibase).first)==visited.end())
		{
			int h=(*ibase).second.height,w=(*ibase).second.width;
			int k=0;
			visited.insert((*ibase).first);
			temp.push_back(*ibase);
			coordinate.push_back(p2cvp((*ibase).first,temp_img.cols));

			//近傍判定(他の目が,基準目の縦,横のdist倍した楕円内にあるか?)

			for(std::map<int,cv::Size>::iterator ite=eyes.begin();ite!=eyes.end();ite++)
			{
				if(near_ellipse((*ite).first,(*ibase).first,h*dist,w*dist,image.cols))
				{
					//大きさ判定(高さの比がr_h倍以内･縦横比r_aspect倍以内,目が小さいときは補正を)
					double rh=(*ibase).second.height<image.cols/160?r_h*2:r_h;
					double raspect=(*ibase).second.height<image.cols/200?r_aspect*2:r_aspect;

					int h1=h,h2=(*ite).second.height;
					int w1=w,w2=(*ite).second.width;
					double a1=(double)h/w,a2=(double)(*ite).second.height/(*ite).second.width;
					if(h1<h2)std::swap(h1,h2);
					if(w1<w2)std::swap(w1,w2);

					if((double)h1/h2<=rh && (double)w1/w2<=raspect)
					{
						temp.push_back((*ite));
						coordinate.push_back(p2cvp(((*ite).first),image.cols));
						visited.insert((*ite).first);
					}
				}
			}
		}

		if(temp.size()>1)
		{
			cv::RotatedRect box=cv::minAreaRect(cv::Mat(coordinate));
			cv::Point2f vtx[4];
			int xmin=temp_img.cols,xmax=0,ymin=temp_img.rows,ymax=0,isize=0,avh=0,avw=0;
			double aspect;

			box.points(vtx);

			for(int i=0;i<4;i++)
			{
				xmin=xmin<vtx[i].x?xmin:vtx[i].x;
				xmax=xmax>vtx[i].x?xmax:vtx[i].x;
				ymin=ymin<vtx[i].y?ymin:vtx[i].y;
				ymax=ymax>vtx[i].y?ymax:vtx[i].y;
			}
			
			
			//目を描画(横長->緑,縦長->黄)
			for(std::vector<std::pair<int,cv::Size>>::iterator iteye=temp.begin();iteye!=temp.end();iteye++)
			{
				cv::Size size=cv::Size2f((*iteye).second.width/2.0,(*iteye).second.height/2.0);
				cv::ellipse(temp_img,p2cvp((*iteye).first,temp_img.cols),size,0,0,360,cv::Scalar(0,255,(*iteye).second.height>(*iteye).second.width?255:0),-1);
				avh+=(*iteye).second.height;
				avw+=(*iteye).second.width;
			}
			//目のサイズは高さ,幅の内長い方
			//上面では,高さは位置による変化が大きいが幅は変化が小さい
			//側面ではその逆
			avh/=temp.size();
			avw/=temp.size();
			isize=(avh>avw?avh:avw)/2/temp.size();
			aspect=(double)avh/avw;

			int h=ymax-ymin+avh*2,w=xmax-xmin+avw*2;

			if(aspect<=0.8)
			{
				top.insert(std::map<int,Dice>::value_type(cvp2p(box.center,temp_img.cols),Dice(h,w,isize,temp.size(),aspect,w)));
				cv::ellipse(temp_img,box.center,cv::Size(w/2,h/2),0,0,360,cv::Scalar(0,0,255),1,CV_AA);
			}
			else
			{
				side.insert(std::map<int,Dice>::value_type(cvp2p(box.center,temp_img.cols),Dice(h,w,isize,temp.size(),aspect,h)));
				cv::ellipse(temp_img,box.center,cv::Size(w/2,h/2),0,0,360,cv::Scalar(0,255,0),1,CV_AA);
			}
		}
	}
	cv::imshow(WINDOW_DICE,temp_img);
}

//赤目もdiceに入れる
void redeye2dice(std::map<int,Dice>& top,std::map<int,Dice>& side,std::map<int,cv::Size>& redeye)
{
	for(std::map<int,cv::Size>::iterator it=redeye.begin();it!=redeye.end();it++)
	{
		//1の面の大きさは目の大きさの2.5倍(実測)
		Dice d((*it).second.height*5/2,(*it).second.width*5/2,((*it).second.height+(*it).second.width)/2,1,(double)(*it).second.height/(*it).second.width,std::max((*it).second.height,(*it).second.width)*5/2);
		if(d.aspect<=1){top.insert(std::map<int,Dice>::value_type((*it).first,d));}
		else{side.insert(std::map<int,Dice>::value_type((*it).first,d));}
	}
}

//上に側面がない上面並びにその直ぐ下の側面を削除->上に側面のある上面,側面は残る
void remove(std::map<int,Dice>& top,std::map<int,Dice>& side,int cols)
{
	std::map<int,Dice> dst_top,dst_side;
	double d=2;//目の離れ度合い(目の高さ倍数)
	for(std::map<int,Dice>::iterator ittop=top.begin();ittop!=top.end();ittop++)
	{
		for(std::map<int,Dice>::iterator itcomp=side.begin();itcomp!=side.end();itcomp++)
		{
			if(!distant((*ittop).first,(*itcomp).first,d*(*ittop).second.compsize,cols)&&((*ittop).first/cols-(*itcomp).first/cols)>(*ittop).second.h/2)
			{
				dst_top.insert(*ittop);
			}
		}
	}
	for(std::map<int,Dice>::iterator itside=side.begin();itside!=side.end();itside++)
	{
		for(std::map<int,Dice>::iterator itcomp=side.begin();itcomp!=side.end();itcomp++)
		{
			if(!distant((*itside).first,(*itcomp).first,d*(*itside).second.compsize,cols)&&((*itside).first/cols-(*itcomp).first/cols)>(*itside).second.h/2)
			{
				dst_side.insert(*itside);
			}
		}
	}
	top=dst_top;
	side=dst_side;
}

//サイコロ全て表示
void showdice(const char* winname,const cv::Mat* image,std::map<int,Dice>* top,std::map<int,Dice>* side)
{
	cv::Mat temp=image->clone();
	for(std::map<int,Dice>::iterator it=top->begin();it!=top->end();it++)
	{
		cv::ellipse(temp,p2cvp((*it).first,image->cols),cv::Size((*it).second.w/2,(*it).second.h/2),0,0,360,cv::Scalar(0,0,255),1,CV_AA);
	}
	for(std::map<int,Dice>::iterator it=side->begin();it!=side->end();it++)
	{
		cv::ellipse(temp,p2cvp((*it).first,image->cols),cv::Size((*it).second.w/2,(*it).second.h/2),0,0,360,cv::Scalar(0,255,0),1,CV_AA);
	}
	cv::imshow(winname,temp);
}

//誤検出排除のマウスI/O paramはmat+(topとsideのpair)
void onMouse_exclude(int event,int x,int y,int flag,void* param)
{
	static int xmin,ymin,xmax,ymax;

	if(event == CV_EVENT_LBUTTONDOWN)
	{
		xmin=x;ymin=y;
	}
	else if(event == CV_EVENT_LBUTTONUP)
	{
		xmax=x;ymax=y;

		std::pair<const cv::Mat*,std::pair<std::map<int,Dice>*,std::map<int,Dice>*>*>* data=reinterpret_cast<std::pair<const cv::Mat*,std::pair<std::map<int,Dice>*,std::map<int,Dice>*>*>*>(param);
		const cv::Mat* image=data->first;
		std::map<int,Dice>* top=data->second->first;
		std::map<int,Dice>* side=data->second->second;

		xmin=xmin>0?xmin:0;
		xmax=xmax<image->cols?xmax:image->cols;
		ymin=ymin>0?ymin:0;
		ymax=ymax<image->rows?ymax:image->rows;
		if(xmax<xmin)std::swap(xmax,xmin);
		if(ymax<ymin)std::swap(ymax,ymin);

		for(int y=ymin;y<=ymax;y++)
		{
			for(int x=xmin;x<=xmax;x++)
			{
				if(top->find(xy2p(x,y,image->cols))!=top->end())
				{
					top->erase(xy2p(x,y,image->cols));
				}
				if(side->find(xy2p(x,y,image->cols))!=side->end())
				{
					side->erase(xy2p(x,y,image->cols));
				}
			}
		}
		showdice(WINDOW_EXCLUDE,image,top,side);
	}
}

//サイコロ数え上げI/O
void CountDice(const cv::Mat& image,std::map<int,Dice>& top,std::map<int,Dice>& side,int& s,int& m,int& l)
{
	std::multimap<char,int> dicesize;//サイズ別のサイコロを此処に入れる
	cv::Mat counted_img=image.clone();

	//自動判定
	cv::namedWindow(WINDOW_SIZE,CV_WINDOW_NORMAL);
	count(image,counted_img,dicesize,top,255);
	std::map<int,Dice> onlyside;
	sift(side,onlyside,top,image.cols);
	count(image,counted_img,dicesize,onlyside,0);
	cv::destroyWindow(WINDOW_SIZE);

	//人力判別(最終手段)
	cv::namedWindow(WINDOW_COUNT,CV_WINDOW_NORMAL);
	cv::imshow(WINDOW_COUNT,counted_img);
	std::pair<cv::Mat*,std::multimap<char,int>*> data=std::pair<cv::Mat*,std::multimap<char,int>*>(&counted_img,&dicesize);
	cv::setMouseCallback(WINDOW_COUNT,onMouse_count,&data);

	while(cv::waitKey()!='n');//'Enter'キーで入力終了
	
	s+=dicesize.count('s');
	m+=dicesize.count('m');
	l+=dicesize.count('l');
}

//サイコロ数え上げ本ルーチン
void count(const cv::Mat& image,cv::Mat& counted_img,std::multimap<char,int>& dicesize,const std::map<int,Dice>& dice,int top)
{
	//観測帯を分けるのは廃止しました.
	//長望遠レンズ(私の奴は270mm相当)で撮影すると遠近感が失われ,同じ大きさに見えます.
	//自画自賛ですが写真部ならではの知識?そんなの見れば解る?
	cv::Mat temp_img;
	int maxsize=0;
	char basesize;
	bool found=false;
	std::map<int,Dice>::const_iterator pos;

	for(std::map<int,Dice>::const_iterator it=dice.begin();it!=dice.end();it++)
	{
		if((*it).second.compsize>maxsize)
		{
			pos=it;
			maxsize=(*it).second.compsize;
		}
	}
	//基準のサイズを選ばせる(何が何でもLサイズだと,Mサイズ以下しかないとき困る)
	temp_img=image.clone();
	cv::ellipse(temp_img,p2cvp((*pos).first,image.cols),cv::Size((*pos).second.w/2,(*pos).second.h/2),0,0,360,cv::Scalar(0,0,255),3,4);
	cv::imshow(WINDOW_SIZE,temp_img);
	basesize=cv::waitKey();

	//大きさ判定
	for(std::map<int,Dice>::const_iterator it=dice.begin();it!=dice.end();it++)
	{
		char size='\0';
		switch(basesize)
		{
		case 'l':
			if((*it).second.compsize>=(*pos).second.compsize*0.7)
			{
				size='l';
			}
			else if((*it).second.compsize>=(*pos).second.compsize*0.4)
			{
				size='m';
			}
			else if((*it).second.compsize>=(*pos).second.compsize*0.2)
			{
				size='s';
			}
			break;
		case 'm':
			if((*it).second.compsize>=(*pos).second.compsize*0.7)
			{
				size='m';
			}
			else if((*it).second.compsize>=(*pos).second.compsize*0.4)
			{
				size='s';
			}
			break;
		case 's':
			if((*it).second.compsize>=(*pos).second.compsize*0.7)
			{
				size='s';
			}
			break;
		}
		dicesize.insert(std::multimap<char,int>::value_type(size,(*it).first));
		cv::Scalar color=cv::Scalar(0,0,0);
		switch(size)
		{
		case 'l':
			color=cv::Scalar(top,0,255);
			break;
		case 'm':
			color=cv::Scalar(0,255,top);
			break;
		case 's':
			color=cv::Scalar(255,top,0);
			break;
		}
		cv::ellipse(counted_img,p2cvp((*it).first,image.cols),cv::Size((*it).second.w/2,(*it).second.h/2),0,0,360,color,1,CV_AA);
	}
}

//上方に上面がある側面を篩い落とす
void sift(const std::map<int,Dice>& src,std::map<int,Dice>& dst,const std::map<int,Dice>& top,int cols)
{
	dst.clear();
	double r=3;
	for(std::map<int,Dice>::const_iterator it=src.begin();it!=src.end();it++)
	{
		for(std::map<int,Dice>::const_iterator ittop=top.begin();ittop!=top.end();ittop++)
		{
			if(distant((*it).first,(*ittop).first,r*(*it).second.eyesize,cols))
			{
				dst.insert(*it);
			}
		}
	}
}

//大きさ判別を人が行う為のマウスコールバック函数
//ドラッグ&ドロップで面の範囲を決め(実はあまり重要ではない),大きさを選択(s,m,l)
//最悪これがあれば人力で行ける!?
void onMouse_count(int event,int x,int y,int flag,void* param)
{
	static int xmin,ymin,xmax,ymax;

	if(event == CV_EVENT_LBUTTONDOWN)
	{
		xmin=x;ymin=y;
	}
	else if(event == CV_EVENT_LBUTTONUP)
	{
		xmax=x;ymax=y;

		std::pair<cv::Mat*,std::multimap<char,int>*>* data=reinterpret_cast<std::pair<cv::Mat*,std::multimap<char,int>*>*>(param);
		cv::Mat *image=data->first;
		std::multimap<char,int>* dicesize=data->second;

		xmin=xmin>0?xmin:0;
		xmax=xmax<image->cols?xmax:image->cols;
		ymin=ymin>0?ymin:0;
		ymax=ymax<image->rows?ymax:image->rows;
		if(xmax<xmin)std::swap(xmax,xmin);
		if(ymax<ymin)std::swap(ymax,ymin);

		char size=cv::waitKey();
		cv::Scalar color=cv::Scalar(0,0,0);

		int xc=(xmin+xmax)/2,yc=(ymin+ymax)/2;
		int h=(ymax-ymin)/2,w=(xmax-xmin)/2;

		switch(size)
		{
		case 'l':
			color=cv::Scalar(0,0,255);
			break;
		case 'm':
			color=cv::Scalar(0,255,0);
			break;
		case 's':
			color=cv::Scalar(255,0,0);
			break;
		}
		int p=xy2p(xc,yc,image->cols);
		cv::ellipse(*image,p2cvp(p,image->cols),cv::Size(w,h),0,0,360,color,1,CV_AA);
		dicesize->insert(std::multimap<char,int>::value_type(size,p));
		cv::imshow(WINDOW_COUNT,*image);
	}
}
