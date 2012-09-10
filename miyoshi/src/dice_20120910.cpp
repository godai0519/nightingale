// サイコロ検出アルゴリズム試作
// copyright(c) 2012 Shota Miyoshi

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
}Dice;

const char* WINDOW_RED="Converted Image";
const char* WINDOW_EYE="Detected Eyes";
const char* WINDOW_DICE="Detected Dice";
const char* WINDOW_COUNT="Counted Dice";

//函数プロトタイプ
void convert(const cv::Mat& src,cv::Mat& dst);
void ioGeteye(const cv::Mat& black,const cv::Mat& red,std::map<int,cv::Size>& blackeye,std::map<int,cv::Size>& redeye);
void geteye(const cv::Mat& src,cv::Mat& dst,std::map<int,cv::Size>& eyes,int min,int max,bool red);
std::pair<int,cv::Size> getsize(const cv::Mat& image,int xs,int ys,int& area,int& perimeter,std::set<int>& visited);
void ioGetdice(cv::Mat& image,std::map<int,Dice>& top,std::map<int,Dice>& side,std::map<int,cv::Size>& eyes);
void getdice(const cv::Mat& input,std::map<int,Dice>& top,std::map<int,Dice>& side,std::map<int,cv::Size>& eyes,double dist,double r_h,double r_aspect);
void redeye2dice(const cv::Mat& input,std::map<int,Dice>& top,std::map<int,Dice>& side,std::map<int,cv::Size>& redeye);
void countdice(cv::Mat& image,std::map<int,Dice>& top,std::map<int,Dice>& side);
double getperimeter(const cv::Mat& image,int ps);
void onMouse_count(int event,int x,int y,int flag,void* param);

///////////////////////////////////////////////////////////////////////////////

//この辺は東様のコードを引用させていただいております(私しの物が芳しくなかった...)//

///////////////////////////////////////////////////////////////////////////////

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


	//厳しめの判定をしたから膨張させる
	cv::dilate(dst,dst,cv::Mat());
}

///////////////////////////////////////////////////////////////////////////////

void draweye(int w,int h,std::map<int,cv::Size>& black,std::map<int,cv::Size>& red)
{
	cv::Mat img=cv::Mat::zeros(h,w,CV_8UC3);
	for(int y=0;y<h;y++)
	{
		for(int x=0;x<w;x++)
		{
			if(black.find(y*w+x)!=black.end())
			{
				cv::ellipse(img,cv::Point(x,y),cv::Size(black[x+y*w].width/2,black[x+y*w].height/2),0,0,360,cv::Scalar(255,255,255),-1);
			}
			if(red.find(y*w+x)!=red.end())
			{
				cv::ellipse(img,cv::Point(x,y),cv::Size(red[x+y*w].width/2,red[x+y*w].height/2),0,0,360,cv::Scalar(0,0,255),-1);
			}
		}
	}
	cv::imshow("eyes",img);
}



int _tmain(int argc, _TCHAR* argv[])
{
	cv::Mat input,black,red,converted,detected;
	std::map<int,cv::Size> blackeye,redeye;
	std::map<int,Dice> top,side;

	char filename[100]="E:\\Dice\\dice.jpg";
	//std::cin.getline(filename,100);

	input=cv::imread(filename);

	//入力画像表示
	cv::namedWindow("input_img", CV_WINDOW_NORMAL);
	cv::imshow("input_img",input);

	//画像変換(赤,黒抽出)
	black_area(black,input);
	red_area(red,input);

	//cv::imshow("black",black);
	//cv::imshow("red",red);

	//目を検出
	ioGeteye(black,red,blackeye,redeye);

	//赤目と黒目の両方を書き出す
	draweye(input.cols,input.rows,blackeye,redeye);

	//目からサイコロ検出
	ioGetdice(input,top,side,blackeye);
	redeye2dice(input,top,side,redeye);
	countdice(input,top,side);
	cv::waitKey();

	return 0;
}


void ioGeteye(const cv::Mat& black,const cv::Mat& red,std::map<int,cv::Size>& blackeye,std::map<int,cv::Size>& redeye)
//目を検知する際のI/O処理
{
	//4値画像(converted)から目を探索
	//始めは既定値探索
	cv::Mat dst=cv::Mat::zeros(black.rows,black.cols,CV_8UC3);
	char flag;
	int bmin=1,bmax=20,rmin=1,rmax=100;
	cv::namedWindow(WINDOW_EYE,CV_WINDOW_NORMAL);
	std::cout<<"\n\nMinimum size of Black Eye: "<<bmin<<std::endl;
	std::cout<<"Maximum size of Black Eye: "<<bmax<<std::endl;
	std::cout<<"Minimum size of Red Eye:   "<<rmin<<std::endl;
	std::cout<<"Maximum size of Red Eye:   "<<rmax<<std::endl;
	geteye(black,dst,blackeye,bmin,bmax,false);
	geteye(red,dst,redeye,rmin,rmax,true);
	cv::imshow(WINDOW_EYE,dst);
	std::cout<<"\ndetect Again? Yes->y,No->n *Please Activate \"Detected Eyes\""<<std::endl;
	flag=cv::waitKey();
	std::cout<<flag<<std::endl;

	//条件をユーザが設定
	//設定可能は赤目,黒目の大きさ下限,上限
	while(flag=='y')
	{
		dst=cv::Mat::zeros(black.rows,black.cols,CV_8UC3);
		std::cout<<"Minimum size of Black Eye: ";
		std::cin>>bmin;
		std::cout<<"Maximum size of Black Eye: ";
		std::cin>>bmax;
		std::cout<<"Minimum size of Red Eye:   ";
		std::cin>>rmin;
		std::cout<<"Maximum size of Red Eye:   ";
		std::cin>>rmax;
		geteye(black,dst,blackeye,bmin,bmax,false);
		geteye(red,dst,redeye,rmin,rmax,true);
		std::cout<<"\ndetect Again? Yes->y,No->n  *Please Activate \"Detected Eyes\""<<std::endl;
		cv::imshow(WINDOW_EYE,dst);
		flag=cv::waitKey(0);
		std::cout<<flag<<std::endl;
	}
	cv::destroyWindow(WINDOW_EYE);
}

void geteye(const cv::Mat& src,cv::Mat& dst,std::map<int,cv::Size>& eyes,int min,int max,bool red)
{
	//目を抽出
	std::set<int> visited;
	eyes.clear();

	for(int ys=0;ys<src.rows;ys++)
	{
		for(int xs=0;xs<src.cols;xs++)
		{
			if(src.data[src.step*ys+xs]==0||visited.find(xs+ys*src.cols)!=visited.end())continue;

			int s=0,l=0;
			int xmin=xs,xmax=xs,ymin=ys,ymax=ys;
			std::pair<int,cv::Size> eye;
			std::set<int> region;
			std::stack<int> stk;
			stk.push(xs+ys*src.cols);

			//再帰じゃないけどdfs(depth first search)です
			//dfsで塗りつぶす事で領域を決定
			while(!stk.empty())
			{
				bool flag=false;
				int x=stk.top()%src.cols;
				int y=stk.top()/src.cols;
				stk.pop();
				if(region.find(x+y*src.cols)!=region.end()||src.data[src.step*y+x]==0)continue;

				region.insert(x+y*src.cols);
				xmin=xmin>x?x:xmin;
				xmax=xmax<x?x:xmax;
				ymin=ymin>y?y:ymin;
				ymax=ymax<y?y:ymax;

				if(x>0){stk.push(x-1+y*src.cols);}
				if(x<src.cols-1){stk.push(x+1+y*src.cols);}
				if(y>0){stk.push(x+(y-1)*src.cols);}
				if(y<src.rows-1){stk.push(x+(y+1)*src.cols);}
				s++;
			}

			//領域が小さすぎ,大きすぎなければ目として処理
			int h=ymax-ymin+1,w=xmax-xmin+1;
			if(h>=min&&h<=max&&w>=min&&w<=max)
			{
				//double l=getperimeter(src,xs+ys*src.cols);
				//円形度判定(円形度=4πs/l^2)
				//if(12.566370614359173*s/l*l>0.5)
				{
					eye.first=xmin+(xmax-xmin)/2+(ymin+(ymax-ymin)/2)*src.cols;//目の中心座標
					eye.second.height=h;
					eye.second.width=w;

					//塗る色設定&目の集合に挿入
					//赤目横長->赤,赤目縦長->黄,->黒目横長->青,黒目縦長->青緑
					int color[3];
					eyes.insert(eye);
					if(red)
					{
						color[0]=0;color[1]=eye.second.height>eye.second.width?255:0;color[2]=255;
					}
					else
					{
						color[0]=255;color[1]=eye.second.height>eye.second.width?255:0;color[2]=0;
					}
					for(std::set<int>::iterator it=region.begin();it!=region.end();it++)
					{
						for(int i=0;i<3;i++)
						{
							dst.data[dst.step*((*it)/src.cols)+(*it)%src.cols*3+i]=color[i];
						}
					}
				}
			}
			visited.insert(region.begin(),region.end());
		}
	}
}

double getperimeter(const cv::Mat& image,int p0)
{
	//周長を求める
	const int move[8][2]={{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1},{0,1},{1,1}};
	int scolor[3];
	for(int i=0;i<3;i++){scolor[i]=image.data[image.step*(p0/image.cols)+p0%image.cols*3+i];}
	int d=4,d1,p1=-1;
	int pn=p0,pn1;
	int diagonal=0,perpendicular=0;
	while(1)
	{
		d1=-1;
		for(int k=0;k<8;k++)
		{
			bool flag=true;
			for(int i=0;i<3;i++)
			{
				int x=pn%image.cols+move[(d+k)%8][0],y=pn/image.cols+move[(d+k)%8][1];
				if(x>=0&&x<image.cols&&y>=0&&y<image.rows)
				{
					if(image.data[image.step*y+x*3+i]!=scolor[i])
					{
						flag=false;
					}
				}
			}
			if(flag)d1=(d+k)%8;
		}
		if(d1==-1)break;//孤立点
		
		pn1=pn;//移動
		pn+=move[d1][0]+move[d1][1]*image.cols;
		if(p1==-1){p1=pn;}
		else{if(pn==p1&&pn1==p0)break;}
		if(d1%2==1){diagonal++;}
		else{perpendicular++;}
	}
	return diagonal*1.41421356237309505+perpendicular;
}


void ioGetdice(cv::Mat& image,std::map<int,Dice>& top,std::map<int,Dice>& side,std::map<int,cv::Size>& eyes)
{
	char flag;
	double dist=3;
	double r_h=1.1,r_aspect=1.1;
	
	cv::namedWindow(WINDOW_DICE,CV_WINDOW_NORMAL);
	std::cout<<"\n\nMaximum Distance Between Eyes: "<<dist<<std::endl;
	std::cout<<"Torelance of Height: "<<r_h<<std::endl;
	std::cout<<"Torerance of Aspect Ratio:"<<r_aspect<<std::endl;
	getdice(image,top,side,eyes,dist,r_h,r_aspect);
	std::cout<<"\ndetect Again? Yes->y,No->n *Please Activate \"Detected Dice\""<<std::endl;
	flag=cv::waitKey();
	std::cout<<flag<<std::endl;

	//条件をユーザが設定
	//設定可能は目の間の距離,高さ,縦横比の,基準となる目との比
	while(flag=='y')
	{
		std::cout<<"\n\nMaximum Distance Between Eyes: "<<std::endl;
		std::cin>>dist;
		std::cout<<"Torelance of Height: "<<std::endl;
		std::cin>>r_h;
		std::cout<<"Torelance of Aspect Ratio:"<<std::endl;
		std::cin>>r_aspect;
		getdice(image,top,side,eyes,dist,r_h,r_aspect);
		std::cout<<"\ndetect Again? Yes->y,No->n *Please Activate \"Detected Dice\""<<std::endl;
		flag=cv::waitKey();
		std::cout<<flag<<std::endl;
	}
	cv::destroyWindow(WINDOW_DICE);
}


void getdice(const cv::Mat& input,std::map<int,Dice>& top,std::map<int,Dice>& side,std::map<int,cv::Size>& eyes,double dist,double r_h,double r_aspect)
{
	//同じ位の大きさで座標が近い目を一つの面にまとめる(未だ難あり)
	cv::Mat image=input.clone();
	std::set<int> visited;

	top.clear();side.clear();
	for(std::map<int,cv::Size>::iterator i=eyes.begin();i!=eyes.end();i++)
	{
		std::vector<std::pair<int,cv::Size> > temp;//サイコロの目集合
		std::vector<cv::Point> coordinate;//サイコロの目座標
		if(visited.find((*i).first)==visited.end())
		{
			int h=(*i).second.height,w=(*i).second.width;
			int k=0;
			visited.insert((*i).first);
			temp.push_back(*i);
			coordinate.push_back(cv::Point((*i).first%input.cols,(*i).first/input.cols));

			//近傍判定(x,y座標が基準の目の高さ,幅のdist倍以内にあるか?)
			for(int y=(*i).first/input.cols;y<=(*i).first/input.cols+h*dist;y++)
			{
				for(int x=(*i).first%input.cols-w*dist;x<=(*i).first%input.cols+w*dist;x++)
				{
					if(eyes.find(x+y*input.cols)!=eyes.end()&&visited.find(x+y*input.cols)==visited.end())
					{
						//大きさ判定(高さの比がr_h倍以内･縦横比r_aspect倍以内)

						int h1=h,h2=eyes[x+y*input.cols].height;
						int w1=w,w2=eyes[x+y*input.cols].width;
						double a1=(double)h/w,a2=(double)eyes[x+y*input.cols].height/eyes[x+y*input.cols].width;
						if(h1<h2)std::swap(h1,h2);
						if(w1<w2)std::swap(w1,w2);

						if((double)h1/h2<=r_h && (double)w1/w2<=r_aspect)
						{
							temp.push_back(std::pair<int,cv::Size>::pair(x+y*input.cols,eyes[x+y*input.cols]));
							coordinate.push_back(cv::Point(x,y));
							visited.insert(x+y*input.cols);
						}
					}
				}
			}
		}

		if(temp.size()>1)//検出したサイコロを四角で表示
		{
			
			cv::RotatedRect box=cv::minAreaRect(cv::Mat(coordinate));
			cv::Point2f vtx[4];
			int xmin=input.cols,xmax=0,ymin=input.rows,ymax=0,isize=0,avh=0,avw=0;
			double aspect;
			Dice d;

			box.points(vtx);
			
			
			//目を描画
			for(std::vector<std::pair<int,cv::Size>>::iterator iteye=temp.begin();iteye!=temp.end();iteye++)
			{
				cv::Size size=cv::Size2f((*iteye).second.width/2.0,(*iteye).second.height/2.0);
				cv::ellipse(image,cv::Point((*iteye).first%input.cols,(*iteye).first/input.cols),size,0,0,360,cv::Scalar(0,255,(*iteye).second.height>(*iteye).second.width?255:0),-1);
				avh+=(*iteye).second.height;
				avw+=(*iteye).second.width;
			}

			isize=(avh+avw)/2/temp.size();
			avh=avh/temp.size()*3;//サイコロの一辺は目のサイズの3倍位
			avw=avw/temp.size()*3;
			aspect=(double)avh/avw;

			//目を囲む四角を描画
			for(int j=0;j<4;j++)
			{
				xmin=xmin>vtx[j].x?vtx[j].x:xmin;
				xmax=xmax<vtx[j].x?vtx[j].x:xmax;
				ymin=ymin>vtx[j].y?vtx[j].y:ymin;
				ymax=ymax<vtx[j].y?vtx[j].y:ymax;
				cv::line(image, vtx[j], vtx[j<3?j+1:0], cv::Scalar(0,0,192), 1, CV_AA);
			}

			d.h=avh;
			d.w=avw;
			d.aspect=aspect;
			d.eyesize=isize;
			d.value=temp.size();

			if(d.aspect<=1){top.insert(std::map<int,Dice>::value_type(box.center.x+box.center.y*input.cols,d));cv::ellipse(image,box.center,cv::Size(avw/2,avh/2),0,0,360,cv::Scalar(0,0,255),1,CV_AA);}
			else{side.insert(std::map<int,Dice>::value_type(box.center.x+box.center.y*input.cols,d));cv::ellipse(image,box.center,cv::Size(avw/2,avh/2),0,0,360,cv::Scalar(255,0,0),1,CV_AA);}
		}
	}
	cv::imshow(WINDOW_DICE,image);
}

void redeye2dice(const cv::Mat& input,std::map<int,Dice>& top,std::map<int,Dice>& side,std::map<int,cv::Size>& redeye)
//赤目もdiceに入れる
{
	cv::Mat image=input.clone();
	for(std::map<int,cv::Size>::iterator it=redeye.begin();it!=redeye.end();it++)
	{
		Dice d;
		d.value=1;
		d.h=(*it).second.height*2;//サイコロ本体は目より大きいので,2倍(適当)
		d.w=(*it).second.width*2;
		d.aspect=(double)(*it).second.height/(*it).second.width;
		d.eyesize=((*it).second.height+(*it).second.width)/2;
		if(d.aspect<=1){top.insert(std::map<int,Dice>::value_type((*it).first,d));cv::ellipse(image,cv::Point2i((*it).first%input.cols,(*it).first/input.cols),cv::Size(d.w/2,d.h/2),0,0,360,cv::Scalar(0,0,255),1,CV_AA);}
		else{side.insert(std::map<int,Dice>::value_type((*it).first,d));cv::ellipse(image,cv::Point2i((*it).first%input.cols,(*it).first/input.cols),cv::Size(d.w/2,d.h/2),0,0,360,cv::Scalar(0,255,0),1,CV_AA);}
	}
}

void countdice(cv::Mat& image,std::map<int,Dice>& top,std::map<int,Dice>& side)
{
	std::multimap<char,int> dicesize;//サイズ別のサイコロを此処に入れる
	cv::Mat temp_img,counted_img=image.clone();
	int l=0,m=0,s=0;//各サイズの数

	int division[21];//観測範囲の帯

	for(int i=0;i<=20;i++){division[i]=image.rows*i/20;}

	//上面
	for(int i=0;i<20;i++)
	{
		int maxsize=0,pos;//同じ観測帯で大きさ最大の物を基準にする
		char basesize;
		bool found=false;
		std::map<int,Dice>::iterator begin=top.end(),end;
		for(int y=division[i];y<division[i+1];y++)
		{
			for(int x=0;x<image.cols;x++)
			{
				if(top.find(x+y*image.cols)!=top.end())
				{
					found=true;
					if(begin==top.end()){begin=top.find(x+y*image.cols);}
					if(top[y*image.cols+x].eyesize>maxsize)
					{
						maxsize=top[y*image.cols+x].eyesize;
						pos=y*image.cols+x;
					}
					end=top.find(x+y*image.cols);
					end++;
				}
			}
		}
		if(!found)continue;
		//基準のサイズを選ばせる
		temp_img=image.clone();
		cv::ellipse(temp_img,cv::Point(pos%image.cols,pos/image.cols),cv::Size(top[pos].w,top[pos].h),0,0,360,cv::Scalar(0,0,255),3,4);
		cv::imshow("CountDice",temp_img);
		std::cout<<"Select size of this dice.(please enter s, m, or l)"<<std::endl;
		basesize=cv::waitKey();

		//大きさ判定
		for(int y=division[i];y<division[i+1];y++)
		{
			for(int x=0;x<image.cols;x++)
			{
				if(top.find(x+y*image.cols)!=top.end())
				{
					char size='\0';
					switch(basesize)
					{
					case 'l':
						if(top[x+y*image.cols].eyesize>=top[pos].eyesize*0.9&&top[x+y*image.cols].eyesize<=top[pos].eyesize*1.1)
						{
							size='l';
						}
						else if(top[x+y*image.cols].eyesize>=top[pos].eyesize*0.6&&top[x+y*image.cols].eyesize<=top[pos].eyesize*0.65)
						{
							size='m';
						}
						else if(top[x+y*image.cols].eyesize>=top[pos].eyesize*0.35&&top[x+y*image.cols].eyesize<=top[pos].eyesize*0.4)
						{
							size='s';
						}
						break;
					case 'm':
						if(top[x+y*image.cols].eyesize>=top[pos].eyesize*0.9&&top[x+y*image.cols].eyesize<=top[pos].eyesize*1.1)
						{
							size='m';
						}
						else if(top[x+y*image.cols].eyesize>=top[pos].eyesize*0.55&&top[x+y*image.cols].eyesize<=top[pos].eyesize*0.65)
						{
							size='s';
						}
						break;
					case 's':
						if(top[x+y*image.cols].eyesize>=top[pos].eyesize*0.9&&top[x+y*image.cols].eyesize<=top[pos].eyesize*1.1)
						{
							size='s';
						}
						break;
					}
					dicesize.insert(std::multimap<char,int>::value_type(size,x+y*image.cols));
					cv::Scalar color=cv::Scalar(0,0,0);
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
					cv::ellipse(counted_img,cv::Point(x,y),cv::Size(top[x+y*image.cols].w,top[x+y*image.cols].h),0,0,360,color,1,CV_AA);
				}
			}
		}
	}
	//側面
	//for(std::map<int,Dice>::iterator its=side.begin();its!=side.end();its++)
	//{
	//	//すぐ上(サイコロ高さの2倍以内)に上面はあるか?
	//	bool onlyside=true;
	//	for(int y=(*its).first/image.cols-(*its).second.h*2;y<(*its).first/image.cols;y++)
	//	{
	//		for(int x=(*its).first%image.cols-(*its).second.w*2;x<(*its).first%image.cols+(*its).second.w*2;x++)
	//		{
	//			if(top.find(x+y*image.cols)!=top.end()&&(double)top[x+y*image.cols].eyesize/(*its).second.eyesize)
	//			{
	//				onlyside=false;
	//				break;
	//			}
	//		}
	//	}
	//	if(onlyside)number++;
	//}

	//人力判別(最終手段)
	cv::imshow(WINDOW_COUNT,counted_img);
	std::pair<cv::Mat*,std::multimap<char,int>*> data=std::pair<cv::Mat*,std::multimap<char,int>*>(&counted_img,&dicesize);
	cv::setMouseCallback(WINDOW_COUNT,onMouse_count,&data);
	cv::waitKey();

	
	s=dicesize.count('s');
	m=dicesize.count('m');
	l=dicesize.count('l');


	std::cout<<"\n\nNumber of Dice: "<<s<<' '<<m<<' '<<l<<std::endl;
	cv::destroyAllWindows();
	std::cout<<"\nPress Any Keys to close..."<<std::endl;
	char c;
	std::cin.get(c);
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
		cv::ellipse(*image,cv::Point(xc,yc),cv::Size(w,h),0,0,360,color,1,CV_AA);
		dicesize->insert(std::multimap<char,int>::value_type(size,xc+yc*image->rows));
		cv::imshow(WINDOW_COUNT,*image);
	}
}