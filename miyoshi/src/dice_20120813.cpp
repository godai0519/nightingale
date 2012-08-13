// �T�C�R�����o�A���S���Y������
// copyright(c) 2012 Shota Miyoshi

//�s���ɂ�����VisualStudio2008+OpenCV2.1�ƂȂ��Ă���܂�.

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

typedef struct Dice
{
	int h;
	int w;
	int value;
	double aspect;
}Dice;


const char* WinConv="Converted Image";
const char* WinDet="Detected Eyes";
const char* WinDice="Detected Dice";

//�����v���g�^�C�v
void ioConv(const cv::Mat& src,cv::Mat& dst);
void convert(const cv::Mat& src,cv::Mat& dst,int thblack,int thwhite);
void ioGeteye(const cv::Mat& src,cv::Mat& dst,std::map<int,cv::Size>& blackeye,std::map<int,cv::Size>& redeye);
void label(const cv::Mat& from,cv::Mat& to,int xs,int ys,int R,int G,int B);
void geteye(const cv::Mat& src,cv::Mat& dst,std::map<int,cv::Size>& blackeye,std::map<int,cv::Size>& redeye,int blackmin,int blackmax,int redmin,int redmax);
std::pair<int,cv::Size> getsize(const cv::Mat& image,int xs,int ys,int& area,int& perimeter,std::set<int>& visited);
void ioGetdice(std::map<int,Dice> dice,std::map<int,cv::Size>& eyes);
void getdice(std::map<int,Dice>& dice,std::map<int,cv::Size>& eyes,int dist,double r_h,double r_aspect);
void redeye2dice(std::map<int,Dice>& dice,std::map<int,cv::Size>& redeye);
void countdice(std::map<int,Dice>& dice);


int height,width;




int _tmain(int argc, _TCHAR* argv[])
{
	cv::Mat input,converted,detected;
	std::map<int,cv::Size> blackeye,redeye;
	std::map<int,Dice> dice;

	char filename[100]="E:\\dice\\P_One(3).jpg";
	//std::cin.getline(filename,100);

	input=cv::imread(filename);
	height=input.rows;
	width=input.cols;

	//���͉摜�\��
	cv::namedWindow("input_img", CV_WINDOW_NORMAL);
	cv::imshow("input_img",input);

	//���͉摜4�l��
	cv::namedWindow(WinConv, CV_WINDOW_NORMAL);
	ioConv(input,converted);

	//�ڂ����o
	cv::namedWindow(WinDet,CV_WINDOW_NORMAL);
	ioGeteye(converted,detected,blackeye,redeye);

	//�ڂ���T�C�R�����o
	cv::namedWindow(WinDice,CV_WINDOW_NORMAL);
	ioGetdice(dice,blackeye);
	redeye2dice(dice,redeye);
	countdice(dice);
	cv::waitKey();

	return 0;
}


void ioConv(const cv::Mat& src,cv::Mat& dst)
//4�l������I/O����+���̑�
{
	char flag;
	int thb=128,thw=192;
	//�悸�͊����臒l�ŕϊ�
	//�Ԃ�臒l�͍��̏��ȗ�
	std::cout<<"Threshold of Black: "<<thb<<std::endl;
	std::cout<<"Threshold of White: "<<thw<<std::endl;
	convert(src,dst,thb,thw);
	std::cout<<"\nConvert again? Yes->y,No->n  *Please Activate \"Converted Image\""<<std::endl;
	flag=cv::waitKey();
	std::cout<<flag<<std::endl;

	//臒l�����[�U���ݒ�(CUI�ɂ��)
	while(flag=='y')
	{
		std::cout<<"Threshold of Black,0-255 : ";
		std::cin>>thb;
		std::cout<<"Threshold of White,0-255 : ";
		std::cin>>thw;
		convert(src,dst,thb,thw);
		std::cout<<"\nConvert Again? Yes->y,No->n  *Please Activate \"Converted Image\""<<std::endl;
		flag=cv::waitKey(0);
		std::cout<<flag<<std::endl;
	}
}

void convert(const cv::Mat& src,cv::Mat& dst,int thblack,int thwhite)
//�摜���,��,��,���̑�(�D)��4�l��
{
	cv::Mat temp_img= cv::Mat(cv::Size(width,height),CV_8UC3);
	cv::Mat hsv_img,blur_img;
	
	cv::bilateralFilter(src,blur_img,20,100,50);

	//RGB->HSV�֕ϊ� H=hue(�F��),S=saturation(�ʓx),V=value(���x)�ƌ����Ӗ�
	cv::cvtColor(blur_img,hsv_img, CV_BGR2HSV);
	temp_img = cv::Scalar(0,0,0);
	for(int y=0; y<src.rows;y++)
	{
		for(int x=0; x<src.cols; x++)
		{
			int a = hsv_img.step*y+(x*3);
			//�Ԍ��o
			if((hsv_img.data[a] >=170 || hsv_img.data[a] <=10) &&hsv_img.data[a+1] >=50)
			{
				temp_img.data[a] = 0;
				temp_img.data[a+1] = 0;
				temp_img.data[a+2] = 255;
			}
			//��(RGB�l��臒l��菬)
			else if(src.data[a]<thblack&&src.data[a+1]<thblack&&src.data[a+2]<thblack)
			{
				temp_img.data[a] = 0;
				temp_img.data[a+1] = 0;
				temp_img.data[a+2] = 0;
			}
			//��(RGB�l��臒l����)
			else if(src.data[a]>thwhite&&src.data[a+1]>thwhite&&src.data[a+2]>thwhite)
			{
				temp_img.data[a] = 255;
				temp_img.data[a+1] = 255;
				temp_img.data[a+2] = 255;
			}
			//�ǂ�ł��Ȃ��ꍇ�͊D
			else
			{
				temp_img.data[a] = 128;
				temp_img.data[a+1] = 128;
				temp_img.data[a+2] = 128;
			}
		}
	}
	dst=temp_img;

	cv::imshow(WinConv,dst);//4�l�摜�\��
}

void ioGeteye(const cv::Mat& src,cv::Mat& dst,std::map<int,cv::Size>& blackeye,std::map<int,cv::Size>& redeye)
//�ڂ����m����ۂ�I/O����
{
	//4�l�摜(converted)����ڂ�T��
	//�n�߂͊���l�T��
	char flag;
	int bmin=1,bmax=20,rmin=1,rmax=100;
	std::cout<<"\n\nMinimum size of Black Eye: "<<bmin<<std::endl;
	std::cout<<"Maximum size of Black Eye: "<<bmax<<std::endl;
	std::cout<<"Minimum size of Red Eye:   "<<rmin<<std::endl;
	std::cout<<"Maximum size of Red Eye:   "<<rmax<<std::endl;
	geteye(src,dst,blackeye,redeye,bmin,bmax,rmin,rmax);
	std::cout<<"\ndetect Again? Yes->y,No->n *Please Activate \"Detected Eyes\""<<std::endl;
	flag=cv::waitKey();
	std::cout<<flag<<std::endl;

	//���������[�U���ݒ�
	//�ݒ�\�͐Ԗ�,���ڂ̑傫������,���
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
		geteye(src,dst,blackeye,redeye,bmin,bmax,rmin,rmax);
		std::cout<<"\ndetect Again? Yes->y,No->n  *Please Activate \"Detected Eyes\""<<std::endl;
		flag=cv::waitKey(0);
		std::cout<<flag<<std::endl;
	}
}

void geteye(const cv::Mat& src,cv::Mat& dst,std::map<int,cv::Size>& blackeye,std::map<int,cv::Size>& redeye,int blackmin,int blackmax,int redmin,int redmax)
//4�l�摜����ڂ����o
{
	cv::Mat detection=cv::Mat::zeros(height,width,CV_8UC3);
	std::pair<int,cv::Size> eye;
	std::set<int> visited;
	int area,perimeter;
	blackeye.clear();
	redeye.clear();

	for(int y=0;y<height;y++)
	{
		for(int x=0;x<width;x++)
		{
			if(visited.find(x+y*width)==visited.end())
			{
				//��f���ԐF->�Ԗ�
				if(src.data[src.step*y+x*3]==0&&src.data[src.step*y+x*3+1]==0&&src.data[src.step*y+x*3+2]==255)
				{
					eye=getsize(src,x,y,area,perimeter,visited);
					if((eye.second.height>=redmin && eye.second.width>=redmin) && (eye.second.height<=redmax && eye.second.width<=redmax))
					{
						redeye.insert(eye);
						label(src,detection,x,y,255,eye.second.height>eye.second.width?255:0,0);
					}
				}
				//��f�����F->����
				else if(src.data[src.step*y+x*3]==0&&src.data[src.step*y+x*3+1]==0&&src.data[src.step*y+x*3+2]==0)
				{
					eye=getsize(src,x,y,area,perimeter,visited);
					if((eye.second.height>=blackmin && eye.second.width>=blackmin) && (eye.second.height<=blackmax && eye.second.width<=blackmax))
					{
						blackeye.insert(eye);
						label(src,detection,x,y,0,255,eye.second.height>eye.second.width?255:0);
					}
				}
			}
		}
	}

	cv::imshow(WinDet,detection);
	dst=detection;
}

std::pair<int,cv::Size> getsize(const cv::Mat& image,int xs,int ys,int& area,int& perimeter,std::set<int>& visited)//xs,ys���܂ޗ̈�T�C�Y�擾
{
	std::stack<int> stk;

	int s=0,l=0;
	int xmin=xs,xmax=xs,ymin=ys,ymax=ys;
	int scolor[3];
	std::pair<int,cv::Size> eye;

	for(int i=0;i<3;i++){scolor[i]=image.data[image.step*ys+xs*3+i];}

	stk.push(xs+ys*width);

	//�ċA����Ȃ�����dfs(depth first search)�ł�
	while(!stk.empty())
	{
		bool flag=false;
		int x=stk.top()%width;
		int y=stk.top()/width;
		stk.pop();
		if(visited.find(x+y*width)!=visited.end())continue;
		for(int i=0;i<3;i++){if(image.data[image.step*y+x*3+i]!=scolor[i])flag=true;}
		if(flag){l++;continue;}

		visited.insert(x+y*width);
		xmin=xmin>x?x:xmin;
		xmax=xmax<x?x:xmax;
		ymin=ymin>y?y:ymin;
		ymax=ymax<y?y:ymax;

		if(x>0){stk.push(x-1+y*width);}
		else{l++;}
		if(x<width-1){stk.push(x+1+y*width);}
		else{l++;}
		if(y>0){stk.push(x+(y-1)*width);}
		else{l++;}
		if(y<height-1){stk.push(x+(y+1)*width);}
		else{l++;}
		s++;
	}
	eye.first=xmin+(xmax-xmin)/2+(ymin+(ymax-ymin)/2)*width;
	eye.second.height=ymax-ymin+1;
	eye.second.width=xmax-xmin+1;

	area=s;
	perimeter=l;
	return eye;
}

void label(const cv::Mat& from,cv::Mat& to,int xs,int ys,int R,int G,int B)
//xs,ys�Ɠ����F�̗̈��R,G,B�Ŏw�肳���F�œh��Ԃ�
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
		if(visited.find(x+y*width)!=visited.end())continue;

		for(int i=0;i<3;i++){if(from.data[from.step*y+x*3+i]!=scolor[i])flag=true;}
		if(flag)continue;

		to.data[to.step*y+x*3]=B;
		to.data[to.step*y+x*3+1]=G;
		to.data[to.step*y+x*3+2]=R;

		visited.insert(x+y*width);
		if(x>0)stk.push(x-1+y*width);
		if(x<width-1)stk.push(x+1+y*width);
		if(y>0)stk.push(x+(y-1)*width);
		if(y<height-1)stk.push(x+(y+1)*width);
	}
}

void ioGetdice(std::map<int,Dice> dice,std::map<int,cv::Size>& eyes)
{
	char flag;
	int dist=5;
	double r_h=.2,r_aspect=.2;
	std::cout<<"\n\nMaximum Distance Between Eyes: "<<dist<<std::endl;
	std::cout<<"Torelance of Height: "<<r_h<<std::endl;
	std::cout<<"Torerance of Aspect Ratio:"<<r_aspect<<std::endl;
	getdice(dice,eyes,dist,r_h,r_aspect);
	std::cout<<"\ndetect Again? Yes->y,No->n *Please Activate \"Detected Dice\""<<std::endl;
	flag=cv::waitKey();
	std::cout<<flag<<std::endl;

	//���������[�U���ݒ�
	//�ݒ�\�͖ڂ̊Ԃ̋���,����,�c�����,��ƂȂ�ڂƂ̔�
	while(flag=='y')
	{
	std::cout<<"\n\nMaximum Distance Between Eyes: "<<std::endl;
	std::cin>>dist;
	std::cout<<"Torelance of Height: "<<std::endl;
	std::cin>>r_h;
	std::cout<<"Torerance of Aspect Ratio:"<<std::endl;
	std::cin>>r_aspect;
	getdice(dice,eyes,dist,r_h,r_aspect);
	std::cout<<"\ndetect Again? Yes->y,No->n *Please Activate \"Detected Dice\""<<std::endl;
	flag=cv::waitKey();
	std::cout<<flag<<std::endl;
	}
}

void getdice(std::map<int,Dice>& dice,std::map<int,cv::Size>& eyes,int dist,double r_h,double r_aspect)
//�����ʂ̑傫���ō��W���߂��ڂ���̖ʂɂ܂Ƃ߂�
{
	cv::Mat image=cv::Mat::zeros(height,width,CV_8UC3);
	std::set<int> visited;

	dice.clear();
	for(std::map<int,cv::Size>::iterator i=eyes.begin();i!=eyes.end();i++)
	{
		std::vector<std::pair<int,cv::Size>> temp;//�T�C�R���̖ڏW��
		std::vector<cv::Point> coordinate;//�T�C�R���̖ڍ��W
		if(visited.find((*i).first)==visited.end())
		{
			int h=(*i).second.height,w=(*i).second.width;
			int k=0;
			visited.insert((*i).first);
			temp.push_back(*i);
			coordinate.push_back(cv::Point((*i).first%width,(*i).first/width));

			//�ߖT����(x,y���W����̖ڂ̍���,����dist�{�ȓ��ɂ��邩?)
			for(int y=(*i).first/width-h*dist;y<=(*i).first/width+h*dist;y++)
			{
				for(int x=(*i).first%width-w*dist;x<=(*i).first%width+w*dist;x++)
				{
					if(eyes.find(x+y*width)!=eyes.end()&&visited.find(x+y*width)==visited.end())
					{
						//�傫������(�����̔䂪�߂���c���䂪�߂�)
						if((double)eyes[x+y*width].height/h<=1+(h>5?r_h:r_h*2) && (double)eyes[x+y*width].height/h>=1-(h>5?r_h:r_h*2) && (double)eyes[x+y*width].height/eyes[x+y*width].width<=(1+(h>5?r_aspect:r_aspect*2))*h/w && (double)eyes[x+y*width].height/eyes[x+y*width].width>=(1-(h>5?r_aspect:r_aspect*2))*h/w)
						{
							temp.push_back(std::pair<int,cv::Size>::pair(x+y*width,eyes[x+y*width]));
							coordinate.push_back(cv::Point(x,y));
							visited.insert(x+y*width);
						}
					}
				}
			}
		}

		if(temp.size()>1)//���o�����T�C�R�����l�p�ŕ\��
		{
			
			cv::RotatedRect box=cv::minAreaRect(cv::Mat(coordinate));
			cv::Point2f vtx[4];
			int xmin=width,xmax=0,ymin=height,ymax=0, avh=0,avw=0;
			double aspect;
			Dice d;

			box.points(vtx);
			
			
			//�ڂ�`��
			for(std::vector<std::pair<int,cv::Size>>::iterator iteye=temp.begin();iteye!=temp.end();iteye++)
			{
				cv::Size size=cv::Size2f((*iteye).second.width/2.0,(*iteye).second.height/2.0);
				cv::ellipse(image,cv::Point((*iteye).first%width,(*iteye).first/width),size,0,0,360,cv::Scalar(0,255,(*iteye).second.height>(*iteye).second.width?255:0),-1);
				avh+=(*iteye).second.height;
				avw+=(*iteye).second.width;
				aspect+=(double)(*iteye).second.height/(*iteye).second.width;
			}

			avh/=temp.size()*5;//
			avw/=temp.size()*5;
			aspect/=temp.size();

			//�ڂ��͂ގl�p��`��
			for(int j=0;j<4;j++)
			{
				xmin=xmin>vtx[j].x?vtx[j].x:xmin;
				xmax=xmax<vtx[j].x?vtx[j].x:xmax;
				ymin=ymin>vtx[j].y?vtx[j].y:ymin;
				ymax=ymax<vtx[j].y?vtx[j].y:ymax;
				cv::line(image, vtx[j], vtx[j<3?j+1:0], cv::Scalar(0,0,255), 1, CV_AA);
			}

			d.h=avh;
			d.w=avw;
			d.aspect=aspect;
			d.value=temp.size();

			dice.insert(std::map<int,Dice>::value_type(box.center.x+box.center.y*width,d));
		}
	}
	cv::imshow(WinDice,image);
	cv::waitKey();
}

void redeye2dice(std::map<int,Dice>& dice,std::map<int,cv::Size>& redeye)
//�Ԗڂ�dice�ɓ����
{
	for(std::map<int,cv::Size>::iterator ite=redeye.begin();ite!=redeye.end();ite++)
	{
		Dice d;
		d.value=1;
		d.h=(*ite).second.height*3;//�T�C�R���{�͖̂ڂ��傫��
		d.w=(*ite).second.width*3;
		d.aspect=(*ite).second.height/(*ite).second.width;
		dice.insert(std::map<int,Dice>::value_type((*ite).first,d));
	}
}

void countdice(std::map<int,Dice>& dice)
{
	int number=0;

	for(std::map<int,Dice>::iterator itd=dice.begin();itd!=dice.end();itd++)
	{
		//�����ɑ傫�������....

		//���
		if((*itd).second.aspect<1)
		{
			number++;
		}
		//����
		else
		{
			//������(�T�C�R��������2�{�ȓ�)�ɏ�ʂ͂��邩?
			bool onlyside=true;
			for(int y=(*itd).first/width-(*itd).second.h*2;y<(*itd).first/width;y++)
			{
				for(int x=(*itd).first%width-(*itd).second.w*2;x<(*itd).first%width+(*itd).second.w*2;x++)
				{
					if(dice.find(x+y*width)!=dice.end())
					{
						onlyside=false;
						break;
					}
				}
			}
			if(onlyside)number++;
		}
	}
	std::cout<<"\n\nNumber of Dice: "<<number<<std::endl;
}