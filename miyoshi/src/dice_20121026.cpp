// �T�C�R�����o�A���S���Y��

#include "stdafx.h"
#include <set>
#include <stack>
#include <vector>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "util.hpp"

#ifdef _DEBUG
    #pragma comment(lib,"C:\\opencv\\build\\x86\\vc10\\lib\\opencv_core242d.lib")
    #pragma comment(lib,"C:\\opencv\\build\\x86\\vc10\\lib\\opencv_imgproc242d.lib")
    #pragma comment(lib,"C:\\opencv\\build\\x86\\vc10\\lib\\opencv_highgui242d.lib")
#else
    #pragma comment(lib,"C:\\opencv\\build\\x86\\vc10\\lib\\opencv_core242.lib")
    #pragma comment(lib,"C:\\opencv\\build\\x86\\vc10\\lib\\opencv_imgproc242.lib")
    #pragma comment(lib,"C:\\opencv\\build\\x86\\vc10\\lib\\opencv_highgui242.lib")
#endif


typedef struct Dice
{
	int h;
	int w;
	int eyesize;
	int value;
	double aspect;
	int compsize;
	char size;
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
	double round;
	TempEye(int location,int height,int width,double round);

}TempEye;

TempEye::TempEye(int location,int height,int width,double round)
{
	this->location=location;
	this->height=height;
	this->width=width;
	this->round=round;

}

typedef struct data_rm
{
	cv::Mat* image;
	std::map<int,Dice>* dice;
	int* dicesize;
	data_rm();
	data_rm(cv::Mat* image,std::map<int, Dice>* dice,int* dicesize);
}data_rm;

data_rm::data_rm()
{
	return;
}

data_rm::data_rm(cv::Mat* image,std::map<int, Dice>* dice,int* dicesize)
{
	this->image=image;
	this->dice=dice;
	this->dicesize=dicesize;
}

//����(���L���v�V����)
const char* WINDOW_EYE="Detected Eyes: Wants to detect again, then Press \'y\', else Press \'z\'";
const char* WINDOW_DICE="Detected Dice: Wants to detect again, then Press \'y\',else Press \'z\'";
const char* WINDOW_EXCLUDE="Exclude Error. Drag and Drop around the center of Wrong Eye";
const char* WINDOW_SIZE="Select size: Enter the size of selected Dice,\'l\', \'m\', or \'s\'";
const char* WINDOW_COUNT="Counted Dice: Wants to finish, Press \'z\', to count again,Press any other keys.";
const char* WINDOW_CROP="Crop the image: Drag and Drop, Press \'z\' to Finish";
const char* WINDOW_RM="Remove MIstaken Dice, in case of the end,press \'z\'";

//�����v���g�^�C�v�B
void onMouse_crop(int event,int x,int y,int flag,void* param);
void crop(const cv::Mat& src,cv::Mat& dst);
void ioGeteye(const cv::Mat& image,const cv::Mat& black,const cv::Mat& red,std::map<int,cv::Size>& blackeye,std::map<int,cv::Size>& redeye);
void geteye(const cv::Mat& src,cv::Mat& dst,std::map<int,cv::Size>& eyes,std::vector<TempEye>& tempeyes,bool red);
void geteye_second(const cv::Mat* src,std::map<int,cv::Size>* eyes,const std::vector<TempEye>* tempeyes,int min,int max,int round,bool red);
std::pair<int,cv::Size> getsize(const cv::Mat& image,int xs,int ys,int& area,int& perimeter,std::set<int>& visited);
void ioGetdice(const cv::Mat& image,std::map<int,Dice>& top,std::map<int,Dice>& side,std::map<int,cv::Size>& black,std::map<int,cv::Size>& red);
void getdice(const cv::Mat* image,std::map<int,Dice>* top,std::map<int,Dice>* side,std::map<int,cv::Size>* eyes,double dist,double r_h,double r_aspect);
bool near_ellipse(int p,int p0,double height,double width,int cols);
void redeye2dice(std::map<int,Dice>& top,std::map<int,Dice>& side,std::map<int,cv::Size>& redeye);
void showdice(const char* winname,const cv::Mat* image,std::map<int, Dice>* top,std::map<int,Dice>* side);
void CountDice(const cv::Mat& image,std::map<int,Dice>& top,std::map<int,Dice>& side,double* rate);
void count(const cv::Mat& image,cv::Mat& counted_img,int dicesize[4], std::map<int,Dice>& dice,int top);
void sift(const std::map<int,Dice>& src,std::map<int,Dice>& dst,const std::map<int,Dice>& top,int cols);
double getperimeter(const cv::Mat& image,int x0,int y0);
void calcbyrate(const double* rate,double weight,int* num_size);
void onMouse_exclude(int event,int x,int y,int flag,void* param);

///////////////////////////////////////////////////////////////////////////////

//���W�ϊ��̏�����B
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

//���C��
int main(int argc, char* argv[])
{
	cv::Mat input,black,red;
	std::map<int,cv::Size> blackeye,redeye;
	std::map<int,Dice> side;
	std::map<int,Dice> top;
	std::string filename;
	double rate[3];
	int num_size[3];
	while(1)
	{
		int i=1;

		//�摜�Ǎ�
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

		//��˗l�̑O���������̕ӂ�...
		gamma(input,input);
		pre_pros_red(input,cv::Mat(),red);
		pre_pros_black(input,cv::Mat(),black);

		ioGeteye(input,black,red,blackeye,redeye);
		ioGetdice(input,top,side,blackeye,redeye);
		CountDice(input,top,side,rate);
	}
	cv::destroyAllWindows();
	std::string str;
	double weight;
	std::cout<<"Input Weight : ";

	std::getline(std::cin,str);
	weight=atof(str.c_str());

	calcbyrate(rate,weight,num_size);

	std::cout<<"Number of Dice. S: "<<num_size[0]<<" M: "<<num_size[1]<<" L: "<<num_size[2];

	std::cin.get();
	
	return 0;
}

//�T���̈���i��}�E�XI/O
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
		size->width = x1!=x2 ? abs(x1-x2) : data->first->cols;
		size->height = y1!=y2 ? abs(y1-y2) : data->first->rows;
		cv::rectangle(image,*size,cv::Scalar(0,0,0),1,CV_AA);
		cv::imshow(WINDOW_CROP,image);
	}
}

//�摜�؂蔲��
void crop(const cv::Mat& src,cv::Mat& dst)
{
	std::pair<const cv::Mat*,cv::Rect*> senddata;
	senddata.first=&src;
	senddata.second=new cv::Rect(0,0,src.cols,src.rows);
	cv::namedWindow(WINDOW_CROP,CV_WINDOW_NORMAL);
	cv::imshow(WINDOW_CROP,src);
	cv::setMouseCallback(WINDOW_CROP,onMouse_crop,&senddata);
	while(cv::waitKey()=='z')
	cv::destroyWindow(WINDOW_CROP);
	cv::Mat cropped(src,*senddata.second);
	dst=cropped.clone();
	if(dst.rows>960)
	{
		cv::resize(dst,dst,cv::Size2f((double)dst.cols/dst.rows*960,960));
	}
	if(dst.cols>1280)
	{
		cv::resize(dst,dst,cv::Size2f(1280,(double)dst.rows/dst.cols*1280));
	}
	cv::destroyWindow(WINDOW_CROP);
}

typedef struct data_gi
{
	const cv::Mat* src;
	std::map<int,cv::Size>* eyes;
	std::vector<TempEye>* tempeyes;
	int* min;
	int* max;
	int* round;
	data_gi(const cv::Mat* src,std::map<int,cv::Size>* eyes,std::vector<TempEye>* tempeyes,int* min,int* max,int* round);
}data_gi;

data_gi::data_gi(const cv::Mat* src,std::map<int,cv::Size>* eyes,std::vector<TempEye>* tempeyes,int* min,int* max,int* round)
{
	this->src=src;
	this->eyes=eyes;
	this->tempeyes=tempeyes;
	this->min=min;
	this->max=max;
	this->round=round;
}

void redeye_callback(int value,void* senddata)
{
	data_gi* data=reinterpret_cast<data_gi*>(senddata);
	geteye_second(data->src,data->eyes,data->tempeyes,*data->min,*data->max,*data->round,true);
}

void blackeye_callback(int value,void* senddata)
{
	data_gi* data=reinterpret_cast<data_gi*>(senddata);
	geteye_second(data->src,data->eyes,data->tempeyes,*data->min,*data->max,*data->round,false);
}

//�ڂ����m����ۂ�I/O����
void ioGeteye(const cv::Mat& image, const cv::Mat& black,const cv::Mat& red,std::map<int,cv::Size>& blackeye,std::map<int,cv::Size>& redeye)
{
	//�n�߂͊���l�T��
	cv::Mat dst=image.clone();
	std::vector<TempEye> tempblack,tempred;
	static int bmin=black.rows/160,bmax=black.rows/20,rmin=red.rows/160,rmax=red.rows/10,deg=50;//�f�t�H���g臒l

	geteye(black,dst,blackeye,tempblack,false);
	geteye(red,dst,redeye,tempred,true);

	data_gi sendred(&image,&redeye,&tempred,&rmin,&rmax,&deg);
	data_gi sendblack(&image,&blackeye,&tempblack,&bmin,&bmax,&deg);


	//臒l�ݒ�p�̃g���b�N�o�[�B��ݒu
	geteye_second(&image,&redeye,&tempred,rmin,rmax,deg,true);
	cv::namedWindow(WINDOW_EYE,CV_WINDOW_NORMAL);
	cv::createTrackbar("Red Min",WINDOW_EYE,&rmin,red.cols/10,redeye_callback,&sendred);
	cv::createTrackbar("Red Max",WINDOW_EYE,&rmax,red.cols/10,redeye_callback,&sendred);
	cv::createTrackbar("Roundness",WINDOW_EYE,&deg,100,redeye_callback,&sendred);
	while(cv::waitKey()!='z');
	cv::destroyWindow(WINDOW_EYE);

	geteye_second(&image,&blackeye,&tempblack,bmin,bmax,deg,false);
	cv::namedWindow(WINDOW_EYE,CV_WINDOW_NORMAL);
	cv::createTrackbar("Black Min",WINDOW_EYE,&bmin,black.cols/10,blackeye_callback,&sendblack);
	cv::createTrackbar("Black Max",WINDOW_EYE,&bmax,black.cols/10,blackeye_callback,&sendblack);
	cv::createTrackbar("Roundness",WINDOW_EYE,&deg,100,blackeye_callback,&sendblack);
	while(cv::waitKey()!='z');
	cv::destroyWindow(WINDOW_EYE);
}

//�ڂ𒊏o
void geteye(const cv::Mat& src,cv::Mat& dst,std::map<int,cv::Size>& eyes,std::vector<TempEye>& tempeyes,bool red)
{
	//�����ł�src�͔����̓�l�ł��鎖�ɒ���
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

			//�ċA����Ȃ�����dfs(depth first search)�ł�
			//dfs�œh��Ԃ����ŗ̈������
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

			//���ڂ͑S�ĖڂƂ��ď���
			//�~�`�x=4��s/l^2
			int h=ymax-ymin+1,w=xmax-xmin+1;
			double l=getperimeter(src,xs,ys);
			double d=12.566370614359173*s/(l*l);
			eye.first=(xmin+xmax)/2+(ymin+ymax)/2*src.cols;//�ڂ̒��S���W
			eye.second.height=h;
			eye.second.width=w;

			//�h��F�ݒ�&�ڂ̏W���ɑ}��
			//�Ԗډ���->��,�Ԗڏc��->��,->���ډ���->��,���ڏc��->��
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

//2��ڈȍ~�̖ڂ̒T��
void geteye_second(const cv::Mat* src,std::map<int,cv::Size>* eyes,const std::vector<TempEye>* tempeyes,int min,int max,int round,bool red)
{
	//1��ڂɓ����ڂ̏����g���č�������}��
	cv::Mat dst=src->clone();
	eyes->clear();
	for(unsigned int i=0;i<tempeyes->size();i++)
	{
		if(tempeyes->at(i).height>=min&&tempeyes->at(i).height<=max&&tempeyes->at(i).width>=min&&tempeyes->at(i).width<=max&&tempeyes->at(i).round>=(double)round/100)
		{
			eyes->insert(std::pair<int,cv::Size>::pair(tempeyes->at(i).location,cv::Size(tempeyes->at(i).width,tempeyes->at(i).height)));
			cv::ellipse(dst,p2cvp(tempeyes->at(i).location,dst.cols),cv::Size(tempeyes->at(i).width/2,tempeyes->at(i).height/2),0,0,360,cv::Scalar(red?0:255,tempeyes->at(i).height>tempeyes->at(i).width?255:0,red?255:0),-1);
		}
	}
	cv::imshow(WINDOW_EYE,dst);
}

//���������߂�
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
			//�����v���ɑ���
			equal=true;
			d=(d+1)%8;
			//�����F�̉�f��,�͈͊O�łȂ���?
			if(x+move[d][0]<0||x+move[d][0]>=image.cols||y+move[d][1]<0||y+move[d][1]>=image.rows||image.data[image.step*(y+move[d][1])+x+move[d][0]]==0){equal=false;}

			//�ړ�(�����F�̉�f�����߂Ĕ���������)
			if(equal)
			{
				int xb=x,yb=y;//�ړ��O��x,y���u��
				x+=move[d][0];//�ړ�
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
	
	//����=�΂�*2^0.5+4�ߖT�א�*1
	return diagonal*1.41421356237309505+perpendicular;
}

typedef struct data_exclude
{
	const cv::Mat* image;
	std::map<int,Dice>* top;
	std::map<int,Dice>* side;

}data_exclude;

typedef struct data_gd
{
	const cv::Mat* image;
	std::map<int,Dice>* top;
	std::map<int,Dice>* side;
	std::map<int,cv::Size>* black;
	int* dist;
	int* r_h;
	int* r_aspect;
	data_gd(const cv::Mat* image,std::map<int,Dice>* top,std::map<int,Dice>* side,std::map<int,cv::Size>* black,int* dist,int* r_h,int* r_aspect);
}data_gd;
data_gd::data_gd(const cv::Mat* image,std::map<int,Dice>* top,std::map<int,Dice>* side,std::map<int,cv::Size>* black,int* dist,int* r_h,int* r_aspect)
{
	this->image=image;
	this->top=top;
	this->side=side;
	this->black=black;
	this->dist=dist;
	this->r_h=r_h;
	this->r_aspect=r_aspect;
}

void getdice_callback(int value,void* senddata)
{
	data_gd* data=reinterpret_cast<data_gd*>(senddata);
	getdice(data->image,data->top,data->side,data->black,*(data->dist)/10.0,*(data->r_h)/10.0,*(data->r_aspect)/10.0);
}


void ioGetdice(const cv::Mat& image,std::map<int,Dice>& top,std::map<int,Dice>& side,std::map<int,cv::Size>& black,std::map<int,cv::Size>& red)
{
	static int dist=40,r_h=12,r_aspect=12;
	
	data_gd data_gd(&image,&top,&side,&black,&dist,&r_h,&r_aspect);
	cv::namedWindow(WINDOW_DICE,CV_WINDOW_NORMAL);
	getdice(&image,&top,&side,&black,(double)dist/10,(double)r_h/10,(double)r_aspect/10);
	cv::createTrackbar("Distance(*10)",WINDOW_DICE,&dist,80,getdice_callback,&data_gd);
	cv::createTrackbar("Height(*10)",WINDOW_DICE,&r_h,30,getdice_callback,&data_gd);
	cv::createTrackbar("Aspect(*10)",WINDOW_DICE,&r_aspect,30,getdice_callback,&data_gd);

	//���������[�U���ݒ�
	//�ݒ�\�͖ڂ̊Ԃ̋���,����,�c�����,��ƂȂ�ڂƂ̔�
	while(cv::waitKey()!='z');
		
	cv::destroyWindow(WINDOW_DICE);
	redeye2dice(top,side,red);


	//����(�Ԗڒǉ��ƌ댟�o�r��)
	cv::namedWindow(WINDOW_EXCLUDE,CV_WINDOW_NORMAL);
	showdice(WINDOW_EXCLUDE,&image,&top,&side);

	data_exclude data_ex;
	data_ex.image=&image;
	data_ex.top=&top;
	data_ex.side=&side;

	cv::setMouseCallback(WINDOW_EXCLUDE,onMouse_exclude,&data_ex);
	cv::waitKey();
	cv::destroyWindow(WINDOW_EXCLUDE);

}

//�ȉ~�͈͂ł̋ߖT����
bool near_ellipse(int p,int p0,double height,double width,int cols)
{
	int dx=p%cols-p0%cols,dy=p/cols-p0/cols;
	return hypot(height*dx,width*dy)<=height*width;
}

//�T�C�R�����o�{����
void getdice(const cv::Mat* image,std::map<int,Dice>* top,std::map<int,Dice>* side,std::map<int,cv::Size>* eyes,double dist,double r_h,double r_aspect)
{
	//�����ʂ̑傫���ō��W���߂��ڂ���̖ʂɂ܂Ƃ߂�
	cv::Mat temp_img=image->clone();
	std::set<int> visited;

	top->clear();side->clear();
	for(std::map<int,cv::Size>::iterator ibase=eyes->begin();ibase!=eyes->end();ibase++)
	{
		std::vector<std::pair<int,cv::Size> > temp;//�T�C�R���̖ڏW��
		std::vector<cv::Point> coordinate;//�T�C�R���̖ڍ��W
		if(visited.find((*ibase).first)==visited.end())
		{
			int h=(*ibase).second.height,w=(*ibase).second.width;
			int k=0;

			//�ߖT����(���̖ڂ�,��ڂ̏c,����dist�{�����ȉ~���ɂ��邩?)

			for(std::map<int,cv::Size>::iterator ite=eyes->begin();ite!=eyes->end();ite++)
			{
				if(visited.find((*ite).first)==visited.end()&&near_ellipse((*ite).first,(*ibase).first,h*dist,w*dist,image->cols))
				{
					//�傫������(�����̔䂪r_h�{�ȓॏc����r_aspect�{�ȓ�,�ڂ��������Ƃ��͕␳��)
					double rh=(*ibase).second.height<image->cols/160?r_h*2:r_h;
					double raspect=(*ibase).second.height<image->cols/200?r_aspect*2:r_aspect;

					int h1=h,h2=(*ite).second.height;
					int w1=w,w2=(*ite).second.width;
					double a1=(double)h/w,a2=(double)(*ite).second.height/(*ite).second.width;
					if(h1<h2)std::swap(h1,h2);
					if(w1<w2)std::swap(w1,w2);

					if((double)h1/h2<=rh && (double)w1/w2<=raspect)
					{
						temp.push_back((*ite));
						coordinate.push_back(p2cvp(((*ite).first),image->cols));
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
			
			
			//�ڂ�`��(����->��,�c��->��)
			for(std::vector<std::pair<int,cv::Size>>::iterator iteye=temp.begin();iteye!=temp.end();iteye++)
			{
				cv::Size size=cv::Size2f((*iteye).second.width/2.0,(*iteye).second.height/2.0);
				cv::ellipse(temp_img,p2cvp((*iteye).first,temp_img.cols),size,0,0,360,cv::Scalar(0,255,(*iteye).second.height>(*iteye).second.width*.8?255:0),-1);
				avh+=(*iteye).second.height;
				avw+=(*iteye).second.width;
			}
			//�ڂ̃T�C�Y�͍���,���̓�������
			//��ʂł�,�����͈ʒu�ɂ��ω����傫�������͕ω���������
			//���ʂł͂��̋t
			avh/=temp.size();
			avw/=temp.size();
			isize=avh>avw?avh:avw;
			aspect=(double)avh/avw;
			
			int h=ymax-ymin+avh*2,w=xmax-xmin+avw*2;

			if(aspect<=0.8)
			{
				top->insert(std::map<int,Dice>::value_type(cvp2p(box.center,temp_img.cols),Dice(h,w,isize,temp.size(),aspect,w)));
				cv::ellipse(temp_img,box.center,cv::Size(w/2,h/2),0,0,360,cv::Scalar(0,0,255),1,CV_AA);
			}
			else
			{
				side->insert(std::map<int,Dice>::value_type(cvp2p(box.center,temp_img.cols),Dice(h,w,isize,temp.size(),aspect,h)));
				cv::ellipse(temp_img,box.center,cv::Size(w/2,h/2),0,0,360,cv::Scalar(0,255,0),1,CV_AA);
			}
		}
	}
	cv::imshow(WINDOW_DICE,temp_img);
}

//�Ԗڂ�dice�ɓ����
void redeye2dice(std::map<int,Dice>& top,std::map<int,Dice>& side,std::map<int,cv::Size>& redeye)
{
	for(std::map<int,cv::Size>::iterator it=redeye.begin();it!=redeye.end();it++)
	{
		//1�̖ʂ̑傫���͖ڂ̑傫����2.5�{(����)
		Dice d((*it).second.height*5/2,(*it).second.width*5/2,std::max((*it).second.height,(*it).second.width)/2.5,1,(double)(*it).second.height/(*it).second.width,std::max((*it).second.height,(*it).second.width)*5/2);
		if(d.aspect<=0.8){top.insert(std::map<int,Dice>::value_type((*it).first,d));}
		else{side.insert(std::map<int,Dice>::value_type((*it).first,d));}
	}
}

//�T�C�R���S�ĕ\��
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

//�댟�o�r���̃}�E�XI/O param��mat+(top��side��pair)
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

		data_exclude* data=(data_exclude *)param;

		xmin=xmin>0?xmin:0;
		if(xmax>10000){xmax=0;}
		else{xmax=xmax<data->image->cols?xmax:data->image->cols;}
		ymin=ymin>0?ymin:0;
		if(ymax>10000){ymax=0;}
		else{ymax=ymax<data->image->rows?ymax:data->image->rows;}
		if(xmax<xmin)std::swap(xmax,xmin);
		if(ymax<ymin)std::swap(ymax,ymin);

		for(int y=ymin;y<=ymax;y++)
		{
			for(int x=xmin;x<=xmax;x++)
			{
				if(data->top->find(xy2p(x,y,data->image->cols))!=data->top->end())
				{
					data->top->erase(xy2p(x,y,data->image->cols));
				}
				if(data->side->find(xy2p(x,y,data->image->cols))!=data->side->end())
				{
					data->side->erase(xy2p(x,y,data->image->cols));
				}
			}
		}
		showdice(WINDOW_EXCLUDE,data->image,data->top,data->side);
	}
}

//�T�C�R�������グI/O
void CountDice(const cv::Mat& image,std::map<int,Dice>& top,std::map<int,Dice>& side,double* rate)
{
	int dicesize[4]={0};//�T�C�Y�ʂ̃T�C�R���������ɓ����
	cv::Mat counted_img=image.clone();
	std::map<int,Dice> onlyside;

	//��������
	count(image,counted_img,dicesize,top,255);
	count(image,counted_img,dicesize,side,0);


	//�ʂ̐��̔�
	for(int i=0;i<3;i++)
	{
		rate[i]=(double)dicesize[i]/dicesize[4];
	}
}

//�T�C�R�������グ�{���[�`��
void count(const cv::Mat& image,cv::Mat& counted_img,int dicesize[4],std::map<int,Dice>& dice,int top)
{
	//�ϑ��т𕪂���͔̂p�~���܂���.
	//���]�������Y(���̓z��270mm����)�ŎB�e����Ɖ��ߊ��������,�����傫���Ɍ����܂�.
	//���掩�^�ł����ʐ^���Ȃ�ł͂̒m��?����Ȃ̌���Ή���?

	do{
		cv::Mat temp_img;
		int maxsize=0;
		char basesize;
		bool found=false;
		std::map<int,Dice>::const_iterator pos=dice.end();

		if(dice.size()==0)return;
		if(pos!=dice.end())dice.erase(pos);

		//��ԑ傫��������������
		for(std::map<int,Dice>::const_iterator it=dice.begin();it!=dice.end();it++)
		{
			if((*it).second.eyesize>maxsize)
			{
				pos=it;
				maxsize=(*it).second.eyesize;
			}
		}
		//��̃T�C�Y��I�΂���(�������ł�L�T�C�Y����,M�T�C�Y�ȉ������Ȃ��Ƃ�����)
		temp_img=image.clone();
		cv::ellipse(temp_img,p2cvp((*pos).first,image.cols),cv::Size((*pos).second.w/2,(*pos).second.h/2),0,0,360,cv::Scalar(0,0,255),3,4);
		cv::namedWindow(WINDOW_SIZE,CV_WINDOW_NORMAL);
		cv::imshow(WINDOW_SIZE,temp_img);
		cv::destroyWindow(WINDOW_SIZE);
		basesize=cv::waitKey();

		//�傫������
		for(std::map<int,Dice>::iterator it=dice.begin();it!=dice.end();it++)
		{
			char size='\0';
			switch(basesize)
			{
			case 'l':
				if((*it).second.eyesize>=(*pos).second.eyesize*0.625)
				{
					size='l';
				}
				else if((*it).second.eyesize>=(*pos).second.eyesize*0.4)
				{
					size='m';
				}
				else if((*it).second.eyesize>=(*pos).second.eyesize*0.2)
				{
					size='s';
				}
				break;
			case 'm':
				if((*it).second.eyesize>=(*pos).second.eyesize*0.6)
				{
					size='m';
				}
				else if((*it).second.eyesize>=(*pos).second.eyesize*0.3)
				{
					size='s';
				}
				break;
			case 's':
				if((*it).second.eyesize>=(*pos).second.eyesize*0.7)
				{
					size='s';
				}
				break;
			}
			(*it).second.size=size;
			cv::Scalar color=cv::Scalar(0,0,0);
			bool flag=false;
			switch(size)//���L->magenta,���M->yellow,���S->cyan,����L->red,����M->green,����S->blue
			{
			case 'l':
				dicesize[2]++;
				color=cv::Scalar(0,0,255);
				break;
			case 'm':
				dicesize[1]++;
				color=cv::Scalar(0,255,0);
				break;
			case 's':
				dicesize[0]++;
				color=cv::Scalar(255,0,0);
				break;
			default:
				flag=true;
				break;
			}
			if(!flag)
			{
				dicesize[3]++;
				cv::ellipse(counted_img,p2cvp((*it).first,image.cols),cv::Size((*it).second.w/2,(*it).second.h/2),0,0,360,color,1,CV_AA);
			}
		}
		cv::namedWindow(WINDOW_COUNT,CV_WINDOW_NORMAL);
		cv::imshow(WINDOW_COUNT,counted_img);
		cv::destroyWindow(WINDOW_COUNT);

	}while(cv::waitKey()!='z');
}

void calcbyrate(const double* rate,double weight,int* num_size)
{
	double x=weight/(rate[0]*0.274+rate[1]*1.35+rate[2]*5.902);
	for(int i=0;i<3;i++)
	{
		num_size[i]=rate[i]*x;
	}
}
