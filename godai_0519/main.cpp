#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//���C�u�����̃����N�錾�����ǋC�ɂ��Ȃ��Ă������Ǝv���
#ifdef _DEBUG
    #pragma comment(lib,"opencv_core240d.lib")
    #pragma comment(lib,"opencv_imgproc240d.lib")
    #pragma comment(lib,"opencv_highgui240d.lib")
#else
    #pragma comment(lib,"opencv_core240.lib")
    #pragma comment(lib,"opencv_imgproc240.lib")
    #pragma comment(lib,"opencv_highgui240.lib")
#endif

//�Ԃ��G���A�𔲂��o���֐�
void red_area(cv::Mat& dst,const cv::Mat& hsv)
{
  cv::Mat channels[3],h_mask,c_mask;
  cv::split(hsv,channels);

  cv::threshold(channels[0],h_mask,160,255,cv::THRESH_BINARY); //�F�����ԕt�߂��ǂ���
  cv::threshold(channels[1],c_mask,150,255,cv::THRESH_BINARY); //�ʓx(�M�p��)���������ǂ���

  //�F�����ԕt�߂ŁC�ʓx����������(->dst)
  cv::bitwise_and(h_mask,c_mask,dst);

  //�����߂̔������������c��������
  cv::dilate(dst,dst,cv::Mat());
  return;
}

//�����G���A�𔲂��o��
void black_area(cv::Mat& dst,const cv::Mat& image)
{
  //RGB�ɕ���
  cv::Mat channels[3],r,g,b;
  cv::split(image,channels);
  
  //(�e�v�f��)�����Ȃ��Ƃ�
  cv::threshold(channels[0],r,50,255,cv::THRESH_BINARY);
  cv::threshold(channels[1],g,50,255,cv::THRESH_BINARY);
  cv::threshold(channels[2],b,50,255,cv::THRESH_BINARY);

  //(�e�v�f��)���]�����č����Ƃ�
  bitwise_not(r,r);
  bitwise_not(g,g);
  bitwise_not(b,b);

  //����(->dst)
  cv::bitwise_and(r,g,dst);
  cv::bitwise_and(b,dst,dst);

  //�����߂̔������������c��������
  cv::dilate(dst,dst,cv::Mat());
}

std::vector<std::vector<cv::Point>> contours;

//�}�E�X�N���b�N�p�֐�(�N���b�N����x�ɌĂ΂�܂�)
//cv::setMouseCallback�̃h�L�������g�ɒ���
void inactive_area_input(int event, int x, int y, int flags, void* param){
  static short start_x,start_y,end_x,end_y; // VC10��GCC��short��2�o�C�g�ŃR���p�C�������̂����p�D

  if(event == CV_EVENT_LBUTTONDOWN)
  {
    //�N���b�N�J�n���̍��W���i�[
    start_x = x; start_y = y;
    std::cout << "Click" << std::endl;
  }
  if(event == CV_EVENT_LBUTTONUP)
  {
    //�h���b�O�I�����̍��W���i�[
    end_x = x; end_y = y;

    //�h���b�O�󋵂̕\��
    std::cout << "UnClick" << std::endl;
    std::cout << "(" << start_x << "," << start_y << ") - (" << end_x << "," << end_y << ")" << std::endl;

    //main��senddata�Ƃ���param�ɗ���̂ŁC�|�C���^�ϊ�
    const std::pair<cv::Mat*,cv::Mat*> *data = reinterpret_cast<std::pair<cv::Mat*,cv::Mat*>*>(param);
    cv::Mat* out = data->second;

    //start������Cend���E���ɗ���悤�ɏC��
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

    //��ʊO�ւ̃N���b�N���C��
    start_x = std::max((short)0,start_x);
    start_y = std::max((short)0,start_y);
    end_x   = std::min((short)out->cols,end_x);
    end_y   = std::min((short)out->rows,end_y);

    //�C������
    std::cout << "(" << start_x << "," << start_y << ") - (" << end_x << "," << end_y << ")\n" << std::endl;

    //�I��͈͂̓h��Ԃ�
    for(int i = start_y; i < end_y; ++i)
      for(int j = start_x; j < end_x; ++j)
        out->at<unsigned char>(i,j) = 0;
    
    //�֊s���̍폜�ƍĒT��
    contours.clear();
    cv::findContours(*out,contours,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);

    //�㏑���\��
    cv::Mat temp = data->first->clone();
    cv::drawContours(temp,contours,-1,CV_RGB(0,0,255),2);
    cv::imshow("temp",temp);
  }
}

int main()
{
  //image��BGR�摜��荞�݁Chsv��HSV�`���ɕϊ��������̂��i�[
  //��ʂɃR���s���[�^�ň�����JPEG/BMP����BGR�ŕۑ�����Ă��āCRGB�ł͂Ȃ�
  cv::Mat image = cv::imread("img.jpg",1),hsv;
  cv::cvtColor(image,hsv,CV_BGR2HSV);
  
  //�����ς݂̊֐����R�[��
  cv::Mat red,black,out;
  red_area(red,hsv);
  black_area(black,image);
  bitwise_or(red,black,out); //�Ԃ����̂Ƃ���(->out)
  
  //�֊s�T��
  cv::findContours(out,contours,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);

  //tmp��image�����S�R�s�[(�N���[��)
  cv::Mat temp = image.clone();
  cv::drawContours(temp,contours,-1,CV_RGB(0,0,255),2); //temp�ɗ֊s�����o��
  
  //��ʂɕ\��
  cv::namedWindow("src");
  cv::namedWindow("temp");
  cv::imshow("src",image);  
  cv::imshow("temp",temp);
   
  std::pair<cv::Mat*,cv::Mat*> senddata = std::pair<cv::Mat*,cv::Mat*>(&image,&out); //setMouseCallback�œn�����f�[�^
  cv::setMouseCallback("temp",&inactive_area_input,&senddata); //�}�E�X�N���b�N����ݒ�
  
  char inputkey;
  while(inputkey = cv::waitKey(0), inputkey != 'q'); //q�����͂����܂őҋ@

  //
  // �����I�I
  //

  return 0;
}
