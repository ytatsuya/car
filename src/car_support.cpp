#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include<std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <GL/glut.h>
#include <dirent.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <opencv2/core/cuda.hpp>
#include<GL/freeglut.h>
#include <stdlib.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include<ros/callback_queue.h>
#include <math.h>

#define WinPos_x_default 0
#define WinPos_y_default 0
#define WinSize_default_width 1920
#define WinSize_default_height 1080
#define cam_Img_Size_Width 1024
#define cam_Img_Size_Height 1024
#define WINDOW_NUM 3				//OpenGL�Ő�������window�̐�
#define distance_size 165		//�㑤�؂��蕝

void GLUT_INIT_LEFT(int&, const char*);
void GLUT_INIT_FRONT(int&, const char*);
void GLUT_INIT_RIGHT(int&, const char*);

void idle(int*);
void display_left(void);
void display_front(void);
void display_right(void);
cv::Mat create(int,cv::Mat);
float circle_x(int ,int ,int );
float circle_y(int ,int ,int );
cv::Mat shift_view(int ,cv::Mat&);
cv::Mat speed_view(float ,cv::Mat&);

class view_param{
	public:
		view_param();
	private:
		void shiftCallback(const std_msgs::Int32::ConstPtr& sub_shift);
		void odomCallback(const nav_msgs::Odometry::ConstPtr& sub_odom);
		ros::NodeHandle nh;
		ros::Subscriber shift_sub_,odom_sub_;
};

class Omnidirectional_Img{
	public:
		Omnidirectional_Img();
		void callOne(void);
	private:
		void imageCallback(const sensor_msgs::ImageConstPtr& msg);
		ros::NodeHandle nh;
		ros::CallbackQueue queue;
		ros::Subscriber image_sub;
};

//�O���[�o���ϐ��@opengl�̃R�[���o�b�N�֐��ɒl��n�����@���킩��Ȃ��c�c
int shift;
float odom_liner_x;
cv::Mat srcImg;		//���摜�ۑ�Mat
cv::Mat copyImg;	//��Ɖ摜�ۑ�Mat
GLfloat lightpos[] = { 0.0, 0.0, 0.0, 1.0 };//���C�g�̈ʒu
cv::Size2i WinSize={WinSize_default_width,WinSize_default_height};
cv::Point2i WinPos[WINDOW_NUM]={{WinPos_x_default,WinPos_y_default},{WinPos_x_default+WinSize.width,WinPos_y_default},{WinPos_x_default+(WinSize.width*2),WinPos_y_default}};

void(*GLUT_INIT_Ary[])(int&, const char*) = { GLUT_INIT_LEFT, GLUT_INIT_FRONT,GLUT_INIT_RIGHT};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"car_support");

	Omnidirectional_Img capture;
	view_param view_param;
	int WinID[WINDOW_NUM];
	const char *WindowName[WINDOW_NUM] = { "leftImg","frontImg","rightImg" };
	
	//opengl:GLUT�֘A�̏�����
 	glutInit(&argc,argv);
	for (int i = 0; i < WINDOW_NUM; i++)
		(*GLUT_INIT_Ary[i])(WinID[i], WindowName[i]);

	//���C�����[�v
	while (ros::ok())
	{
		ros::spinOnce();
		capture.callOne();
		glutMainLoopEvent();								//OpenGL�̃C�x���g�̊J�n
		idle(WinID);												//�edisplay�֐��̌Ăяo��
	}

	return 0;
}
view_param::view_param(){
	ros::NodeHandle nh("~");
	shift_sub_=nh.subscribe("/shift",1,&view_param::shiftCallback,this);
	odom_sub_=nh.subscribe("/odom",1,&view_param::odomCallback,this);
}

void view_param::shiftCallback(const std_msgs::Int32::ConstPtr& sub_shift){
	shift=sub_shift->data;
}

void view_param::odomCallback(const nav_msgs::Odometry::ConstPtr& sub_odom){
	odom_liner_x=sub_odom->twist.twist.linear.x;
}

Omnidirectional_Img::Omnidirectional_Img(){
	ros::NodeHandle nh("~");
	// image_transport::ImageTransport it(nh);
	// image_transport::Subscriber image_sub = it.subscribe("/img_publisher/image", 10, &Omnidirectional_Img::imageCallback,this);
	nh.setCallbackQueue(&queue);
	image_sub = nh.subscribe("/cv_bridge_omnidirectional_cam/image", 10, &Omnidirectional_Img::imageCallback,this);
}
void Omnidirectional_Img::callOne(void){
	queue.callOne(ros::WallDuration(10));
}

void Omnidirectional_Img::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try {
		srcImg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
		if(!srcImg.empty())
			srcImg.copyTo(copyImg);			//���摜����Ɨp�ɃR�s�[
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
 }

void GLUT_INIT_LEFT(int &ID, const char *name)							
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);		//�`����@�̐ݒ�
	glutInitWindowPosition(WinPos[0].x, WinPos[0].y);			//window�ʒu�̐ݒ�
	glutInitWindowSize(WinSize.width,WinSize.height);	//window�T�C�Y�̐ݒ�
	ID = glutCreateWindow(name);									//window�ɖ��O�Ɣԍ���U�蕪��
	glEnable(GL_LIGHTING);											//�����ݒ��on�ɂ���
	glEnable(GL_LIGHT0);											//��ڂ̌���
	glEnable(GL_DEPTH_TEST);										//�A�ʏ����̐ݒ�
	glEnable(GL_BLEND);												//�������ݒ��on�ɂ���
	glEnable(GL_NORMALIZE);											//�@���x�N�g���������I�ɐ��K���i���_�̌����ɑ΂�����������肵�Đ^�����ȃ|���S���ɂȂ�̂�h���j

	glutDisplayFunc(display_left);	//window��display�����蓖��
}
void GLUT_INIT_FRONT(int &ID, const char *name)							
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);		//�`����@�̐ݒ�
	glutInitWindowPosition(WinPos[1].x, WinPos[1].y);			//window�ʒu�̐ݒ�
	glutInitWindowSize(WinSize.width,WinSize.height);	//window�T�C�Y�̐ݒ�
	ID = glutCreateWindow(name);									//window�ɖ��O�Ɣԍ���U�蕪��

	glutDisplayFunc(display_front);	//window��display�����蓖��
}
void GLUT_INIT_RIGHT(int &ID, const char *name)							
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);		//�`����@�̐ݒ�
	glutInitWindowPosition(WinPos[2].x, WinPos[2].y);			//window�ʒu�̐ݒ�
	glutInitWindowSize(WinSize.width,WinSize.height);	//window�T�C�Y�̐ݒ�
	ID = glutCreateWindow(name);									//window�ɖ��O�Ɣԍ���U�蕪��

	glutDisplayFunc(display_right);	//window��display�����蓖��
}

//�E�B���h�E�X�V�p�֐�
void idle(int WinID[])
{
	for (int i = 0; i < WINDOW_NUM; ++i) {
		glutSetWindow(WinID[i]);	//�X�V����window�̃Z�b�g
		glutPostRedisplay();		//�X�V�̎��s
	}
}

//�`��p�֐���������
void display_left(void)
{	
	cv::Mat Img=create(0,copyImg);																
																									//OpenGL�ł̕`��ݒ�A��������
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);												//��ʂ̐F�ƃf�v�X�̃o�b�t�@���N���A
	//�ȉ��ꕔ�s��
	glViewport(0, 0, WinSize.width, WinSize.height);										//viewport�̐ݒ�
	glMatrixMode(GL_PROJECTION);																	//���e�ϊ����[�h
	glLoadIdentity();																				//���e�ϊ��̕ϊ��s���P�ʍs��ŏ�����
	gluPerspective(30.0, (double)WinSize.width / (double)WinSize.height, 1.0, 1000.0);	//���E�̌���
	glMatrixMode(GL_MODELVIEW);																		//���f���r���[�ϊ��s��̐ݒ�
	glLoadIdentity();																				//���f���r���[�ϊ��s���P�ʍs��ŏ�����
	gluLookAt(0.0, 0.0, 65.0, //�J�����̍��W
		8.0, 180.0, 0, // �����_�̍��W
		0.0, 0.0, 1.0); // ��ʂ̏�������w���x�N�g��
	glLightfv(GL_LIGHT0, GL_POSITION, lightpos);													//���C�g�𓖂Ă�
	//CV_FRONT_FUNC(Img);
	// CV_CALL_FUNC(0, Img);																		//opencv�̊e�������s���֐��̌Ăяo��
	glDrawPixels(Img.cols, Img.rows, GL_RGB, GL_UNSIGNED_BYTE, Img.data);			//�`�揈��
	glClear(GL_DEPTH_BUFFER_BIT);																	//�f�v�X�o�b�t�@�̃N���A
	glutSwapBuffers();																				//��ʂ��X�V
}

void display_front(void)
{					
	cv::Mat Img= create(1, copyImg);																	
																									//OpenGL�ł̕`��ݒ�A��������
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);												//��ʂ̐F�ƃf�v�X�̃o�b�t�@���N���A
	glLoadIdentity();																				//�P�ʍs��̏�����
	//CV_CALL_FUNC(1, leftImg);																		//opencv�̊e�������s���֐��̌Ăяo��
	glDrawPixels(Img.cols, Img.rows, GL_RGB, GL_UNSIGNED_BYTE, Img.data);				//�`�揈��
	glutSwapBuffers();																				//��ʂ��X�V//OpenGL�ł̕`��ݒ�A�����܂�
}
void display_right(void)
{					
	cv::Mat Img= create(2, copyImg);
																									//OpenGL�ł̕`��ݒ�A��������
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);												//��ʂ̐F�ƃf�v�X�̃o�b�t�@���N���A
	glLoadIdentity();																				//�P�ʍs��̏�����
	//CV_CALL_FUNC(1, leftImg);																		//opencv�̊e�������s���֐��̌Ăяo��
	glDrawPixels(Img.cols, Img.rows, GL_RGB, GL_UNSIGNED_BYTE, Img.data);				//�`�揈��
	glutSwapBuffers();																				//��ʂ��X�V//OpenGL�ł̕`��ݒ�A�����܂�
}

cv::Mat create(int display,cv::Mat Img)
{
	cv::Mat create_Img(cv::Size(WinSize_default_width, WinSize_default_height), CV_8UC3, CV_RGB(0,0,0));
	cv::Mat tmp_Img(cv::Size(WinSize_default_width/90,WinSize_default_height),CV_8UC3,CV_RGB(0,0,0));
	cv::Mat roi;
	cv::Mat map_matrix;																								//�ϊ��s��ۑ��pMat
	cv::Point2f src_pnt[4];																				//�ϊ��O�̒��_�ۑ��p�ϐ�
	cv::Point2f dst_pnt[4];																				//�ϊ���̒��_�ۑ��p�ϐ�
	double radius=100;

	// radius=cam_Img_Size_Width*M_PI/4;																		//���摜�̉~����1/4
	// radius=radius/16*9;
	// radius=cam_Img_Size_Width-radius;
	
	for(int i=0;i<90;++i){
		cv::Mat roi(create_Img,cv::Rect(WinSize_default_width/90*i,0,WinSize_default_width/90,WinSize_default_height));
		
		src_pnt[0]={circle_x(cam_Img_Size_Width/2,radius,135+90*display+i),circle_y(cam_Img_Size_Width/2,radius,135+90*display+i)};
		src_pnt[1]={circle_x(cam_Img_Size_Width/2,radius,135+90*display+i+1),circle_y(cam_Img_Size_Width/2,radius,135+90*display+i+1)};
		src_pnt[2]={circle_x(cam_Img_Size_Width/2,cam_Img_Size_Width/2,135+90*display+i+1),circle_y(cam_Img_Size_Width/2,cam_Img_Size_Width/2,135+90*display+i+1)};
		src_pnt[3]={circle_x(cam_Img_Size_Width/2,cam_Img_Size_Width/2,135+90*display+i),circle_y(cam_Img_Size_Width/2,cam_Img_Size_Width/2,135+90*display+i)};
		dst_pnt[0]={0,0};
		dst_pnt[1]={WinSize_default_width/90,0};
		dst_pnt[2]={WinSize_default_width/90,WinSize_default_height};
		dst_pnt[3]={0,WinSize_default_height};
		cv::line(srcImg,src_pnt[0],src_pnt[0],CV_RGB(0, 255, 0),1);
		cv::line(srcImg,src_pnt[1],src_pnt[1],CV_RGB(0, 255, 0),1);
		
		map_matrix = getPerspectiveTransform(src_pnt, dst_pnt);																//�ϊ��s������߂�
		cv::warpPerspective(Img, tmp_Img, map_matrix,tmp_Img.size());							//�����ϊ��̎��s

		tmp_Img.copyTo(roi);
	}
	if(display==1){
		create_Img=shift_view(shift,create_Img);
		create_Img=speed_view(odom_liner_x,create_Img);
	}

	flip(create_Img, create_Img, 0);								//Opencv�̉摜��OpenGL�ɍ����悤�ɍ��W����ύX
	cvtColor(create_Img, create_Img, cv::COLOR_BGR2RGB);				//Opencv�̉摜��OpenGL�ɍ����悤�ɉ摜�F��ύX

	return create_Img;
}

float circle_x(int a,int radius,int theta){
	float ans;
	
	ans=a+radius*std::cos(theta*M_PI/180);
	
	return ans;
}

float circle_y(int b,int radius,int theta){
	float ans;

	ans=b+radius*std::sin(theta*M_PI/180);

	return ans;
}

cv::Mat shift_view(int shift,cv::Mat& Img){
	cv::Point2i pos={WinSize.width/2-200, WinSize.height-50};
	switch (shift){
		case 0:		//�j���[�g����
			putText(Img, "N", pos, cv::FONT_HERSHEY_SIMPLEX, 3.0, CV_RGB(0, 255, 0), 10, CV_AA);
			break;
		case -1:	//���o�[�X
			putText(Img, "R", pos, cv::FONT_HERSHEY_SIMPLEX, 3.0, CV_RGB(255, 127, 0), 10, CV_AA);
			break;
		case 1:	//�h���C�u
			putText(Img, "D", pos, cv::FONT_HERSHEY_SIMPLEX, 3.0, CV_RGB(255, 255, 255), 10, CV_AA);
			break;
		default:
			putText(Img, "E", pos, cv::FONT_HERSHEY_SIMPLEX, 3.0, CV_RGB(255, 0, 0), 10, CV_AA);
			break;
	}	
	
	return Img;
}

cv::Mat speed_view(float odom_liner_x,cv::Mat& Img){
	std::string str;
	str=cv::format("%03.0fm/h",odom_liner_x*360);
	putText(Img, str, cv::Point2i(WinSize.width/2+100, WinSize.height-50), cv::FONT_HERSHEY_SIMPLEX, 3.0, CV_RGB(255, 255, 255), 10, CV_AA);

	return Img;
}