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
#define WINDOW_NUM 3				//OpenGLで生成するwindowの数
#define distance_size 165		//上側切り取り幅

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

//グローバル変数　openglのコールバック関数に値を渡す方法がわからない……
int shift;
float odom_liner_x;
cv::Mat srcImg;		//元画像保存Mat
cv::Mat copyImg;	//作業画像保存Mat
GLfloat lightpos[] = { 0.0, 0.0, 0.0, 1.0 };//ライトの位置
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
	
	//opengl:GLUT関連の初期化
 	glutInit(&argc,argv);
	for (int i = 0; i < WINDOW_NUM; i++)
		(*GLUT_INIT_Ary[i])(WinID[i], WindowName[i]);

	//メインループ
	while (ros::ok())
	{
		ros::spinOnce();
		capture.callOne();
		glutMainLoopEvent();								//OpenGLのイベントの開始
		idle(WinID);												//各display関数の呼び出し
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
			srcImg.copyTo(copyImg);			//元画像を作業用にコピー
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
 }

void GLUT_INIT_LEFT(int &ID, const char *name)							
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);		//描画方法の設定
	glutInitWindowPosition(WinPos[0].x, WinPos[0].y);			//window位置の設定
	glutInitWindowSize(WinSize.width,WinSize.height);	//windowサイズの設定
	ID = glutCreateWindow(name);									//windowに名前と番号を振り分け
	glEnable(GL_LIGHTING);											//光源設定をonにする
	glEnable(GL_LIGHT0);											//一つ目の光源
	glEnable(GL_DEPTH_TEST);										//陰面消去の設定
	glEnable(GL_BLEND);												//半透明設定をonにする
	glEnable(GL_NORMALIZE);											//法線ベクトルを自動的に正規化（頂点の光源に対する方向を決定して真っ黒なポリゴンになるのを防ぐ）

	glutDisplayFunc(display_left);	//windowにdisplayを割り当て
}
void GLUT_INIT_FRONT(int &ID, const char *name)							
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);		//描画方法の設定
	glutInitWindowPosition(WinPos[1].x, WinPos[1].y);			//window位置の設定
	glutInitWindowSize(WinSize.width,WinSize.height);	//windowサイズの設定
	ID = glutCreateWindow(name);									//windowに名前と番号を振り分け

	glutDisplayFunc(display_front);	//windowにdisplayを割り当て
}
void GLUT_INIT_RIGHT(int &ID, const char *name)							
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);		//描画方法の設定
	glutInitWindowPosition(WinPos[2].x, WinPos[2].y);			//window位置の設定
	glutInitWindowSize(WinSize.width,WinSize.height);	//windowサイズの設定
	ID = glutCreateWindow(name);									//windowに名前と番号を振り分け

	glutDisplayFunc(display_right);	//windowにdisplayを割り当て
}

//ウィンドウ更新用関数
void idle(int WinID[])
{
	for (int i = 0; i < WINDOW_NUM; ++i) {
		glutSetWindow(WinID[i]);	//更新するwindowのセット
		glutPostRedisplay();		//更新の実行
	}
}

//描画用関数ここから
void display_left(void)
{	
	cv::Mat Img=create(0,copyImg);																
																									//OpenGLでの描画設定、ここから
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);												//画面の色とデプスのバッファをクリア
	//以下一部不明
	glViewport(0, 0, WinSize.width, WinSize.height);										//viewportの設定
	glMatrixMode(GL_PROJECTION);																	//投影変換モード
	glLoadIdentity();																				//投影変換の変換行列を単位行列で初期化
	gluPerspective(30.0, (double)WinSize.width / (double)WinSize.height, 1.0, 1000.0);	//視界の決定
	glMatrixMode(GL_MODELVIEW);																		//モデルビュー変換行列の設定
	glLoadIdentity();																				//モデルビュー変換行列を単位行列で初期化
	gluLookAt(0.0, 0.0, 65.0, //カメラの座標
		8.0, 180.0, 0, // 注視点の座標
		0.0, 0.0, 1.0); // 画面の上方向を指すベクトル
	glLightfv(GL_LIGHT0, GL_POSITION, lightpos);													//ライトを当てる
	//CV_FRONT_FUNC(Img);
	// CV_CALL_FUNC(0, Img);																		//opencvの各処理を行う関数の呼び出し
	glDrawPixels(Img.cols, Img.rows, GL_RGB, GL_UNSIGNED_BYTE, Img.data);			//描画処理
	glClear(GL_DEPTH_BUFFER_BIT);																	//デプスバッファのクリア
	glutSwapBuffers();																				//画面を更新
}

void display_front(void)
{					
	cv::Mat Img= create(1, copyImg);																	
																									//OpenGLでの描画設定、ここから
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);												//画面の色とデプスのバッファをクリア
	glLoadIdentity();																				//単位行列の初期化
	//CV_CALL_FUNC(1, leftImg);																		//opencvの各処理を行う関数の呼び出し
	glDrawPixels(Img.cols, Img.rows, GL_RGB, GL_UNSIGNED_BYTE, Img.data);				//描画処理
	glutSwapBuffers();																				//画面を更新//OpenGLでの描画設定、ここまで
}
void display_right(void)
{					
	cv::Mat Img= create(2, copyImg);
																									//OpenGLでの描画設定、ここから
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);												//画面の色とデプスのバッファをクリア
	glLoadIdentity();																				//単位行列の初期化
	//CV_CALL_FUNC(1, leftImg);																		//opencvの各処理を行う関数の呼び出し
	glDrawPixels(Img.cols, Img.rows, GL_RGB, GL_UNSIGNED_BYTE, Img.data);				//描画処理
	glutSwapBuffers();																				//画面を更新//OpenGLでの描画設定、ここまで
}

cv::Mat create(int display,cv::Mat Img)
{
	cv::Mat create_Img(cv::Size(WinSize_default_width, WinSize_default_height), CV_8UC3, CV_RGB(0,0,0));
	cv::Mat tmp_Img(cv::Size(WinSize_default_width/90,WinSize_default_height),CV_8UC3,CV_RGB(0,0,0));
	cv::Mat roi;
	cv::Mat map_matrix;																								//変換行列保存用Mat
	cv::Point2f src_pnt[4];																				//変換前の頂点保存用変数
	cv::Point2f dst_pnt[4];																				//変換後の頂点保存用変数
	double radius=100;

	// radius=cam_Img_Size_Width*M_PI/4;																		//元画像の円周の1/4
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
		
		map_matrix = getPerspectiveTransform(src_pnt, dst_pnt);																//変換行列を求める
		cv::warpPerspective(Img, tmp_Img, map_matrix,tmp_Img.size());							//透視変換の実行

		tmp_Img.copyTo(roi);
	}
	if(display==1){
		create_Img=shift_view(shift,create_Img);
		create_Img=speed_view(odom_liner_x,create_Img);
	}

	flip(create_Img, create_Img, 0);								//Opencvの画像をOpenGLに合うように座標軸を変更
	cvtColor(create_Img, create_Img, cv::COLOR_BGR2RGB);				//Opencvの画像をOpenGLに合うように画像色を変更

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
		case 0:		//ニュートラル
			putText(Img, "N", pos, cv::FONT_HERSHEY_SIMPLEX, 3.0, CV_RGB(0, 255, 0), 10, CV_AA);
			break;
		case -1:	//リバース
			putText(Img, "R", pos, cv::FONT_HERSHEY_SIMPLEX, 3.0, CV_RGB(255, 127, 0), 10, CV_AA);
			break;
		case 1:	//ドライブ
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