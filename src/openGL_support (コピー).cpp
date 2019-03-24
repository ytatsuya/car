#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
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

//定数
#define LEFT_U (215+robot_trimming_point[0][0])					//ロボット左上x座標
#define UP_V (160+robot_trimming_point[0][1])					//ロボット左上y座標
#define RIGHT_U (415+robot_trimming_point[1][0])					//ロボット右下x座標
#define DOWN_V (280+robot_trimming_point[1][1])					//ロボット右下y座標
#define Imgwidth 640								//キャプチャ画像の横サイズ
#define Imgheight 480								//キャプチャ画像の縦サイズ
#define FRONT_WINDOW_WIDTH 1920		//前面画像表示windowの横サイズ
#define FRONT_WINDOW_HEIGHT 1080	//前面画像表示windowの縦サイズ
#define LEFT_WINDOW_WIDTH 960		//左面画像表示windowの横サイズ
#define LEFT_WINDOW_HEIGHT 1080		//左面画像表示windowの縦サイズ
#define RIGHT_WINDOW_WIDTH 960		//右面画像表示windowの横サイズ
#define RIGHT_WINDOW_HEIGHT 1080	//右面画像表示windowの縦サイズ
#define BACK_WINDOW_WIDTH 1024		//後面画像表示windowの横サイズ
#define BACK_WINDOW_HEIGHT 768		//後面画像表示windowの縦サイズ
#define WINDOW_NUM 5				//OpenGLで生成するwindowの数
#define line_distance_default 0
#define window5_ratio_default 1.5
#define turn_scale 1.0

//各種関数のプロトタイプ宣言

int turn_direction_mode(void);
void save_param(void);
void reset_mode(void);
void speed_change(void);
void robot_trimming_size_change(int);
void line_distance_change(void);
void line_point_set(void);
cv::Point2d Bezier_curve(cv::Point2d,cv::Point2d,double);
void one_updown(double& ,int,int&,double);

//OpenCV
void CV_FRONT_FUNC(cv::Mat&);
void CV_CALL_FUNC(int, cv::Mat&);
void tmpImgcreate(cv::Mat&);
cv::Mat warp(int, cv::Mat);
cv::Mat Dividewarp(int, cv::Mat);
void cv_fixed_line(cv::Mat&);
void cv_mode_change(int&);
void cv_move_line(cv::Mat&);
void cv_move_elipse(double, cv::Mat&,int);
void gain(double&, cv::Mat&);
void front_scale_change(double&,cv::Mat&);
void cv_mode(cv::Mat&);
void cv_turn_mode(cv::Mat&);
void cv_draw_target(cv::Mat&);
void cv_save_img(int, cv::Mat);
void cv_turn_mat(cv::Mat&, double);
void turn_change(void);

//OpenGL
void mainLoop(void);
void GLUT_INIT1(int&, const char*);
void GLUT_INIT2(int&, const char*);
void GLUT_INIT3(int&, const char*);
void GLUT_INIT4(int&, const char*);
void GLUT_CALL_FUNC1(void);
void GLUT_CALL_FUNC2(void);
void GLUT_CALL_FUNC3(void);
void GLUT_CALL_FUNC4(void);
void GLUT_CALL_FUNC5(void);
void display1(void);
void display2(void);
void display3(void);
void display4(void);
void idle(void);
void GLUT_INIT5(int&, const char*);
void display5(void);
void resize1(int ,int);
void resize2(int ,int);
void resize3(int ,int);
void resize4(int ,int);
void resize5(int ,int);

//joystic動作判定
bool joystick_buttons_save(void);

void data_save(int64, int64);

//OpenGL初期化用関数ポインタ
void(*GLUT_INIT_Ary[])(int&, const char*) = { GLUT_INIT1,GLUT_INIT2,GLUT_INIT3,GLUT_INIT4,GLUT_INIT5};
void(*GLUT_CALL_FUNC_Ary[])() = { GLUT_CALL_FUNC1,GLUT_CALL_FUNC2,GLUT_CALL_FUNC3,GLUT_CALL_FUNC4,GLUT_CALL_FUNC5 };

//グローバル変数
int WinID[WINDOW_NUM];				//生成したwindowを管理する配列
const char *WindowName[] = { "frontImg","leftImg","rightImg","backImg","aroundImg" };		//生成したwindowの名前を管理する配列アドレス
int face[] = { cv::FONT_HERSHEY_SIMPLEX, cv::FONT_HERSHEY_PLAIN, cv::FONT_HERSHEY_DUPLEX, cv::FONT_HERSHEY_COMPLEX,			//opencvの文字フォントデータ用配列
cv::FONT_HERSHEY_TRIPLEX, cv::FONT_HERSHEY_COMPLEX_SMALL, cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
cv::FONT_HERSHEY_SCRIPT_COMPLEX, cv::FONT_ITALIC };
cv::Mat srcImg;		//元画像保存Mat
cv::Mat copyImg;	//作業画像保存Mat
int mode = 0;						//操作モード保存用変数
GLfloat red[] = { 1.0,0.0,0.0,1.0 };
GLfloat lightpos[] = { 0.0, 0.0, 0.0, 1.0 };//ライトの位置
int line_mode=0;
int Buttons_data[13]={};
int Rxyz_data[3]={};
bool low_speed=0;
bool display5_view_mode=0;
double robot_trimming_point[2][2]={};
bool window_change_mode[4]={};
int window_size[4][2]={};
double line_distance=line_distance_default;
cv::Point2d left_line_points[3];
cv::Point2d right_line_points[3];
double gain_k=1,front_scale=1;
double window5_ratio=window5_ratio_default;
const cv::Point2d windowPos[WINDOW_NUM] = {	//初期window位置の決定
	{1920,0},
	{970,0},
	{3840,0},
	{2364,1080},
	{0,0}
};

//データ管理用
std::ofstream fout("/home/mouse/data/0115_data.csv");		//データを出力するファイルの作成

class joydata{
	public:
	joydata();
	double X;
	double Y;
	double Z;
	double Rx;
	double Ry;
	double Rz;
	double slider;
	int pov[4];
	int Buttons[13];
};

joydata::joydata()
:X(0),Y(0),Z(0),Rx(0),Ry(0),Rz(0),slider(0){
	for(int i=0;i<13;++ i){
		if(i<4)
			pov[i]=0;
		Buttons[i]=0;
	}
}

joydata joyinput={};

class joystick{
	public:
		double X;
		double Y;
		double Z;
		double Rx;
		double Ry;
		double Rz;
		double slider;
		int pov[4];
		int Buttons[13];
		joystick();
		
	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		ros::NodeHandle nh;
		int vel_linear,vel_angular,vel_viceangular;
		double l_scale_,a_scale_;
		ros::Subscriber joy_sub_;
};

joystick::joystick():vel_linear(1),vel_angular(0),l_scale_(1), a_scale_(1), vel_viceangular(2)
{
	joy_sub_=nh.subscribe<sensor_msgs::Joy>("joy",10,&joystick::joyCallback,this);
}

void joystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	if(low_speed){
		a_scale_=0.5;
		l_scale_=0.5;
	}
	else {
		a_scale_=1;
		l_scale_=1;
	}
	joyinput.X=l_scale_*joy->axes[vel_linear];
	switch (mode)
	{
		case 1:
			joyinput.Y=a_scale_*joy->axes[vel_viceangular];
			break;
		
		case 2:
			if(joyinput.X==0)
				joyinput.Y=a_scale_*joy->axes[2];
			else joyinput.Y=a_scale_*joy->axes[0];
			break;
		
		case 0:
		default:
			joyinput.Y=a_scale_*joy->axes[vel_angular];
	}

	joyinput.Rx=joy->axes[5];
	joyinput.Ry=joy->axes[4];

	if(joy->axes[5]==-1){
		joyinput.X*=-1;
		joyinput.Y*=-1;
	}

	for(int i=0;i<13;++i)
		joyinput.Buttons[i]=joy->buttons[i];
	
	ROS_INFO_STREAM("("<<joyinput.X<<""<<joyinput.Y<<")");
}

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

int main(int argc,char** argv)
{
	ros::init(argc,argv,"openGL_support");

	Omnidirectional_Img capture;
	joystick a;

	//opengl:GLUT関連の初期化
 	glutInit(&argc,argv);
	 for (int i = 0; i < WINDOW_NUM; i++)
	{
		(*GLUT_INIT_Ary[i])(WinID[i], WindowName[i]);
		(*GLUT_CALL_FUNC_Ary[i])();
	}

	//メインループ
	while (ros::ok())
	{
		int64 start = cv::getTickCount();						//ローカルタイムの取得
		
		ros::spinOnce();

		capture.callOne();
		
		if(!joyinput.Buttons[6]&&!joyinput.Buttons[8]&&!joyinput.Buttons[10]&&!joyinput.Buttons[7]&&!joyinput.Buttons[9]){
			turn_change();
			speed_change();
			cv_mode_change(mode);					//モードの決定
		}

		if(joyinput.Buttons[6])
			robot_trimming_size_change(1);
		if(joyinput.Buttons[8])
			robot_trimming_size_change(-1);
		if(joyinput.Buttons[6]||joyinput.Buttons[8])
			tmpImgcreate(copyImg);							//tmpImgを作る
		if(joyinput.Buttons[10])
			line_distance_change();

		if(joyinput.Buttons[1])
			reset_mode();

		line_point_set();

		glutMainLoopEvent();								//OpenGLのイベントの開始
		idle();												//各display関数の呼び出し
		
		std::cout<<mode<<std::endl;
		int64 end = cv::getTickCount();						//ローカルタイムの取得
		data_save(start, end);								//データの保存
		if (joystick_buttons_save())
		{
			fout.close();									//ファイルの保存
			srcImg.copyTo(copyImg);
			tmpImgcreate(copyImg);							//tmpImgを作る
			imwrite("/home/mouse/data/srcImg.png", srcImg);					//srcImgの保存
			imwrite("/home/mouse/data/tmpImg.png", copyImg);					//copyImgの保存
			save_param();
			exit(0);										//ループを抜ける
		}
	}

	return 0;
}

int turn_direction_mode(void){
	if (joyinput.Rx==1)
		return 0;
	else if(joyinput.Rx==-1)
		return 1;
	else if(joyinput.Ry==-1)
		return 2;
	else if(joyinput.Ry==1)
		return 3;
}

void speed_change(void){
	if(joyinput.Buttons[2]&&!Buttons_data[2]&&!low_speed){
		++low_speed;
		++Buttons_data[2];
	}
	else if(joyinput.Buttons[2]&&Buttons_data[2]==0){
		low_speed=0;
		++Buttons_data[2];
	}
	else if(!joyinput.Buttons[2])
		Buttons_data[2]=0;
}

Omnidirectional_Img::Omnidirectional_Img(){
	ros::NodeHandle nh("~");
	// image_transport::ImageTransport it(nh);
	// image_transport::Subscriber image_sub = it.subscribe("/img_publisher/image", 10, &Omnidirectional_Img::imageCallback,this);
	nh.setCallbackQueue(&queue);
	image_sub = nh.subscribe("/cv_bridge_omnidirectional_cam/image", 10, &Omnidirectional_Img::imageCallback,this);
}
void Omnidirectional_Img::callOne(void){
	queue.callOne(ros::WallDuration(100));
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

void save_param(void){
	std::ofstream ofs ("/home/mouse/data/last_param.csv");		//データを出力するファイルの作成
	ofs<<gain_k<<std::endl;
	ofs<<front_scale<<std::endl;
	for(int i=0;i<2;++i)
		for(int j=0;j<2;++j)
			ofs<<robot_trimming_point[i][j]<<std::endl;;
	ofs<<line_distance<<std::endl;
	ofs.close();
}
void reset_mode(void){
	std::ifstream ifs("/home/mouse/data/last_param.csv");
	mode=0;
	low_speed=0;
	ifs>>gain_k;
	ifs>>front_scale;
	for(int i=0;i<2;++i)
		for(int j=0;j<2;++j)
			ifs>>robot_trimming_point[i][j];
	ifs>>line_distance;
	ifs.close();
}

void robot_trimming_size_change(int up_down){
	if(joyinput.Buttons[4])
		robot_trimming_point[0][0]-=up_down;
	if(joyinput.Buttons[2])
		robot_trimming_point[0][1]-=up_down;
	if(joyinput.Buttons[5])
		robot_trimming_point[1][0]+=up_down;
	if(joyinput.Buttons[3])
		robot_trimming_point[1][1]+=up_down;
}

void line_distance_change(void){
	if(joyinput.Buttons[4])
		++line_distance;
	if(joyinput.Buttons[2])
		--line_distance;
}

void line_point_set(void){
	double left_start=UP_V,right_start=UP_V;
	left_start-=((UP_V/LEFT_U)*line_distance);
	right_start-=((UP_V/(srcImg.cols-RIGHT_U))*line_distance);
	left_line_points[0]={LEFT_U-line_distance, left_start};
	right_line_points[0]={RIGHT_U+line_distance,UP_V-((UP_V/(srcImg.cols-RIGHT_U))*line_distance)};
	left_line_points[1]={LEFT_U-line_distance, (1-joyinput.X*front_scale)*left_start};
	right_line_points[1]={RIGHT_U+line_distance, (1-joyinput.X*front_scale)*right_start};
	left_line_points[2]={(1-joyinput.Y*gain_k)*(LEFT_U-line_distance), (1-joyinput.X*front_scale)*left_start};
	right_line_points[2]={(RIGHT_U+line_distance)-(joyinput.Y*gain_k*(RIGHT_U-line_distance)), (1-joyinput.X*front_scale)*right_start};
}

cv::Point2d Bezier_curve(cv::Point2d start_point,cv::Point2d end_point,double t){
	cv::Point2d ans;
	ans.x=(1-t)*start_point.x+t*end_point.x;
	ans.y=(1-t)*start_point.y+t*end_point.y;

	return ans;
}

void one_updown(double& change,int Buttons,int& data,double up_down){
	if(Buttons&&data==0&&change==0){
		change+=up_down;
		++data;
	}
	else if(!Buttons)
		data=0;
}

//OpenCV関数のまとめ
//OpenCVでの処理用CALL関数
void CV_FRONT_FUNC(cv::Mat &Img)
{
		if(line_mode==0||line_mode==1){
			if(line_mode==0)
				cv_mode(Img);						//画面に操作モードを表示
			if(low_speed)
				putText(Img, cv::format("Low Speed Mode"), cv::Point(window_size[0][0]-420, 100), face[3], 1.0, CV_RGB(180, 180, 255), 4, CV_AA);
			if ((joyinput.Buttons[3] || joyinput.Buttons[5]||joyinput.Buttons[4]||joyinput.Buttons[2])&&!joyinput.Buttons[6]&&!joyinput.Buttons[8]&&!joyinput.Buttons[10]&&joyinput.Buttons[7]&&!joyinput.Buttons[9]){
				if(joyinput.Buttons[3] || joyinput.Buttons[5])
					gain(gain_k,Img);							//表示ゲインの変更
				else front_scale_change(front_scale,Img);
			}
		}
		if(line_mode!=0)
			cv_turn_mode(Img);
		if ((joyinput.Buttons[3] || joyinput.Buttons[5]||joyinput.Buttons[0])&&!joyinput.Buttons[6]&&!joyinput.Buttons[8]&&!joyinput.Buttons[10]&&!joyinput.Buttons[7]&&!joyinput.Buttons[9])
			cv_draw_target(Img);					//joystickのボタンに合わせてターゲットの状態を表示
}

void CV_CALL_FUNC(int ID, cv::Mat &Img)
{
	if (joystick_buttons_save())					//保存ボタンが押されたかを判定
		cv_save_img(ID, Img);						//画像の保存
	flip(Img, Img, 0);								//Opencvの画像をOpenGLに合うように座標軸を変更
	cvtColor(Img, Img, cv::COLOR_BGR2RGB);				//Opencvの画像をOpenGLに合うように画像色を変更
}
//切り取り範囲を表示するtmpImgを作る関数
void tmpImgcreate(cv::Mat &tmp)
{
	rectangle(tmp, cv::Point(LEFT_U, UP_V), cv::Point(RIGHT_U, DOWN_V), CV_RGB(0, 0, 255), 2, 4);		//ロボットを四角で囲む
	line(tmp, cv::Point(0, 0), cv::Point(LEFT_U, UP_V), CV_RGB(0, 0, 255), 2, 4);						//以下で区切り線を引く
	line(tmp, cv::Point(Imgwidth, 0), cv::Point(RIGHT_U, UP_V), CV_RGB(0, 0, 255), 2, 4);
	line(tmp, cv::Point(0, Imgheight), cv::Point(LEFT_U, DOWN_V), CV_RGB(0, 0, 255), 2, 4);
	line(tmp, cv::Point(Imgwidth, Imgheight), cv::Point(RIGHT_U, DOWN_V), CV_RGB(0, 0, 255), 2, 4);
}
//透視変換用関数
cv::Mat warp(int ID, cv::Mat warpImg)
{
	int out_width=0,out_height=0;
	cv::Mat map_matrix;																								//変換行列保存用Mat
	cv::Mat gsrcImg;																						//src画像用Mat
	cv::Point2f src_pnt[4], dst_pnt[4];																				//変換前および変換後の頂点保存用変数
	gsrcImg=warpImg.clone();																					//GPUに元画像データを送信
	switch (ID)
	{
	case 0:																													//前面
	{
		out_width=window_size[0][0];
		out_height=window_size[0][1];
		dst_pnt[0] = cv::Point2f(0.0, 0.0);																				//出力左上座標
		dst_pnt[1] = cv::Point2f(out_width, 0.0);																	//出力右上座標
		dst_pnt[2] = cv::Point2f(out_width, out_height);														//出力右下座標
		dst_pnt[3] = cv::Point2f(0.0, out_height);																	//出力左下座標
		src_pnt[0] = cv::Point2f(0,0);
		src_pnt[1] = cv::Point2f(Imgwidth,0);
		src_pnt[2] = cv::Point2f(RIGHT_U,UP_V);
		src_pnt[3] = cv::Point2f(LEFT_U,UP_V);
	}
		break;
	case 1:																													//左面
	{
		out_width=window_size[1][0];
		out_height=window_size[1][1];
		dst_pnt[0] = cv::Point2f(0.0, 0.0);																				//出力左上座標
		dst_pnt[1] = cv::Point2f(out_width, 0.0);																	//出力右上座標				
		dst_pnt[2] = cv::Point2f(out_width, out_height);														//出力右下座標
		dst_pnt[3] = cv::Point2f(0.0, out_height);																	//出力左下座標
		src_pnt[0] = cv::Point2f(0,Imgheight);
		src_pnt[1] = cv::Point2f(0,0);
		src_pnt[2] = cv::Point2f(LEFT_U,UP_V);
		src_pnt[3] = cv::Point2f(LEFT_U,DOWN_V);
	}
		break;
	case 2:																													//右面
	{
		out_width=window_size[2][0];
		out_height=window_size[2][1];
		dst_pnt[0] = cv::Point2f(0.0, 0.0);																				//出力左上座標
		dst_pnt[1] = cv::Point2f(out_width, 0.0);																	//出力右上座標
		dst_pnt[2] = cv::Point2f(out_width, out_height);														//出力右下座標
		dst_pnt[3] = cv::Point2f(0.0, out_height);																	//出力左下座標
		src_pnt[0] = cv::Point2f(Imgwidth,0);
		src_pnt[1] = cv::Point2f(Imgwidth,Imgheight);
		src_pnt[2] = cv::Point2f(RIGHT_U,DOWN_V);
		src_pnt[3] = cv::Point2f(RIGHT_U,UP_V);
	}
		break;
	case 3:																													//後面
	{
		out_width=window_size[3][0];
		out_height=window_size[3][1];
		dst_pnt[0] = cv::Point2f(0.0, 0.0);																				//出力左上座標
		dst_pnt[1] = cv::Point2f(out_width, 0.0);																	//出力右上座標
		dst_pnt[2] = cv::Point2f(out_width, out_height);														//出力右下座標
		dst_pnt[3] = cv::Point2f(0.0, out_height);																	//出力左下座標
		src_pnt[0] = cv::Point2f(0,Imgheight);
		src_pnt[1] = cv::Point2f(Imgwidth,Imgheight);
		src_pnt[2] = cv::Point2f(RIGHT_U,DOWN_V);
		src_pnt[3] = cv::Point2f(LEFT_U,DOWN_V);
	}
	}
	if(ID>=0&&ID<=3){
		warpImg= cv::Mat(cv::Size(out_width, out_height), CV_8UC3);																//gminiImgの設定
		map_matrix = getPerspectiveTransform(src_pnt, dst_pnt);																//変換行列を求める
		cv::warpPerspective(gsrcImg, warpImg, map_matrix,warpImg.size());							//透視変換の実行
	}
	return warpImg;
}

//画像に固定直線を引く関数
void cv_fixed_line(cv::Mat &Img)
{
	cv::line(Img, left_line_points[0] , cv::Point(LEFT_U-line_distance, 0), CV_RGB(0, 255, 0), 2, 4);				//左
	cv::line(Img, right_line_points[0] , cv::Point(RIGHT_U+line_distance, 0), CV_RGB(0, 255, 0), 2, 4);				//右
}

//操作モードの変更を行う関数
void cv_mode_change(int& mode)
{
	if(joyinput.Buttons[4]&&Buttons_data[4]==0&&(mode==0||mode==1)){
		++mode;																							//X.Yモード→X.Rzモード→X.Y.Rzモード→X.Yモード
		++Buttons_data[4];
	}
	else if(joyinput.Buttons[4]&&Buttons_data[4]==0){
		mode=0;
		++Buttons_data[4];
	}
	else if(joyinput.Buttons[4]==0)
		Buttons_data[4]=0;
}
//直進時の線を引く関数
void cv_move_line(cv::Mat &Img)
{
	cv::line(Img, left_line_points[0], left_line_points[1] , CV_RGB(255, 0, 0), 2, 4);		//左
	cv::line(Img, right_line_points[0] , right_line_points[1] , CV_RGB(255, 0, 0), 2, 4);		//右
}
//カーブ時の線を引く関数
void cv_move_elipse(double k, cv::Mat &Img,int mode)
{
	 if (abs(joyinput.Y *1000)>= 10 && joyinput.X*1000 >= 50)
	{
		cv::Point2d left_point,right_point, last_left=left_line_points[0] ,last_right=right_line_points[0];
		for (double t = 0; t <= 1; t += 0.005)																//ベジェ曲線描画ここから
		{
			left_point=Bezier_curve(Bezier_curve(left_line_points[0],left_line_points[1],t),Bezier_curve(left_line_points[1],left_line_points[2],t),t);
			cv::line(Img, last_left , left_point , cv::Scalar(255, 0, 0),2);
			last_left=left_point;
			right_point=Bezier_curve(Bezier_curve(right_line_points[0],right_line_points[1],t),Bezier_curve(right_line_points[1],right_line_points[2],t),t);
			cv::line(Img, last_right, right_point , cv::Scalar(255, 0, 0),2);
			last_right=right_point;
		}
	}
	else if (joyinput.Y <0 && joyinput.X == 0)	//右矢印描画用
	{
		cv::line(Img, cv::Point(Img.cols / 2 - 10 - (joyinput.Y*0.001 * 20*-1000), UP_V / 2), cv::Point(Img.cols / 2 + 10 + (joyinput.Y*0.001 * 20*-1000), UP_V / 2), cv::Scalar(255, 0, 0),2);
		cv::line(Img, cv::Point(Img.cols / 2 + 10 + (joyinput.Y*0.001 * 20*-1000), UP_V / 2), cv::Point(Img.cols / 2 + 5 + (joyinput.Y*0.001 * 20*-1000), UP_V / 2-10), cv::Scalar(255, 0, 0),2);
		cv::line(Img, cv::Point(Img.cols / 2 + 10 + (joyinput.Y*0.001 * 20*-1000), UP_V / 2), cv::Point(Img.cols / 2 + 5 + (joyinput.Y*0.001 * 20*-1000), UP_V / 2+10), cv::Scalar(255, 0, 0),2);
	}
	else if (joyinput.Y >0 && joyinput.X == 0)	//左矢印描画用
	{
		cv::line(Img, cv::Point(Img.cols / 2 - 50 + (joyinput.Y*0.001 * 10*-1000), UP_V / 2), cv::Point(Img.cols / 2 + 50 - (joyinput.Y*0.001 * 10*-1000), UP_V / 2), cv::Scalar(255, 0, 0),2);
		cv::line(Img, cv::Point(Img.cols / 2 - 50 + (joyinput.Y*0.001 * 10*-1000), UP_V / 2), cv::Point(Img.cols / 2 - 25 + (joyinput.Y*0.001 * 10*-1000), UP_V / 2 - 10), cv::Scalar(255, 0, 0),2);
		cv::line(Img, cv::Point(Img.cols / 2 - 50 + (joyinput.Y*0.001 * 10*-1000), UP_V / 2), cv::Point(Img.cols / 2 - 25 + (joyinput.Y*0.001 * 10*-1000), UP_V / 2 + 10), cv::Scalar(255, 0, 0),2);
	}
}
//カーブの曲がり具合を調整する関数
void gain(double& k,cv::Mat &Img)
{
	std::string str;																		//文字列
	if (joyinput.Buttons[3] == 1) 
		k -= 0.1;											//ゲインの増減
	else if (joyinput.Buttons[5] == 1) 
		k += 0.1;
	if(k<=0)
		k=0;
	str = cv::format("Curvature correction=%.1lf", k);									//ゲインを文字列に直す
	putText(Img, str, cv::Point(window_size[0][0]-520, 150), face[3], 1.0, CV_RGB(0, 255, 0), 4, CV_AA);	//描画
}

void front_scale_change(double& front_scale,cv::Mat& Img){
	std::string str;																		//文字列
	if (joyinput.Buttons[2] == 1) 
		front_scale -= 0.1;											//ゲインの増減
	else if (joyinput.Buttons[4] == 1) 
		front_scale += 0.1;
	if(front_scale<=0)
		front_scale=0;
	str = cv::format("Front sensitivity=%.1lf", front_scale);									//ゲインを文字列に直す
	putText(Img, str, cv::Point(window_size[0][0]-470, 175), face[3], 1.0, CV_RGB(255, 200, 200), 4, CV_AA);	//描画
}

//モードの描画を行う関数
void cv_mode(cv::Mat &Img)
{
	static std::string str[3] = { "mode_X,Y", "mode_X,Rz", "mode_X,Y,Rz" };						//文字列定義
	putText(Img, str[mode], cv::Point(window_size[0][0]-320, 50), face[3], 1.0, CV_RGB(0, 255, 0), 4, CV_AA);	//描画
}

void cv_turn_mode(cv::Mat &Img){
	static std::string str[3] = { "back","right" , "left" };						//文字列定義
	putText(Img, str[line_mode-1], cv::Point(window_size[0][0]-320, 50), face[3], 1.0, CV_RGB(0, 255, 255), 4, CV_AA);	//描画
}

//ターゲットの状態を画面に表示する関数
void cv_draw_target(cv::Mat &Img)
{
	if (joyinput.Buttons[5])
		putText(Img, "Safe", cv::Point(Img.cols/2,Img.rows/2), face[3], 1.0, CV_RGB(0, 255, 0), 4, CV_AA);	//描画
	else if (joyinput.Buttons[3])
		putText(Img, "Broken", cv::Point(Img.cols / 2, Img.rows / 2), face[3], 1.0, CV_RGB(255, 0, 0), 4, CV_AA);	//描画
	if(joyinput.Buttons[0]){
		putText(Img, "Start/Stop", cv::Point(Img.cols / 2, Img.rows / 2), face[3], 1.0, CV_RGB(255, 0, 0), 4, CV_AA);	//描画
	}
}
//データを保存する関数
void cv_save_img(int ID, cv::Mat Img)
{
	switch (ID)
	{
	case 0:
		imwrite("/home/mouse/data/frontImg.png", Img);
		break;
	case 1:
		imwrite("/home/mouse/data/leftImg.png", Img);
		break;
	case 2:
		imwrite("/home/mouse/data/rightImg.png", Img);
		break;
	case 3:
		imwrite("/home/mouse/data/backImg.png", Img);
	}
}

void cv_turn_mat(cv::Mat &Img, double degree){
	cv::Point2f center = cv::Point2f(static_cast<float>(Img.cols / 2),
    static_cast<float>(Img.rows / 2));
 
     // アフィン変換行列
    cv::Mat affine;
    cv::getRotationMatrix2D(center, degree, turn_scale).copyTo(affine);
 
    cv::warpAffine(Img, Img, affine, Img.size(), cv::INTER_CUBIC);
}

void turn_change(void){
	line_mode=turn_direction_mode();
		switch(line_mode){
			case 1:	cv_turn_mat(copyImg, 180);
					break;
			case 2: cv_turn_mat(copyImg, 90);
					break;
			case 3: cv_turn_mat(copyImg, -90);
		}
}

//OpenGLここから
//display初期化ここから
void GLUT_INIT1(int &ID, const char *name)							//前面
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);		//描画方法の設定
	glutInitWindowPosition(windowPos[0].x, windowPos[0].y);			//window位置の設定
	glutInitWindowSize(FRONT_WINDOW_WIDTH, FRONT_WINDOW_HEIGHT);	//windowサイズの設定
	ID = glutCreateWindow(name);									//windowに名前と番号を振り分け
	glEnable(GL_LIGHTING);											//光源設定をonにする
	glEnable(GL_LIGHT0);											//一つ目の光源
	glEnable(GL_DEPTH_TEST);										//陰面消去の設定
	glEnable(GL_BLEND);												//半透明設定をonにする
	glEnable(GL_NORMALIZE);											//法線ベクトルを自動的に正規化（頂点の光源に対する方向を決定して真っ黒なポリゴンになるのを防ぐ）
}
void GLUT_INIT2(int &ID, const char *name)							//左面
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);		//描画方法の設定
	glutInitWindowPosition(windowPos[1].x, windowPos[1].y);			//window位置の設定
	glutInitWindowSize(LEFT_WINDOW_WIDTH, LEFT_WINDOW_HEIGHT);		//windowサイズの設定
	ID = glutCreateWindow(name);									//windowに名前と番号を振り分け
}
void GLUT_INIT3(int &ID, const char *name)							//右面
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);		//描画方法の設定
	glutInitWindowPosition(windowPos[2].x, windowPos[2].y);			//window位置の設定
	glutInitWindowSize(RIGHT_WINDOW_WIDTH, RIGHT_WINDOW_HEIGHT);	//windowサイズの設定
	ID = glutCreateWindow(name);									//windowに名前と番号を振り分け
}
void GLUT_INIT4(int &ID, const char *name)							//後面
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);		//描画方法の設定
	glutInitWindowPosition(windowPos[3].x, windowPos[3].y);			//window位置の設定
	glutInitWindowSize(BACK_WINDOW_WIDTH, BACK_WINDOW_HEIGHT);		//windowサイズの設定
	ID = glutCreateWindow(name);									//windowに名前と番号を振り分け
}
//displayコールバック関数設定ここまで
//ウィンドウ更新用関数
void idle(void)
{
	for (int i = 0; i < WINDOW_NUM; ++i) {
		glutSetWindow(WinID[i]);	//更新するwindowのセット
		glutPostRedisplay();		//更新の実行
	}
}

//joystick入力判定用関数、ここから
bool joystick_buttons_save(void)
{
	if (joyinput.Buttons[11])
		return true;
	else
		return false;
}
//描画用関数ここまで
//displayコールバック関数設定ここから
void GLUT_CALL_FUNC1(void)
{
	glutDisplayFunc(display1);	//windowにdisplayを割り当て
	glutReshapeFunc(resize1);
}
void GLUT_CALL_FUNC2(void)
{
	glutDisplayFunc(display2);	//windowにdisplayを割り当て
	glutReshapeFunc(resize2);
}
void GLUT_CALL_FUNC3(void)
{
	glutDisplayFunc(display3);	//windowにdisplayを割り当て
	glutReshapeFunc(resize3);
}
void GLUT_CALL_FUNC4(void)
{
	glutDisplayFunc(display4);	//windowにdisplayを割り当て
	glutReshapeFunc(resize4);
}
void GLUT_CALL_FUNC5(void)
{
	glutDisplayFunc(display5);	//windowにdisplayを割り当て
	glutReshapeFunc(resize5);
}

//display初期化ここまで
//描画用関数ここから
void display1(void)
{	
	if(line_mode==0||line_mode==1){
		cv_fixed_line(copyImg);							//固定の線を描画
		if (joyinput.X!=0||joyinput.Y!=0)
		{
			if (joyinput.Y == 0){
				if(joyinput.X>0)
					cv_move_line(copyImg);				//直線
			}
			else
				cv_move_elipse( gain_k, copyImg, mode);	//曲線
		}
	}																								//前画面
	cv::Mat frontImg = warp(0, copyImg);																//透視変換した画像の用意
																									//OpenGLでの描画設定、ここから
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);												//画面の色とデプスのバッファをクリア
	glViewport(0, 0, FRONT_WINDOW_WIDTH, FRONT_WINDOW_HEIGHT);										//viewportの設定
	glMatrixMode(GL_PROJECTION);																	//投影変換モード
	glLoadIdentity();																				//投影変換の変換行列を単位行列で初期化
	gluPerspective(30.0, (double)FRONT_WINDOW_WIDTH / (double)FRONT_WINDOW_HEIGHT, 1.0, 1000.0);	//視界の決定
	glMatrixMode(GL_MODELVIEW);																		//モデルビュー変換行列の設定
	glLoadIdentity();																				//モデルビュー変換行列を単位行列で初期化
	gluLookAt(0.0, 0.0, 65.0, //カメラの座標
		8.0, 180.0, 0, // 注視点の座標
		0.0, 0.0, 1.0); // 画面の上方向を指すベクトル
	glLightfv(GL_LIGHT0, GL_POSITION, lightpos);													//ライトを当てる
	CV_FRONT_FUNC(frontImg);
	CV_CALL_FUNC(0, frontImg);																		//opencvの各処理を行う関数の呼び出し
	glDrawPixels(frontImg.cols, frontImg.rows, GL_RGB, GL_UNSIGNED_BYTE, frontImg.data);			//描画処理
	glClear(GL_DEPTH_BUFFER_BIT);																	//デプスバッファのクリア
	glutSwapBuffers();																				//画面を更新
																									//OpenGLでの描画設定、ここから
}
void display2(void)
{																									//左画面
	cv::Mat leftImg = warp(1, copyImg);																	//透視変換した画像の用意
																									//OpenGLでの描画設定、ここから
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);												//画面の色とデプスのバッファをクリア
	glLoadIdentity();																				//単位行列の初期化
	CV_CALL_FUNC(1, leftImg);																		//opencvの各処理を行う関数の呼び出し
	glDrawPixels(leftImg.cols, leftImg.rows, GL_RGB, GL_UNSIGNED_BYTE, leftImg.data);				//描画処理
	glutSwapBuffers();																				//画面を更新//OpenGLでの描画設定、ここまで
}
void display3(void)
{																									//右画面
	cv::Mat rightImg = warp(2, copyImg);																//透視変換した画像の用意
																									//OpenGLでの描画設定、ここから
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);												//画面の色とデプスのバッファをクリア
	glLoadIdentity();																				//単位行列の初期化
	CV_CALL_FUNC(2, rightImg);																	//opencvの各処理を行う関数の呼び出し
	glDrawPixels(rightImg.cols, rightImg.rows, GL_RGB, GL_UNSIGNED_BYTE, rightImg.data);			//描画処理
	glutSwapBuffers();																			//画面を更新  OpenGLでの描画設定、ここまで
}
void display4(void)
{																								//後画面
	cv::Mat backImg = warp(3, copyImg);																//透視変換した画像の用意
																								//OpenGLでの描画設定、ここから
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);												//画面の色とデプスのバッファをクリア
	glLoadIdentity();																				//単位行列の初期化
	CV_CALL_FUNC(3, backImg);																		//opencvの各処理を行う関数の呼び出し
	glDrawPixels(backImg.cols, backImg.rows, GL_RGB, GL_UNSIGNED_BYTE, backImg.data);				//描画処理
	glutSwapBuffers();																				//画面を更新  OpenGLでの描画設定、ここまで
}

void GLUT_INIT5(int &ID, const char *name)							
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);		//描画方法の設定
	glutInitWindowPosition(windowPos[4].x, windowPos[4].y);			//window位置の設定
	glutInitWindowSize(Imgwidth*window5_ratio, Imgheight*window5_ratio);		//windowサイズの設定
	ID = glutCreateWindow(name);									//windowに名前と番号を振り分け
}

void display5(void)
{	
		flip(copyImg, copyImg, 0);								//Opencvの画像をOpenGLに合うように座標軸を変更
		cvtColor(copyImg, copyImg, cv::COLOR_BGR2RGB);				//Opencvの画像をOpenGLに合うように画像色を変更																
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);												//画面の色とデプスのバッファをクリア
		glLoadIdentity();																				//単位行列の初期化
		std::cout<<window5_ratio<<std::endl;
		if(window5_ratio!=1)
			cv::resize(copyImg,copyImg,cv::Size(),window5_ratio,window5_ratio);
		glDrawPixels(copyImg.cols, copyImg.rows, GL_RGB, GL_UNSIGNED_BYTE, copyImg.data);				//描画処理
		glutSwapBuffers();																				//画面を更新
}
void resize1(int w,int h){
	printf("resize has been called. w=%d\th=%d\n", w, h);
	window_size[0][0]=w-(w%4);
	window_size[0][1]=h;
}
void resize2(int w,int h){
	printf("resize has been called. w=%d\th=%d\n", w, h);
	window_size[1][0]=w-(w%4);
	window_size[1][1]=h;
}
void resize3(int w,int h){
	printf("resize has been called. w=%d\th=%d\n", w, h);
	window_size[2][0]=w-(w%4);
	window_size[2][1]=h;
}
void resize4(int w,int h){
	printf("resize has been called. w=%d\th=%d\n", w, h);
	window_size[3][0]=w-(w%4);
	window_size[3][1]=h;
}
void resize5(int w,int h){
	printf("resize has been called. w=%d\th=%d\n", w, h);
	int width_tmp;
		window5_ratio=(double)w/Imgwidth;
	if(window5_ratio>(double)h/Imgheight)
		window5_ratio=(double)h/Imgheight;
	width_tmp=Imgwidth*window5_ratio;
	if(width_tmp%4){
		width_tmp-=width_tmp%4;
		window5_ratio=(double)width_tmp/Imgwidth;
	}
}

//データ保存用関数、使ってる
void data_save(int64 s, int64 e)
{
	double msec = (e - s) * 1000 / cv::getTickFrequency();	//プログラムの時間
	std::cout << msec << std::endl;
	fout << joyinput.X<<",";									// X軸データ
	fout << joyinput.Y<<",";									// Y軸データ
	fout << mode<<",";										// モードデータ
	fout << msec<<",";										// プログラム作動時間
	if (joyinput.Buttons[5] == 1){						//探索状態の表示
		fout << "Safe"<<",";
	}
	else if (joyinput.Buttons[3] == 1){
		fout << "Broken"<<",";
	}
	if(joyinput.Buttons[0])
		fout<<"Start/Stop"<<",";
	else fout << "Searching"<<",";
	auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    fout << " - " <<std::ctime(&now_time);
}