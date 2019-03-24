#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#define Imgwidth 1024								//キャプチャ画像の横サイズ
#define Imgheight 1024								//キャプチャ画像の縦サイズ
 
int main(int argc, char** argv)
{
	ros::init (argc, argv, "cv_bridge_omnidirectional_cam");
	ros::NodeHandle nh("~");
 
	image_transport::ImageTransport it(nh);
	image_transport::Publisher image_pub = it.advertise("image", 10);
 
	cv::Mat image;
	//キャプチャ設定/
    cv::VideoCapture cap("http://172.16.0.254:9176");	//キャプチャ先のURL
 
	if (!cap.isOpened()) {
		ROS_INFO("failed to open camera.");
		return -1;
	}
   	cap.set(CV_CAP_PROP_FRAME_WIDTH, Imgwidth);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, Imgheight);

 
	while(ros::ok()) {
        do {
		cap >> image;					//画像取得
	    } while (image.empty()&&ros::ok());			//キャプチャするまで待機
 
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
		image_pub.publish(msg);
 	}
 
	return 0;
}