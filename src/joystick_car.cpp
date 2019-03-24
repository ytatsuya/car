#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include<std_msgs/Int32.h>

void shift_change(int& ,int ,int, int&);

class TeleopTurtle{
	public:
	TeleopTurtle();
	private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	ros::NodeHandle nh;
	int vel_linear,vel_angular,vel_brake,shift_up,shift_down;
	std_msgs::Int32 shift;
	int buttons_data[21];
	double l_scale_,a_scale_,b_scale_;
	ros::Publisher vel_pub_,shift_pub_;
	ros::Subscriber joy_sub_,odom_sub_;
};

TeleopTurtle::TeleopTurtle():vel_linear(1),vel_angular(0),vel_brake(2),l_scale_(0.2), a_scale_(-1.7),b_scale_(0.1), shift_up(12),shift_down(13)
{
	shift.data=0;
	ros::NodeHandle nh("~");
	joy_sub_=nh.subscribe<sensor_msgs::Joy>("/joy",10,&TeleopTurtle::joyCallback,this);
	//odom_sub_=nh.subscribe("/odom",1,&TeleopTurtle::odom,this);
	// vel_pub_=nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
	vel_pub_=nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	shift_pub_=nh.advertise<std_msgs::Int32>("/shift",1);
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist twist;
	
	shift_change(shift.data,joy->buttons[shift_up] ,joy->buttons[shift_down] , buttons_data[shift_up]);
	if(shift.data){
		if(joy->axes[vel_linear]==-1)
			twist.linear.x=l_scale_*0.1;
		else	
			twist.linear.x=l_scale_*(joy->axes[vel_linear]+1)/2;
		
		if(joy->axes[vel_brake]!=-1)
			twist.linear.x-=((joy->axes[vel_brake]+1)*b_scale_);
		if(twist.linear.x<0)
			twist.linear.x=0;

		twist.angular.z=a_scale_*joy->axes[vel_angular]*twist.linear.x;

		if(shift.data<0){
			twist.linear.x*=-1;
			twist.angular.z*=-1;
		}
	}

	ROS_INFO_STREAM("("<<joy->axes[vel_linear]<<""<<joy->axes[vel_angular]<<""<<joy->axes[vel_brake]<<")");
	ROS_INFO_STREAM("("<<"shift:"<<shift.data<<")");
	
	vel_pub_.publish(twist);
	shift_pub_.publish(shift);
}
int main(int argc,char** argv)
{
	ros::init(argc,argv,"teleop_car");
	TeleopTurtle teleop_car;
	ros::spin();

	return 0;
}

void shift_change(int& shift,int buttons_up ,int buttons_down , int& buttons_data)
{
	if(buttons_up==1&&buttons_data==0&&shift>=0){
		if(!shift)
			++shift;																							//shift==D
		++buttons_data;
	}
	else if(buttons_up==1&&buttons_data==0&&shift<0){
		shift=0;
		++buttons_data;
	}
	if(buttons_down==1&&buttons_data==0){
		if(shift>-1)
			--shift;																							//shift==R
		++buttons_data;
	}
	if(!buttons_up&&!buttons_down)
		buttons_data=0;
}

