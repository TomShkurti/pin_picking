#include <cwru_davinci/uv_control/psm_interface.h>
#include <cwru_davinci_white_tracker/cwru_davinci_white_tracker.h>
int main(int argc, char** argv){

	int n = 6;

	ros::init(argc, argv, "pins");
	ros::NodeHandle nh;

	psm_interface psm = psm_interface(1, nh);

	//Identify pins.
	std::vector<cv::Point3d> pins = std::vector<cv::Point3d>();

	int cm = p_tracker::find_multiple_markers(pins, nh, n);

	/*for(int i = 0; i < n; i++){
		ROS_INFO("%f %f %f", pins[i].x, pins[i].y, pins[i].z);
	}*/

	cv::Mat handeye = (cv::Mat_<double>(4, 4) <<
   -0.6891,    0.7182,    0.0964,   -0.1030,
    0.6880,    0.6903,   -0.2241,    0.0659,
   -0.2275,   -0.0881,   -0.9698,    0.0335,
         0,         0,         0,    1.0000
	);

	std::vector<double> x_down = {0.99, 0.0, 0.01};
	std::vector<double> z_down = {0.0, 0.01, -0.99};

	for(int i = 0; i < n; i++){
		cv::Mat tmp = (cv::Mat_<double>(4, 1) << 
			pins[i].x,
			pins[i].y,
			pins[i].z,
			1.0
		);
		cv::Mat transformed = handeye * tmp;

		ROS_INFO("PIN POS %f %f %f",
			transformed.at<double>(0, 0),
			transformed.at<double>(0, 1),
			transformed.at<double>(0, 2)
		);
		
		std::vector<double> tf_vec = {
			transformed.at<double>(0, 0),
			transformed.at<double>(0, 1),
			transformed.at<double>(0, 2)
		};
		std::vector<double> tf_vec_up = {
			transformed.at<double>(0, 0),
			transformed.at<double>(0, 1),
			transformed.at<double>(0, 2) + 0.06
		};

		psm.go(tf_vec_up, x_down, z_down, 1.8, 2.0);
		psm.go(tf_vec   , x_down, z_down, 1.8, 2.0);
		psm.go(tf_vec   , x_down, z_down, -0.5, 2.0);
		psm.go(tf_vec_up, x_down, z_down, -0.5, 2.0);
		psm.go(tf_vec_up, x_down, z_down, 1.8, 2.0);
	}

	return 0;
}
