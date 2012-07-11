#include <uav_camera/video_publisher.h>

/* Create a node handle and a video_publisher */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "video_publisher");
	ros::NodeHandle n;
	video_publisher vid_pub(n);
	ros::spin();
	return 0;
}

/* Constructor */
video_publisher::video_publisher(ros::NodeHandle &n): n_ (n), it_ (n_)
{
	get_params();
	create_publishers();
	create_subscribers();
	set_camera_calibration();
	resolution_.current_width = resolution_.width_low;
	resolution_.current_height = resolution_.height_low;
	capture.open(camera_number);

	/* Start always with lowest resolution */
	capture.set(CV_CAP_PROP_FRAME_WIDTH, resolution_.width_low);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, resolution_.height_low);
	resolution_.current_width = resolution_.width_low;
	resolution_.current_height = resolution_.height_low;

	main_loop_ = new boost::thread(boost::bind(&video_publisher::main_loop, this));
}

/* Destructor */
video_publisher::~video_publisher()
{

}

void video_publisher::main_loop() 
{
	cv::Mat frame;

	ros::Rate loop_rate(50);
	while(n_.ok()) {
		/* Grab a new frame */
		capture >> frame;

		/* Get the undistort image with the camera parameters */
		cv::Mat undistorted;
		undistort(frame, undistorted, camera_matrix_, 
						distorted_coefficients_, camera_matrix_);
		/* Create message to send the image */
		cv_bridge::CvImage cvi;
		cvi.header.stamp = ros::Time::now();
		cvi.header.frame_id = "usb_cam";
		cvi.encoding = "bgr8";
		cvi.image = undistorted;

		/* Publish the message of the image */
		sensor_msgs::Image image_msg;
		cvi.toImageMsg(image_msg);
		image_pub_.publish(image_msg);

		/* Create the CameraInfo message */
		sensor_msgs::CameraInfo info_msg;
		info_msg.header.stamp = ros::Time::now();
		info_msg.header.frame_id = "usb_cam";
		info_msg.width = resolution_.current_width;
		info_msg.height = resolution_.current_height;
		info_msg.distortion_model = "";
		info_msg.D = cv::vector<double>(ar_params_.d, ar_params_.d + 
						sizeof(ar_params_.d)/sizeof(*ar_params_.d));
		info_msg.K = ar_params_.k;
		info_msg.R = ar_params_.r;
		info_msg.P = ar_params_.p;

		/* Publish the message of the CameraInfo */
		info_pub_.publish(info_msg);

		loop_rate.sleep();
	}
}


/* Get the parameters, if none given, assume defaults */
void video_publisher::get_params()
{
	ros::NodeHandle n_param("~");

	if(!n_param.getParam("camera_number", camera_number)) {
		camera_number = 0;
	}
	if(!n_param.getParam("width_high", resolution_.width_high)) {
		resolution_.width_high = 640;
	}
	if(!n_param.getParam("height_high", resolution_.height_high)) {
		resolution_.height_high = 480;
	}
	if(!n_param.getParam("width_low", resolution_.width_low)) {
		resolution_.width_low = 320;
	}
	if(!n_param.getParam("height_low", resolution_.height_low)) {
		resolution_.height_low = 240;
	}
}

/* Create the publishers to advertise in the corresponding topics */
void video_publisher::create_publishers()
{
	/* Get the string of the msg topics with the camera_num parameter */
	char cam_info[40], cam_image[40];
	int n = sprintf(cam_image, "usb_cam%d/image_raw", camera_number);
	n = sprintf(cam_info, "usb_cam%d/camera_info", camera_number);

	image_pub_ = it_.advertise(cam_image, 1);
	info_pub_ = n_.advertise<sensor_msgs::CameraInfo>(cam_info, 1);
	res_ready_pub_ = n_.advertise<uav_msgs::camera_msg>("inc_res", 1);
}

/* Create the subscribers to the corresponding topcis */
void video_publisher::create_subscribers()
{
	dec_res_sub_ = n_.subscribe("dec_res", 1,
		&video_publisher::decrease_resolution_callback, this);	
	inc_res_sub_ = n_.subscribe("inc_res", 1,
		&video_publisher::increase_resolution_callback, this);
}

/* Set camera calibration values and ar_pose values */
void video_publisher::set_camera_calibration()
{
    /* Parameters for camera with fish-eye lens */
    camera_matrix_ = (cv::Mat_<double>(3,3) << 1408.831918, 0.000000, 294.668141,
                                           0.000000, 1389.801615, 237.959610,
                                           0.000000, 0.000000, 1.000000);
    distorted_coefficients_ = (cv::Mat_<double>(1,5) << 
								-3.3458, 10.736563, 0.077535, 0.003666, 0.0);

    /* Parameters so ARToolkit think we have a normal	*
	 * camera, and see a rectify image 					*/
    double temp_d[] = {-0.363638, 0.095521, 0.002198, 0.002716, 0.000000};
	*ar_params_.d = *temp_d;
    ar_params_.k = { {450.724360, 0.000000, 317.891495,
                      0.000000, 444.956819, 233.815601,
                      0.0, 0.0, 1.0} };
    ar_params_.r = { {1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0} };
    ar_params_.p = { {450.72436, 0.000000, 317.891495, 0.0,
                      0.000000, 444.956819, 233.815601, 0.0,
                      0.0, 0.0, 1.0, 0.0} };
}

/* Decrease resolution of camera */
void video_publisher::decrease_resolution_callback(uav_msgs::camera_msg msg)
{
    /* Check wether this node has the camera to decrease the resolution */
    if(msg.change_res == camera_number) {

		/* Decrease resolution*/
		capture.set(CV_CAP_PROP_FRAME_WIDTH, resolution_.width_low);
		capture.set(CV_CAP_PROP_FRAME_HEIGHT, resolution_.height_low);

		resolution_.current_width = resolution_.width_low;
		resolution_.current_height = resolution_.height_low;

		ROS_INFO("Resolution decreased");

        /* Publish the msg so that the other camera can increse	*
		 * the resolution without causing problems 				*/
        uav_msgs::camera_msg change_msg;
        change_msg.change_res = !camera_number;
        res_ready_pub_.publish(change_msg);
    }
}

/* Increase resolution of camera */
void video_publisher::increase_resolution_callback(uav_msgs::camera_msg msg)
{
    /* Check wether this node has the camera to increase the resolution */
    if(msg.change_res == camera_number) {

		/* Increase resolution*/
		capture.set(CV_CAP_PROP_FRAME_WIDTH, resolution_.width_low);
		capture.set(CV_CAP_PROP_FRAME_HEIGHT, resolution_.height_low);

		resolution_.current_width = resolution_.width_low;
		resolution_.current_height = resolution_.height_low;
	}
}
 
