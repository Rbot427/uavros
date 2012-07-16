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
	capture.open(camera_number);

	/* Start always with lowest resolution */
	capture.set(CV_CAP_PROP_FRAME_WIDTH, resolution_.width);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, resolution_.height);

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
		cvi.header.frame_id = camera_id;
		cvi.encoding = "bgr8";
		cvi.image = undistorted;

		/* Publish the message of the image */
		sensor_msgs::Image image_msg;
		cvi.toImageMsg(image_msg);
		image_pub_.publish(image_msg);

		/* Create the CameraInfo message */
		sensor_msgs::CameraInfo info_msg;
		info_msg.header.stamp = ros::Time::now();
		info_msg.header.frame_id = camera_id;
		info_msg.width = resolution_.width;
		info_msg.height = resolution_.height;
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
	if(!n_param.getParam("width", resolution_.width)) {
		resolution_.width = 640;
	}
	if(!n_param.getParam("height", resolution_.height)) {
		resolution_.height = 480;
	}
}

/* Create the publishers to advertise in the corresponding topics */
void video_publisher::create_publishers()
{
	/* Get the string of the msg topics with the camera_num parameter */
	char cam_info[40], cam_image[40], cam_id[30];
	int n = sprintf(cam_image, "usb_cam%d/image_raw", camera_number);
	n = sprintf(cam_info, "usb_cam%d/camera_info", camera_number);
	n = sprintf(cam_id, "usb_cam%d", camera_number);
	camera_id = cam_id;

	image_pub_ = it_.advertise(cam_image, 1);
	info_pub_ = n_.advertise<sensor_msgs::CameraInfo>(cam_info, 1);
}

/* Create the subscribers to the corresponding topcis */
void video_publisher::create_subscribers()
{
}

/* Set camera calibration values and ar_pose values */
void video_publisher::set_camera_calibration()
{
	if(resolution_.width == 640) {
   		camera_matrix_ = (cv::Mat_<double>(3,3) << 1408.8319, 0.0000, 294.6681,
        		                                   0.0000, 1389.8016, 237.9596,
                		                           0.0000, 0.0000, 1.0000);
	    distorted_coefficients_ = (cv::Mat_<double>(1,5) << 
								-3.3458, 10.73656, 0.0775, 0.0037, 0.0);
	} else {
		camera_matrix_ = (cv::Mat_<double>(3,3) << 210.9809, 0.0000, 164.0612,
												   0.000, 208.9717, 128.6563,
												   0.0000, 0.0000, 1.0000);
		distorted_coefficients_ = (cv::Mat_<double>(1,5) <<
								-0.3686, 0.1074, -0.0092, 0.0008, 0.0);
	}	

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

