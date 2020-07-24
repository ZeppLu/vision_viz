#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/VisionInfo.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>


// global variables
sensor_msgs::Image image;
sensor_msgs::Image rendered;
vision_msgs::Detection2DArray detections;


void image_callback(const sensor_msgs::ImageConstPtr& p_image) {
	image = *p_image;
	ROS_DEBUG("image received (%u)", image.header.stamp.sec);
}

void detections_callback(const vision_msgs::Detection2DArrayConstPtr& p_detections) {
	detections = *p_detections;
	ROS_DEBUG("detections received (%u)", detections.header.stamp.sec);
}


void draw_bboxes(const ros::Duration& max_delay) {
	// timestamp of last used image / detections
	static ros::Time last_image_stamp, last_detections_stamp;

	// check whether need to re-draw
	if (image.header.stamp == last_image_stamp && detections.header.stamp == last_detections_stamp) {
		ROS_INFO("no new data, using last rendered image");
		return;
	}

	ROS_INFO("going to render a new image");

	// initialize output image
	//rendered.height			= image.height;
	//rendered.width			= image.width;
	//rendered.encoding		= image.encoding;
	//rendered.is_bigendian	= image.is_bigendian;
	//rendered.step			= image.step;
	//rendered.data			= image.data;
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(image, image.encoding);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	last_image_stamp = image.header.stamp;

	// check delay between detections result & source image
	ros::Duration delay = detections.header.stamp - image.header.stamp;
	if (delay <= ros::Duration(0.0) || delay > max_delay) {
		ROS_INFO("no up-to-date detection for this frame (delay: %u.%u)", delay.sec, delay.nsec);
		return;
	}

	// draw bounding boxes
	ROS_INFO("going to draw bounding boxes");
	//
	cv_ptr->toImageMsg(rendered);
	last_detections_stamp = detections.header.stamp;
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "detections_viz");

	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	image_transport::ImageTransport it(private_nh);

	// XXX: without setting this parameter, compressed image would not be subscribed!
	private_nh.setParam("image_transport", "compressed");

	// max delay between timestamps of image & detections,
	// if delay exceeds this value, detections are consider out-dated
	int max_delay_ms;
	if (!private_nh.getParam("max_delay_ms", max_delay_ms)) {
		ROS_ERROR("parameter `~max_delay_ms' not set! exitting...");
		return 0;
	}
	ros::Duration max_delay(max_delay_ms / 1000.0);
	ROS_INFO("max delay set to %us + %uns", max_delay.sec, max_delay.nsec);

	image_transport::Publisher image_pub = it.advertise("image_out", 1);

	image_transport::Subscriber image_sub = it.subscribe("image_in", 1, &image_callback);
	ros::Subscriber detections_sub = private_nh.subscribe<vision_msgs::Detection2DArray>("detections", 1, &detections_callback);

	// publish rate
	double hz = 30.0;
	private_nh.param("hz", hz, hz);
	ros::Rate rate(hz);

	while (ros::ok()) {
		rate.sleep();

		draw_bboxes(max_delay);

		rendered.header.stamp = ros::Time::now();
		image_pub.publish(rendered);

		ros::spinOnce();
	}

	return 0;
}
