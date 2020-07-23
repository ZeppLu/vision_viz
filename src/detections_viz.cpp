#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/VisionInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


void callback(const sensor_msgs::ImageConstPtr& image_in, const vision_msgs::Detection2DArrayConstPtr& detections, image_transport::Publisher& image_pub) {
	void(0);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "detections_viz");

	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	image_transport::ImageTransport it(private_nh);
	image_transport::Publisher image_pub = it.advertise("image_out", 1);

	// TODO: what's the last argument in these 3 lines?
	message_filters::Subscriber<sensor_msgs::Image> image_sub(private_nh, "image_in", 1);
	message_filters::Subscriber<vision_msgs::Detection2DArray> detect_sub(private_nh, "detections", 1);
	message_filters::TimeSynchronizer<sensor_msgs::Image, vision_msgs::Detection2DArray> sync(image_sub, detect_sub, 10);
	sync.registerCallback(boost::bind(&callback, _1, _2, image_pub));

	ros::spin();

	return 0;
}
