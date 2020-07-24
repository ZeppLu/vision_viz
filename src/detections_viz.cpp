#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/VisionInfo.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


void callback(const sensor_msgs::ImageConstPtr& image_in, const vision_msgs::Detection2DArrayConstPtr& detections, image_transport::Publisher& image_pub) {
	ROS_INFO("received a pair");
	sensor_msgs::Image msg;
	image_pub.publish(msg);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "detections_viz");

	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	image_transport::ImageTransport it(private_nh);

	// parameters used to help messages matching faster
	// see http://wiki.ros.org/message_filters/ApproximateTime for details
	int detect_delay_lb_ms;
	double age_penalty;
	// TODO: raise exception
	if (!private_nh.getParam("detect_delay_lb_ms", detect_delay_lb_ms)) {
		ROS_ERROR("parameter `detect_delay_lb_ms' not set! exitting...");
		return 0;
	}
	if (!private_nh.getParam("age_penalty", age_penalty)) {
		ROS_ERROR("parameter `age_penalty' not set! exitting...");
		return 0;
	}

	// subscribe to compressed image
	// XXX: this step is necessary, or subscribed topic is default to `raw',
	// which greatly consumes bandwidth
	private_nh.setParam("image_transport", "compressed");

	// rendered images publisher
	image_transport::Publisher image_pub = it.advertise("image_out", 1);

	// subscribers for camera captures & detections output from jetson nano
	// use message_filters to match messages from different topics by timestamps,
	// note that image_transport::SubscriberFilter() is used for sensor_msgs/Image type
	// TODO: what does last argument (queue_size) in these 3 lines do?
	// XXX: when remapping topics, be sure to remap /detections_viz/image_in/compressed
	image_transport::SubscriberFilter image_sub(it, "image_in", 1);
	message_filters::Subscriber<vision_msgs::Detection2DArray> detect_sub(private_nh, "detections", 1);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, vision_msgs::Detection2DArray> SyncPol;
	// queue_size = 10
	message_filters::Synchronizer<SyncPol> sync(SyncPol(10), image_sub, detect_sub);
	// 1ms == 1000000ns
	sync.setInterMessageLowerBound(ros::Duration(0, detect_delay_lb_ms * 1000000));
	sync.setAgePenalty(age_penalty);
	sync.registerCallback(boost::bind(&callback, _1, _2, image_pub));

	ros::spin();

	return 0;
}
