#include "avr_ros/Twist.h"
using namespace geometry_msgs;
ros::MsgSz Twist::serialize(uint8_t *in_data)
{
	ros::MsgSz offset = 0;
	offset += this->linear.serialize(in_data + offset);
	offset += this->angular.serialize(in_data + offset);
	return offset;
}
ros::MsgSz Twist::deserialize(uint8_t *out_data)
{
	ros::MsgSz offset = 0;
	offset += this->linear.deserialize(out_data + offset);
	offset += this->angular.deserialize(out_data + offset);
	return offset;
}
ros::MsgSz Twist::bytes()
{
	ros::MsgSz msgSize = 0;
	msgSize += linear.bytes();
	msgSize += angular.bytes();
	return msgSize;
}
