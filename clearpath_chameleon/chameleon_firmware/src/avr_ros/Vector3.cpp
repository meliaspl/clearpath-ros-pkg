#include "avr_ros/Vector3.h"
using namespace geometry_msgs;
ros::MsgSz Vector3::serialize(uint8_t *in_data)
{
	ros::MsgSz offset = 0;
	union {
		ros::float64 real;
		uint64_t base;
	} u_x;
	u_x.real = this->x;
	*(in_data + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
	*(in_data + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
	*(in_data + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
	*(in_data + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
	*(in_data + offset + 4) = (u_x.base >> (8 * 4)) & 0xFF;
	*(in_data + offset + 5) = (u_x.base >> (8 * 5)) & 0xFF;
	*(in_data + offset + 6) = (u_x.base >> (8 * 6)) & 0xFF;
	*(in_data + offset + 7) = (u_x.base >> (8 * 7)) & 0xFF;
	offset += sizeof(this->x);
	union {
		ros::float64 real;
		uint64_t base;
	} u_y;
	u_y.real = this->y;
	*(in_data + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
	*(in_data + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
	*(in_data + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
	*(in_data + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
	*(in_data + offset + 4) = (u_y.base >> (8 * 4)) & 0xFF;
	*(in_data + offset + 5) = (u_y.base >> (8 * 5)) & 0xFF;
	*(in_data + offset + 6) = (u_y.base >> (8 * 6)) & 0xFF;
	*(in_data + offset + 7) = (u_y.base >> (8 * 7)) & 0xFF;
	offset += sizeof(this->y);
	union {
		ros::float64 real;
		uint64_t base;
	} u_z;
	u_z.real = this->z;
	*(in_data + offset + 0) = (u_z.base >> (8 * 0)) & 0xFF;
	*(in_data + offset + 1) = (u_z.base >> (8 * 1)) & 0xFF;
	*(in_data + offset + 2) = (u_z.base >> (8 * 2)) & 0xFF;
	*(in_data + offset + 3) = (u_z.base >> (8 * 3)) & 0xFF;
	*(in_data + offset + 4) = (u_z.base >> (8 * 4)) & 0xFF;
	*(in_data + offset + 5) = (u_z.base >> (8 * 5)) & 0xFF;
	*(in_data + offset + 6) = (u_z.base >> (8 * 6)) & 0xFF;
	*(in_data + offset + 7) = (u_z.base >> (8 * 7)) & 0xFF;
	offset += sizeof(this->z);
	return offset;
}
ros::MsgSz Vector3::deserialize(uint8_t *out_data)
{
	ros::MsgSz offset = 0;
	union {
		ros::float64 real;
		uint64_t base;
	} u_x;
	u_x.base = 0;
	u_x.base |= ((typeof(u_x.base)) (*(out_data + offset + 0))) << (8 * 0);
	u_x.base |= ((typeof(u_x.base)) (*(out_data + offset + 1))) << (8 * 1);
	u_x.base |= ((typeof(u_x.base)) (*(out_data + offset + 2))) << (8 * 2);
	u_x.base |= ((typeof(u_x.base)) (*(out_data + offset + 3))) << (8 * 3);
	u_x.base |= ((typeof(u_x.base)) (*(out_data + offset + 4))) << (8 * 4);
	u_x.base |= ((typeof(u_x.base)) (*(out_data + offset + 5))) << (8 * 5);
	u_x.base |= ((typeof(u_x.base)) (*(out_data + offset + 6))) << (8 * 6);
	u_x.base |= ((typeof(u_x.base)) (*(out_data + offset + 7))) << (8 * 7);
	this->x = u_x.real;
	offset += sizeof(this->x);
	union {
		ros::float64 real;
		uint64_t base;
	} u_y;
	u_y.base = 0;
	u_y.base |= ((typeof(u_y.base)) (*(out_data + offset + 0))) << (8 * 0);
	u_y.base |= ((typeof(u_y.base)) (*(out_data + offset + 1))) << (8 * 1);
	u_y.base |= ((typeof(u_y.base)) (*(out_data + offset + 2))) << (8 * 2);
	u_y.base |= ((typeof(u_y.base)) (*(out_data + offset + 3))) << (8 * 3);
	u_y.base |= ((typeof(u_y.base)) (*(out_data + offset + 4))) << (8 * 4);
	u_y.base |= ((typeof(u_y.base)) (*(out_data + offset + 5))) << (8 * 5);
	u_y.base |= ((typeof(u_y.base)) (*(out_data + offset + 6))) << (8 * 6);
	u_y.base |= ((typeof(u_y.base)) (*(out_data + offset + 7))) << (8 * 7);
	this->y = u_y.real;
	offset += sizeof(this->y);
	union {
		ros::float64 real;
		uint64_t base;
	} u_z;
	u_z.base = 0;
	u_z.base |= ((typeof(u_z.base)) (*(out_data + offset + 0))) << (8 * 0);
	u_z.base |= ((typeof(u_z.base)) (*(out_data + offset + 1))) << (8 * 1);
	u_z.base |= ((typeof(u_z.base)) (*(out_data + offset + 2))) << (8 * 2);
	u_z.base |= ((typeof(u_z.base)) (*(out_data + offset + 3))) << (8 * 3);
	u_z.base |= ((typeof(u_z.base)) (*(out_data + offset + 4))) << (8 * 4);
	u_z.base |= ((typeof(u_z.base)) (*(out_data + offset + 5))) << (8 * 5);
	u_z.base |= ((typeof(u_z.base)) (*(out_data + offset + 6))) << (8 * 6);
	u_z.base |= ((typeof(u_z.base)) (*(out_data + offset + 7))) << (8 * 7);
	this->z = u_z.real;
	offset += sizeof(this->z);
	return offset;
}
ros::MsgSz Vector3::bytes()
{
	ros::MsgSz msgSize = 0;
	msgSize += sizeof(ros::float64);
	msgSize += sizeof(ros::float64);
	msgSize += sizeof(ros::float64);
	return msgSize;
}
