#include "avr_ros/ChameleonCmdSpeed.h"
using namespace chameleon_msgs;
ros::MsgSz ChameleonCmdSpeed::serialize(uint8_t *in_data)
{
	ros::MsgSz offset = 0;
	union {
		float real;
		uint32_t base;
	} u_left;
	u_left.real = this->left;
	*(in_data + offset + 0) = (u_left.base >> (8 * 0)) & 0xFF;
	*(in_data + offset + 1) = (u_left.base >> (8 * 1)) & 0xFF;
	*(in_data + offset + 2) = (u_left.base >> (8 * 2)) & 0xFF;
	*(in_data + offset + 3) = (u_left.base >> (8 * 3)) & 0xFF;
	offset += sizeof(this->left);
	union {
		float real;
		uint32_t base;
	} u_right;
	u_right.real = this->right;
	*(in_data + offset + 0) = (u_right.base >> (8 * 0)) & 0xFF;
	*(in_data + offset + 1) = (u_right.base >> (8 * 1)) & 0xFF;
	*(in_data + offset + 2) = (u_right.base >> (8 * 2)) & 0xFF;
	*(in_data + offset + 3) = (u_right.base >> (8 * 3)) & 0xFF;
	offset += sizeof(this->right);
	union {
		float real;
		uint32_t base;
	} u_accel;
	u_accel.real = this->accel;
	*(in_data + offset + 0) = (u_accel.base >> (8 * 0)) & 0xFF;
	*(in_data + offset + 1) = (u_accel.base >> (8 * 1)) & 0xFF;
	*(in_data + offset + 2) = (u_accel.base >> (8 * 2)) & 0xFF;
	*(in_data + offset + 3) = (u_accel.base >> (8 * 3)) & 0xFF;
	offset += sizeof(this->accel);
	return offset;
}
ros::MsgSz ChameleonCmdSpeed::deserialize(uint8_t *out_data)
{
	ros::MsgSz offset = 0;
	union {
		float real;
		uint32_t base;
	} u_left;
	u_left.base = 0;
	u_left.base |= ((typeof(u_left.base)) (*(out_data + offset + 0))) << (8 * 0);
	u_left.base |= ((typeof(u_left.base)) (*(out_data + offset + 1))) << (8 * 1);
	u_left.base |= ((typeof(u_left.base)) (*(out_data + offset + 2))) << (8 * 2);
	u_left.base |= ((typeof(u_left.base)) (*(out_data + offset + 3))) << (8 * 3);
	this->left = u_left.real;
	offset += sizeof(this->left);
	union {
		float real;
		uint32_t base;
	} u_right;
	u_right.base = 0;
	u_right.base |= ((typeof(u_right.base)) (*(out_data + offset + 0))) << (8 * 0);
	u_right.base |= ((typeof(u_right.base)) (*(out_data + offset + 1))) << (8 * 1);
	u_right.base |= ((typeof(u_right.base)) (*(out_data + offset + 2))) << (8 * 2);
	u_right.base |= ((typeof(u_right.base)) (*(out_data + offset + 3))) << (8 * 3);
	this->right = u_right.real;
	offset += sizeof(this->right);
	union {
		float real;
		uint32_t base;
	} u_accel;
	u_accel.base = 0;
	u_accel.base |= ((typeof(u_accel.base)) (*(out_data + offset + 0))) << (8 * 0);
	u_accel.base |= ((typeof(u_accel.base)) (*(out_data + offset + 1))) << (8 * 1);
	u_accel.base |= ((typeof(u_accel.base)) (*(out_data + offset + 2))) << (8 * 2);
	u_accel.base |= ((typeof(u_accel.base)) (*(out_data + offset + 3))) << (8 * 3);
	this->accel = u_accel.real;
	offset += sizeof(this->accel);
	return offset;
}
ros::MsgSz ChameleonCmdSpeed::bytes()
{
	ros::MsgSz msgSize = 0;
	msgSize += sizeof(float);
	msgSize += sizeof(float);
	msgSize += sizeof(float);
	return msgSize;
}
