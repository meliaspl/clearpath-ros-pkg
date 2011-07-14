#include "avr_ros/ChameleonSense.h"
using namespace chameleon_msgs;
ros::MsgSz ChameleonSense::serialize(uint8_t *in_data)
{
	ros::MsgSz offset = 0;
	union {
		float real;
		uint32_t base;
	} u_travel_left;
	u_travel_left.real = this->travel_left;
	*(in_data + offset + 0) = (u_travel_left.base >> (8 * 0)) & 0xFF;
	*(in_data + offset + 1) = (u_travel_left.base >> (8 * 1)) & 0xFF;
	*(in_data + offset + 2) = (u_travel_left.base >> (8 * 2)) & 0xFF;
	*(in_data + offset + 3) = (u_travel_left.base >> (8 * 3)) & 0xFF;
	offset += sizeof(this->travel_left);
	union {
		float real;
		uint32_t base;
	} u_travel_right;
	u_travel_right.real = this->travel_right;
	*(in_data + offset + 0) = (u_travel_right.base >> (8 * 0)) & 0xFF;
	*(in_data + offset + 1) = (u_travel_right.base >> (8 * 1)) & 0xFF;
	*(in_data + offset + 2) = (u_travel_right.base >> (8 * 2)) & 0xFF;
	*(in_data + offset + 3) = (u_travel_right.base >> (8 * 3)) & 0xFF;
	offset += sizeof(this->travel_right);
	union {
		float real;
		uint32_t base;
	} u_speed_left;
	u_speed_left.real = this->speed_left;
	*(in_data + offset + 0) = (u_speed_left.base >> (8 * 0)) & 0xFF;
	*(in_data + offset + 1) = (u_speed_left.base >> (8 * 1)) & 0xFF;
	*(in_data + offset + 2) = (u_speed_left.base >> (8 * 2)) & 0xFF;
	*(in_data + offset + 3) = (u_speed_left.base >> (8 * 3)) & 0xFF;
	offset += sizeof(this->speed_left);
	union {
		float real;
		uint32_t base;
	} u_speed_right;
	u_speed_right.real = this->speed_right;
	*(in_data + offset + 0) = (u_speed_right.base >> (8 * 0)) & 0xFF;
	*(in_data + offset + 1) = (u_speed_right.base >> (8 * 1)) & 0xFF;
	*(in_data + offset + 2) = (u_speed_right.base >> (8 * 2)) & 0xFF;
	*(in_data + offset + 3) = (u_speed_right.base >> (8 * 3)) & 0xFF;
	offset += sizeof(this->speed_right);
	union {
		float real;
		uint32_t base;
	} u_pwm_left;
	u_pwm_left.real = this->pwm_left;
	*(in_data + offset + 0) = (u_pwm_left.base >> (8 * 0)) & 0xFF;
	*(in_data + offset + 1) = (u_pwm_left.base >> (8 * 1)) & 0xFF;
	*(in_data + offset + 2) = (u_pwm_left.base >> (8 * 2)) & 0xFF;
	*(in_data + offset + 3) = (u_pwm_left.base >> (8 * 3)) & 0xFF;
	offset += sizeof(this->pwm_left);
	union {
		float real;
		uint32_t base;
	} u_pwm_right;
	u_pwm_right.real = this->pwm_right;
	*(in_data + offset + 0) = (u_pwm_right.base >> (8 * 0)) & 0xFF;
	*(in_data + offset + 1) = (u_pwm_right.base >> (8 * 1)) & 0xFF;
	*(in_data + offset + 2) = (u_pwm_right.base >> (8 * 2)) & 0xFF;
	*(in_data + offset + 3) = (u_pwm_right.base >> (8 * 3)) & 0xFF;
	offset += sizeof(this->pwm_right);
	union {
		float real;
		uint32_t base;
	} u_voltage;
	u_voltage.real = this->voltage;
	*(in_data + offset + 0) = (u_voltage.base >> (8 * 0)) & 0xFF;
	*(in_data + offset + 1) = (u_voltage.base >> (8 * 1)) & 0xFF;
	*(in_data + offset + 2) = (u_voltage.base >> (8 * 2)) & 0xFF;
	*(in_data + offset + 3) = (u_voltage.base >> (8 * 3)) & 0xFF;
	offset += sizeof(this->voltage);
	return offset;
}
ros::MsgSz ChameleonSense::deserialize(uint8_t *out_data)
{
	ros::MsgSz offset = 0;
	union {
		float real;
		uint32_t base;
	} u_travel_left;
	u_travel_left.base = 0;
	u_travel_left.base |= ((typeof(u_travel_left.base)) (*(out_data + offset + 0))) << (8 * 0);
	u_travel_left.base |= ((typeof(u_travel_left.base)) (*(out_data + offset + 1))) << (8 * 1);
	u_travel_left.base |= ((typeof(u_travel_left.base)) (*(out_data + offset + 2))) << (8 * 2);
	u_travel_left.base |= ((typeof(u_travel_left.base)) (*(out_data + offset + 3))) << (8 * 3);
	this->travel_left = u_travel_left.real;
	offset += sizeof(this->travel_left);
	union {
		float real;
		uint32_t base;
	} u_travel_right;
	u_travel_right.base = 0;
	u_travel_right.base |= ((typeof(u_travel_right.base)) (*(out_data + offset + 0))) << (8 * 0);
	u_travel_right.base |= ((typeof(u_travel_right.base)) (*(out_data + offset + 1))) << (8 * 1);
	u_travel_right.base |= ((typeof(u_travel_right.base)) (*(out_data + offset + 2))) << (8 * 2);
	u_travel_right.base |= ((typeof(u_travel_right.base)) (*(out_data + offset + 3))) << (8 * 3);
	this->travel_right = u_travel_right.real;
	offset += sizeof(this->travel_right);
	union {
		float real;
		uint32_t base;
	} u_speed_left;
	u_speed_left.base = 0;
	u_speed_left.base |= ((typeof(u_speed_left.base)) (*(out_data + offset + 0))) << (8 * 0);
	u_speed_left.base |= ((typeof(u_speed_left.base)) (*(out_data + offset + 1))) << (8 * 1);
	u_speed_left.base |= ((typeof(u_speed_left.base)) (*(out_data + offset + 2))) << (8 * 2);
	u_speed_left.base |= ((typeof(u_speed_left.base)) (*(out_data + offset + 3))) << (8 * 3);
	this->speed_left = u_speed_left.real;
	offset += sizeof(this->speed_left);
	union {
		float real;
		uint32_t base;
	} u_speed_right;
	u_speed_right.base = 0;
	u_speed_right.base |= ((typeof(u_speed_right.base)) (*(out_data + offset + 0))) << (8 * 0);
	u_speed_right.base |= ((typeof(u_speed_right.base)) (*(out_data + offset + 1))) << (8 * 1);
	u_speed_right.base |= ((typeof(u_speed_right.base)) (*(out_data + offset + 2))) << (8 * 2);
	u_speed_right.base |= ((typeof(u_speed_right.base)) (*(out_data + offset + 3))) << (8 * 3);
	this->speed_right = u_speed_right.real;
	offset += sizeof(this->speed_right);
	union {
		float real;
		uint32_t base;
	} u_pwm_left;
	u_pwm_left.base = 0;
	u_pwm_left.base |= ((typeof(u_pwm_left.base)) (*(out_data + offset + 0))) << (8 * 0);
	u_pwm_left.base |= ((typeof(u_pwm_left.base)) (*(out_data + offset + 1))) << (8 * 1);
	u_pwm_left.base |= ((typeof(u_pwm_left.base)) (*(out_data + offset + 2))) << (8 * 2);
	u_pwm_left.base |= ((typeof(u_pwm_left.base)) (*(out_data + offset + 3))) << (8 * 3);
	this->pwm_left = u_pwm_left.real;
	offset += sizeof(this->pwm_left);
	union {
		float real;
		uint32_t base;
	} u_pwm_right;
	u_pwm_right.base = 0;
	u_pwm_right.base |= ((typeof(u_pwm_right.base)) (*(out_data + offset + 0))) << (8 * 0);
	u_pwm_right.base |= ((typeof(u_pwm_right.base)) (*(out_data + offset + 1))) << (8 * 1);
	u_pwm_right.base |= ((typeof(u_pwm_right.base)) (*(out_data + offset + 2))) << (8 * 2);
	u_pwm_right.base |= ((typeof(u_pwm_right.base)) (*(out_data + offset + 3))) << (8 * 3);
	this->pwm_right = u_pwm_right.real;
	offset += sizeof(this->pwm_right);
	union {
		float real;
		uint32_t base;
	} u_voltage;
	u_voltage.base = 0;
	u_voltage.base |= ((typeof(u_voltage.base)) (*(out_data + offset + 0))) << (8 * 0);
	u_voltage.base |= ((typeof(u_voltage.base)) (*(out_data + offset + 1))) << (8 * 1);
	u_voltage.base |= ((typeof(u_voltage.base)) (*(out_data + offset + 2))) << (8 * 2);
	u_voltage.base |= ((typeof(u_voltage.base)) (*(out_data + offset + 3))) << (8 * 3);
	this->voltage = u_voltage.real;
	offset += sizeof(this->voltage);
	return offset;
}
ros::MsgSz ChameleonSense::bytes()
{
	ros::MsgSz msgSize = 0;
	msgSize += sizeof(float);
	msgSize += sizeof(float);
	msgSize += sizeof(float);
	msgSize += sizeof(float);
	msgSize += sizeof(float);
	msgSize += sizeof(float);
	msgSize += sizeof(float);
	return msgSize;
}
