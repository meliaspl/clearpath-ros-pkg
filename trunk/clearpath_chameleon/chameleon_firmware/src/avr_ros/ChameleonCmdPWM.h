/* ChameleonCmdPWM.h
 * MSG file auto generated by "Rutgers avr_bridge"
 */
#ifndef CHAMELEONCMDPWM_H_
#define CHAMELEONCMDPWM_H_
#include "avr_ros/Msg.h"
#include "avr_ros/vector.h"
#include "avr_ros/ros_string.h"
#include "avr_ros/ros_float64.h"
namespace chameleon_msgs {
	class ChameleonCmdPWM : public ros::Msg {
	public:
		virtual ros::MsgSz bytes();
		virtual ros::MsgSz serialize(uint8_t *out_buffer);
		virtual ros::MsgSz deserialize(uint8_t *data);
		float left;
		float right;
	}; /* class ChameleonCmdPWM */
} /* namespace chameleon_msgs */
#endif /* CHAMELEONCMDPWM_H_ */