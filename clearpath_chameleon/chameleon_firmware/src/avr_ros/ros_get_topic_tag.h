/* This file was autogenerated as a part of the avr_bridge pkg
 * avr_bridge was written by Adam Stambler and Phillip Quiza of
 * Rutgers University.
 */
char getTopicTag(char const* topic) {
	if (!strcmp(topic, "cmd_speed"))
		return 0;
	if (!strcmp(topic, "cmd_pwm"))
		return 1;
	if (!strcmp(topic, "sense"))
		return 3;
	if (!strcmp(topic, "cmd_vel"))
		return 2;
	return 0;
} /* getTopicTag */
