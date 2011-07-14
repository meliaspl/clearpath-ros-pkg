
#include "Wire/Wire.h"

// ROS avr_bridge headers
#include "avr_ros/ros.h"
#include "avr_ros/ChameleonSense.h"
#include "avr_ros/ChameleonCmdPWM.h"
#include "avr_ros/ChameleonCmdSpeed.h"
#include "avr_ros/Twist.h"

// Projects headers
#include "hardware.h"
#include "encoder.h"
#include "fixed.h"
#include "pid.h"



ros::Publisher sense_pub;

chameleon_msgs::ChameleonCmdPWM pwm_msg;
chameleon_msgs::ChameleonCmdSpeed speed_msg;
geometry_msgs::Twist vel_msg;
chameleon_msgs::ChameleonSense sense_msg;


#define LEFT 0
#define RIGHT 1

Encoder encoders[2];


#define MODE_NONE 0
#define MODE_PWM 1
#define MODE_SPEED 2
#define MODE_ERROR 255

uint8_t mode = MODE_NONE;
long int last_cmd_millis = 0;

const Fixed motor_max = Fixed::from_raw(255);
const Fixed speed_max = Fixed::from_raw(MAX_SPEED * TICKS_PER_METER);

FixedIncr motor_speed[2] = { FixedIncr(0.0, speed_max, -speed_max),
                             FixedIncr(0.0, speed_max, -speed_max) };

const Fixed pid_output_scale = Fixed(64);
FixedPIDConstants pid_constants(1.8, 0.1, 0.3, 
                                motor_max * pid_output_scale, 
                                -motor_max * pid_output_scale);
FixedPID motor_pid[2] = { FixedPID(pid_constants),
                          FixedPID(pid_constants) };

FixedIncr motor_pwm[2] = { FixedIncr(MOTOR_SLEW, motor_max, -motor_max),
                           FixedIncr(MOTOR_SLEW, motor_max, -motor_max) };

Fixed f0(int(0));


// Have we received a cmd within the timeout window?
int timed_out() {
  return millis() > last_cmd_millis + CMD_TIMEOUT_MS;
}


void write_pwm(uint8_t pin_dir, uint8_t pin_pwm, Fixed pwm) {
  if (pwm > f0) {
    digitalWrite(pin_dir, 1);
    analogWrite(pin_pwm, pwm.get_byte(0));
  } else {
    digitalWrite(pin_dir, 0);
    analogWrite(pin_pwm, (-pwm).get_byte(0));
  }
}


void write_leds(uint8_t leds) {
  Wire.beginTransmission(IOX_ADDR);
  Wire.send(IOX_OUTPUT_ADDR);
  Wire.send(leds);
  Wire.endTransmission();
}


void update_leds() {
  uint8_t leds = 0x0;
  if (!timed_out()) leds |= 0x1;
  if (mode == MODE_ERROR) leds |= 0x2;
  if (sense_msg.voltage > BATT_THRESH_LOW) leds |= 4;
  if (sense_msg.voltage > BATT_THRESH_MED) leds |= 8;
  if (sense_msg.voltage > BATT_THRESH_HIGH) leds |= 16;
  if (sense_msg.voltage > BATT_THRESH_FULL) leds |= 32;
  write_leds(leds);
}



// Receive ROS messages.
void cmd(const ros::Msg *msg) {
  // Assume bad until shown otherwise.
  mode = MODE_ERROR;
  last_cmd_millis = millis();

  if (msg == &pwm_msg) {
    // Bounds check
    if (pwm_msg.left < -1 || pwm_msg.left > 1) return;
    if (pwm_msg.right < -1 || pwm_msg.right > 1) return;

    // Set output. The -1..1 float value maps nicely into
    // the lower byte of the 8x8 Fixed.
    motor_pwm[LEFT].target = pwm_msg.left;
    motor_pwm[RIGHT].target = pwm_msg.right;
    mode = MODE_PWM;

  } else if (msg == &speed_msg || msg == &vel_msg) {
    if (msg == &vel_msg) {
      // Populate speed msg from data in the vel msg.
      float trans = vel_msg.linear.x.convert();
      float rot = vel_msg.angular.z.convert();
      speed_msg.left = trans - (rot * TRACK / 2);
      speed_msg.right = trans + (rot * TRACK / 2);
      speed_msg.accel = MAX_ACCEL;  // default
    }
    if (speed_msg.left < -MAX_SPEED || speed_msg.left > MAX_SPEED) return;
    if (speed_msg.right < -MAX_SPEED || speed_msg.right > MAX_SPEED) return;
    if (speed_msg.accel < 0 || speed_msg.accel > MAX_ACCEL) return;

    // Default acceleration if zero given.
    if (speed_msg.accel == 0.0) speed_msg.accel = MAX_ACCEL;

    // Speed in 8x8 is expressed as ticks per 256th of a second;
    // the max speed in those units is around 50.
    motor_speed[LEFT].target = speed_msg.left * TICKS_PER_METER / 256;
    motor_speed[RIGHT].target = speed_msg.right * TICKS_PER_METER / 256;

    // Acceleration in speed change per control period.
    motor_speed[LEFT].rate = motor_speed[RIGHT].rate =
      (speed_msg.accel * TICKS_PER_METER / 256) / CONTROL_HZ;

    mode = MODE_SPEED;
  }
}


void setup() {
  // Set up serial comm.
  Serial.begin(57600);
  
  // Set up IO.
  pinMode(PIN_ENCA_LEFT, INPUT);
  pinMode(PIN_ENCB_LEFT, INPUT);
  pinMode(PIN_ENCA_RIGHT, INPUT);
  pinMode(PIN_ENCB_RIGHT, INPUT);
  
  pinMode(PIN_PWM_LEFT, OUTPUT);
  pinMode(PIN_PWM_RIGHT, OUTPUT);
  pinMode(PIN_DIR_LEFT, OUTPUT);
  pinMode(PIN_DIR_RIGHT, OUTPUT);

  pinMode(PIN_HEARTBEAT, OUTPUT);
  
  // Enable pin change interrupts for left encoder (port 2)
  // and right encoder (port 1).
  PCMSK2 = 1 << (PIN_ENCA_LEFT - 0) | 1 << (PIN_ENCB_LEFT - 0);
  PCMSK0 = 1 << (PIN_ENCA_RIGHT - 8) | 1 << (PIN_ENCB_RIGHT - 8);
  PCICR |= 1 << 0 | 1 << 2;

  // Enable PWM at 4khz frequency on pins 9 and 10.
  // See: http://www.arduino.cc/playground/Code/PwmFrequency
  TCCR1B = TCCR1B & 0b11111000 | 0x02;

  // Set up IO expander for LEDs.
  Wire.begin();
  Wire.beginTransmission(IOX_ADDR);
  Wire.send(IOX_CONFIG_ADDR);
  Wire.send(IOX_CONFIG_DATA);
  Wire.endTransmission();

  // Startup flash.
  write_leds(0b00111111);
  delay(800);
  write_leds(0);
  delay(300);

  // Set up ROS messaging.
  sense_pub = node.advertise("sense");
  node.subscribe("cmd_pwm", cmd, &pwm_msg);
  node.subscribe("cmd_speed", cmd, &speed_msg);
  node.subscribe("cmd_vel", cmd, &vel_msg);
}
 


void loop() {
  // Handle received ROS messages.
  node.spin();
  
  // Task: Control loop
  static long int control_millis = 0;  
  if (millis() > control_millis) {
    control_millis += CONTROL_PERIOD_MS;
    encoders[LEFT].compute_speed(millis());
    encoders[RIGHT].compute_speed(millis());
    
    if (mode == MODE_SPEED) {
      // Advance speed toward commanded at accel rate.
      motor_speed[LEFT].step();
      motor_speed[RIGHT].step();

      // Insert speed targets from accel into PID controller.
      motor_pid[LEFT].setpoint = motor_speed[LEFT].actual;
      motor_pid[RIGHT].setpoint = motor_speed[RIGHT].actual;
      motor_pid[LEFT].input = Fixed::from_raw(encoders[LEFT].ticks_per_second);
      motor_pid[RIGHT].input = Fixed::from_raw(encoders[RIGHT].ticks_per_second);
      motor_pid[LEFT].step();
      motor_pid[RIGHT].step();

      motor_pwm[LEFT].target = motor_pid[LEFT].output / pid_output_scale;
      motor_pwm[RIGHT].target = motor_pid[RIGHT].output / pid_output_scale;
    }

    // Advance PWM toward targets at slew rate.
    motor_pwm[LEFT].step();
    motor_pwm[RIGHT].step();

    if (timed_out() || mode == MODE_ERROR) {
      // Timeout or a bad message. Reset everything.
      motor_pwm[LEFT].actual = 0;
      motor_pwm[RIGHT].actual = 0;
      motor_speed[LEFT].actual = 0;
      motor_speed[RIGHT].actual = 0;
    }

    // Write PWM to motors.
    write_pwm(PIN_DIR_LEFT, PIN_PWM_LEFT, motor_pwm[LEFT].actual);
    write_pwm(PIN_DIR_RIGHT, PIN_PWM_RIGHT, motor_pwm[RIGHT].actual);
  }
  
  // Task: Send ROS sensor msg, update HMI LEDs.
  static long int sense_millis = 0;  
  if (millis() > sense_millis) {
    sense_millis += SENSE_PERIOD_MS;
    sense_msg.travel_left = METERS_PER_TICK * encoders[LEFT].ticks;
    sense_msg.travel_right = METERS_PER_TICK * encoders[RIGHT].ticks;
    sense_msg.speed_left = METERS_PER_TICK * encoders[LEFT].ticks_per_second;
    sense_msg.speed_right = METERS_PER_TICK * encoders[RIGHT].ticks_per_second;
    sense_msg.pwm_left = float(motor_pwm[LEFT].actual);
    sense_msg.pwm_right = float(motor_pwm[RIGHT].actual);
    sense_msg.voltage = analogRead(PIN_VSENSE) * VSENSE_SCALE + VSENSE_OFFSET;
    node.publish(sense_pub, &sense_msg);

    update_leds();
  }
  
  // Task: Heartbeat
  static long int heartbeat_millis = 0;
  if (millis() > heartbeat_millis) {
    static char heartbeat = 0;
    if (timed_out()) {
      heartbeat_millis += HEARTBEAT_TIMEOUT_PERIOD_MS;      
    } else {
      heartbeat_millis += HEARTBEAT_DRIVING_PERIOD_MS;
    }
    digitalWrite(13, heartbeat ^= 1);
  }  
}


// Encoder interrupt handlers
ISR(PCINT2_vect) {
  encoders[LEFT].tick(digitalRead(PIN_ENCB_LEFT), digitalRead(PIN_ENCA_LEFT));
}

ISR(PCINT0_vect) {
  encoders[RIGHT].tick(digitalRead(PIN_ENCB_RIGHT), digitalRead(PIN_ENCA_RIGHT));
}


// Communication for ROS avr_bridge.
namespace ros {
  int fputc(char c, FILE *f) {
    Serial.write(c);
  }
  int fgetc(FILE *f) {
    return Serial.read();
  }
}
