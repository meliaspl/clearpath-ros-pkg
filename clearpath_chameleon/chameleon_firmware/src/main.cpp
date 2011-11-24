


#include <WProgram.h>
#include "I2C/I2C.h"

// ROS
#include <ros.h>
#include <chameleon_msgs/ChameleonSense.h>
#include <chameleon_msgs/ChameleonCmdPWM.h>
#include <chameleon_msgs/ChameleonCmdSpeed.h>
#include <geometry_msgs/Twist.h>

// Chameleon software
#include "hardware.h"
#include "encoder.h"
#include "fixed.h"
#include "pid.h"


ros::NodeHandle_<ArduinoHardware, 5, 5, 100, 100> nh;

chameleon_msgs::ChameleonSense sense_msg;
ros::Publisher sense_pub("sense", &sense_msg);

void cmd_speed(const chameleon_msgs::ChameleonCmdSpeed&);
ros::Subscriber<chameleon_msgs::ChameleonCmdSpeed> speed_sub("cmd_speed", &cmd_speed);

void cmd_pwm(const chameleon_msgs::ChameleonCmdPWM&);
ros::Subscriber<chameleon_msgs::ChameleonCmdPWM> pwm_sub("cmd_pwm", &cmd_pwm);

#define LEFT 0
#define RIGHT 1

Encoder<CONTROL_PERIOD_MS> encoders[2];

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


uint16_t ranges[SRF02_COUNT];
uint8_t current_range = 0;


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
  I2c.write(IOX_ADDR, IOX_OUTPUT_ADDR, leds);
}


void update_leds() {
  uint8_t leds = 0x0;
  if (timed_out()) leds |= 0x1;
  if (mode == MODE_ERROR) leds |= 0x2;
  if (sense_msg.voltage > BATT_THRESH_LOW) leds |= 4;
  if (sense_msg.voltage > BATT_THRESH_MED) leds |= 8;
  if (sense_msg.voltage > BATT_THRESH_HIGH) leds |= 16;
  if (sense_msg.voltage > BATT_THRESH_FULL) leds |= 32;
  write_leds(leds);
}

void cmd_pwm(const chameleon_msgs::ChameleonCmdPWM& pwm_msg) {
  // Assume bad.
  mode = MODE_ERROR;
  last_cmd_millis = millis();

  // Bounds check
  if (pwm_msg.left < -1 || pwm_msg.left > 1) return;
  if (pwm_msg.right < -1 || pwm_msg.right > 1) return;

  // Set output. The -1..1 float value maps nicely into
  // the lower byte of the 8x8 Fixed.
  motor_pwm[LEFT].target = pwm_msg.left;
  motor_pwm[RIGHT].target = pwm_msg.right;
  mode = MODE_PWM;
}

void cmd_speed(const chameleon_msgs::ChameleonCmdSpeed& speed_msg) {
  // Assume bad.
  mode = MODE_ERROR;
  last_cmd_millis = millis();

  // Bounds check.
  if (speed_msg.left < -MAX_SPEED || speed_msg.left > MAX_SPEED) return;
  if (speed_msg.right < -MAX_SPEED || speed_msg.right > MAX_SPEED) return;
  if (speed_msg.accel < 0 || speed_msg.accel > MAX_ACCEL) return;

  // Default acceleration if zero given.
  float accel = speed_msg.accel;
  if (accel == 0.0) accel = MAX_ACCEL;

  // Speed in 8x8 is expressed as ticks per 256th of a second;
  // the max speed in those units is around 50.
  motor_speed[LEFT].target = speed_msg.left * TICKS_PER_METER / 256;
  motor_speed[RIGHT].target = speed_msg.right * TICKS_PER_METER / 256;

  // Acceleration in speed change per control period.
  motor_speed[LEFT].rate = motor_speed[RIGHT].rate =
    (accel * TICKS_PER_METER / 256) / CONTROL_HZ;

  mode = MODE_SPEED;
}


void setup() {
  // Set up IO.
  // pinMode(PIN_VSENSE, INPUT);
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
  I2c.begin();
  I2c.timeOut(I2C_READ_TIMEOUT_MS);
  I2c.write(IOX_ADDR, IOX_CONFIG_ADDR, IOX_CONFIG_DATA);
            
  // Startup flash.
  write_leds(0b00111111);
  delay(300);
  write_leds(0);
  delay(100);
  write_leds(0b00000001);

  // Set up ROS.
  nh.initNode();
  //write_leds(0b00000000);

  nh.advertise(sense_pub);
  nh.subscribe(speed_sub);
  nh.subscribe(pwm_sub);
}
 

void loop() {
  // Handle received ROS messages.
  nh.spinOnce();
  
  // Task: Control loop
  static long int control_millis = 0;  
  if (millis() > control_millis) {
    control_millis += CONTROL_PERIOD_MS;
    
    if (mode == MODE_SPEED) {
      // Advance speed toward commanded at accel rate.
      motor_speed[LEFT].step();
      motor_speed[RIGHT].step();

      // Insert speed targets from accel into PID controller.
      motor_pid[LEFT].setpoint = motor_speed[LEFT].actual;
      motor_pid[RIGHT].setpoint = motor_speed[RIGHT].actual;
      motor_pid[LEFT].input = Fixed::from_raw(encoders[LEFT].get_ticks_per_second());
      motor_pid[RIGHT].input = Fixed::from_raw(encoders[RIGHT].get_ticks_per_second());
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

    // Round-robin through rangefinders, one every 80ms.
    #if(SRF02_COUNT > 0)
    static uint8_t sonar_count = 0;
    sonar_count++;
    if (sonar_count >= (SONAR_PERIOD_MS / CONTROL_PERIOD_MS)) { 
      sonar_count = 0;

      // Read previous value.
      uint8_t* byte_ptr = (uint8_t*)&ranges[current_range];
      I2c.read(SRF02_ADDR + current_range, 0x02, 1, byte_ptr+1);
      I2c.read(SRF02_ADDR + current_range, 0x03, 1, byte_ptr);

      // Transmit request-to-ping for next sonar.
      if (++current_range >= SRF02_COUNT) current_range = 0;
      I2c.write((uint8_t)(SRF02_ADDR + current_range), SRF02_CMD, SRF02_RANGE_CM);
    }
    #endif

    // Send ROS sensor msg, update HMI LEDs (every nth control period).
    static uint8_t sense_count = 0;
    sense_count++;
    if (sense_count >= (SENSE_PERIOD_MS / CONTROL_PERIOD_MS)) {
      sense_count = 0;
      sense_msg.travel_left = METERS_PER_TICK * encoders[LEFT].ticks;
      sense_msg.travel_right = METERS_PER_TICK * encoders[RIGHT].ticks;
      sense_msg.speed_left = METERS_PER_TICK * encoders[LEFT].get_ticks_per_second();
      sense_msg.speed_right = METERS_PER_TICK * encoders[RIGHT].get_ticks_per_second();
      sense_msg.pwm_left = float(motor_pwm[LEFT].actual);
      sense_msg.pwm_right = float(motor_pwm[RIGHT].actual);
      sense_msg.voltage = analogRead(PIN_VSENSE) * VSENSE_SCALE + VSENSE_OFFSET;
 
      sense_msg.ranges_length = SRF02_COUNT; 
      sense_msg.ranges = ranges;

      sense_pub.publish(&sense_msg);
      update_leds();
    }    

    // Reset encoders
    encoders[LEFT].reset_ticks_per_second();
    encoders[RIGHT].reset_ticks_per_second();
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

