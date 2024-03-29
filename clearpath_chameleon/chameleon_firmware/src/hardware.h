#ifndef HARDWARE_H
#define HARDWARE_H

#define PIN_ENCA_LEFT 4
#define PIN_ENCB_LEFT 5
#define PIN_DIR_LEFT 6
#define PIN_DIR_RIGHT 7
#define PIN_PWM_LEFT 9
#define PIN_PWM_RIGHT 10
#define PIN_ENCA_RIGHT 11
#define PIN_ENCB_RIGHT 12

#define PIN_DIO1 2
#define PIN_DIO2 3
#define PIN_DIO3 8

#define PIN_HEARTBEAT 13

#define PIN_VSENSE 0
#define VSENSE_SCALE 0.01512
#define VSENSE_OFFSET 0.0

#define BATT_THRESH_LOW 12.1
#define BATT_THRESH_MED 12.7
#define BATT_THRESH_HIGH 13.2
#define BATT_THRESH_FULL 13.6


#define HEARTBEAT_DRIVING_PERIOD_MS 500
#define HEARTBEAT_TIMEOUT_PERIOD_MS 100
#define SENSE_PERIOD_MS 100
#define CONTROL_PERIOD_MS 20
#define SONAR_PERIOD_MS 80
#define CMD_TIMEOUT_MS 500
#define CONTROL_HZ (1000 / CONTROL_PERIOD_MS)

#define MAX_SPEED 0.5  // m/s
#define MAX_ACCEL 10.0  // m/s^2
#define MOTOR_SLEW 0.1  // Full range in 10 control periods.
#define TRACK 0.26

#define TICKS_PER_METER 26700
#define METERS_PER_TICK (1.0/TICKS_PER_METER)

#define I2C_READ_TIMEOUT_MS 5

#define IOX_ADDR         (uint8_t)0x70
#define IOX_CONFIG_ADDR  (uint8_t)0x03
#define IOX_CONFIG_DATA  (uint8_t)0x00
#define IOX_OUTPUT_ADDR  (uint8_t)0x01

#define SRF02_COUNT      0
#define SRF02_ADDR       (uint8_t)0x71
#define SRF02_CMD        (uint8_t)0x00
#define SRF02_RANGE_CM   (uint8_t)0x51

#define MUX_ADC_COUNT    0
#define MUX_PIN0         PIN_DIO3
#define MUX_PIN1         PIN_DIO2
#define MUX_PIN2         PIN_DIO1

#define RANGE_COUNT (SRF02_COUNT + MUX_ADC_COUNT)

#endif
