#include <PID_v1.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "consts.h"
//Arda Abi e e waka waka eee eee
//Create interrupts functions and variables
unsigned long rr_encoder_counter, lr_encoder_counter, lf_encoder_counter, rf_encoder_counter;
unsigned long rr_encoder_old_counter, lr_encoder_old_counter, lf_encoder_old_counter, rf_encoder_old_counter;
unsigned long time_;

void lf_encoder_interrupt() {lf_encoder_counter = lf_encoder_counter + 1;}
void rf_encoder_interrupt() {rf_encoder_counter = rf_encoder_counter + 1;}
void lr_encoder_interrupt() {lr_encoder_counter = lr_encoder_counter + 1;}
void rr_encoder_interrupt() {rr_encoder_counter = rr_encoder_counter + 1;}

//Create controlled objects for steering and motors
double lr_pid_input = 0, lr_pid_output, lr_pid_setpoint = 0;
double rr_pid_input = 0, rr_pid_output, rr_pid_setpoing = 0;
double steer_pid_input, steer_pid_output, steer_pid_setpoing;

PID PID_left_motor(&lr_pid_input, &lr_pid_output, &lr_pid_setpoint, 17, 10, .03, DIRECT);
PID PID_right_motor(&rr_pid_input, &rr_pid_output, &rr_pid_setpoing, 15, 7, .04, DIRECT);
PID PID_steering(&steer_pid_input, &steer_pid_output, &steer_pid_setpoing, 0.3, 0.004, 0.05, DIRECT);

//Serial communication variables
int command_array[] = {0, 0, 0, 0, 0, 0, 0, 0}, bno_turn = 0; //speed, angular_speed, bno055 heading, drive_mode * * * *

struct speed_values {
  double linear;
  double angular;
};

speed_values speed_req;

//Create motor driver objects
Servo left_motor, right_motor;

//init i2c servo driver and set steering first position
Adafruit_PWMServoDriver servo_driver = Adafruit_PWMServoDriver(0x40);
int steering_remote_array[6], steering_remote_array_counter;

void setup() {
  servo_driver.begin();
  servo_driver.setPWMFreq(333);
  servo_driver.setPWM(RIGHT_SERVO_PIN, 850, 4096 - 850);
  servo_driver.setPWM(LEFT_SERVO_PIN, 850, 4096 - 850);

  pinMode(RC_CHANNEL_ONE, INPUT); 
  pinMode(RC_CHANNEL_TWO, INPUT); 

  left_motor.attach(LEFT_MOTOR_PIN);
  right_motor.attach(RIGHT_MOTOR_PIN);

  Serial.begin(230400);
  Serial.setTimeout(5);

  left_motor.writeMicroseconds(1000);
  right_motor.writeMicroseconds(1000);

  //delay(6000);

  pinMode(ENCODER_RR, INPUT_PULLUP);
  pinMode(ENCODER_LR, INPUT_PULLUP);
  pinMode(ENCODER_LF, INPUT_PULLUP);
  pinMode(ENCODER_RF, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LR), lr_encoder_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RR), rr_encoder_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LF), lf_encoder_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RF), rf_encoder_interrupt, RISING);

  PID_left_motor.SetMode(AUTOMATIC);
  PID_right_motor.SetMode(AUTOMATIC);
  PID_steering.SetMode(AUTOMATIC);

  PID_right_motor.SetOutputLimits(998, 2000);
  PID_left_motor.SetOutputLimits(998, 2000);
  PID_steering.SetOutputLimits(-300, 300);
}

void loop() {
  update_command_array();

  int drive_mode = command_array[3];
  drive_mode = 1;
  if (drive_mode == 0) { //drive via RC
    speed_req = read_remote_values();
  }
  else if (drive_mode == 1) { //drive via jetson nano
    speed_req = {command_array[0] / 100.0, (- command_array[1] + 1500) + command_array[2]};
  }

  update_speed_setpoints(speed_req);


  if ((time_ + 100) < millis()) {update_execute_pid();}
}
