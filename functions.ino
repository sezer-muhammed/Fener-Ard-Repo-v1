// This is .ino file for Fener's Arduino Functions

#include "consts.h"

// Prints variables on Serial Monitor
void debug_printer() {

    Serial.print(steer_pid_input); Serial.print("  ");
    Serial.print(steer_pid_output); Serial.print("  ");
    Serial.print(steer_pid_setpoing); Serial.print("  ");
    
  Serial.println(" ");
}


int second_steering_angle(int Degree) {
    /*
    Calculates other wheels steering angle then returns
    */
  if (Degree < 25 && Degree > -25) return Degree;

  int sign = 0;
  if (Degree < 0) {
    sign = 1;
    Degree = -Degree;
  }

  double Degree_real = Degree / 10.0;
  Degree_real = Degree_real * 3.141592 / 180;

  double second_degree = atan(1 / ((FrontRearMm * tan(3.141592 / 2 - Degree_real) + FrontWheelMm) / FrontRearMm));
  second_degree = second_degree * 180 / 3.141592;

  second_degree = second_degree * 10;
  if (sign) second_degree = -second_degree;

  return int(second_degree);
}

void execute_steering(int degree_one, int degree_two) {

  if (degree_one < 0) {
    degree_one = map(degree_one, 300, -250, R_L, R_R); //sağ teker in sola dönme açısı, sağa dönme açısı
    degree_two = map(degree_two, 256, -291, L_L, L_R); //sol teker in sola dönme açısı, sağa dönme açısı
    degree_one = constrain(degree_one, R_R, R_L);
    degree_two = constrain(degree_two, L_R, L_L);
    servo_driver.setPWM(RIGHT_SERVO_PIN, degree_one, 4096 - degree_one);
    servo_driver.setPWM(LEFT_SERVO_PIN, degree_two, 4096 - degree_two);
  }

  else {
    degree_one = map(degree_one, 256, -291, L_L, L_R);
    degree_two = map(degree_two, 300, -250, R_L, R_R);
    degree_one = constrain(degree_one, L_R, L_L);
    degree_two = constrain(degree_two, R_R, R_L);
    servo_driver.setPWM(RIGHT_SERVO_PIN, degree_two, 4096 - degree_two);
    servo_driver.setPWM(LEFT_SERVO_PIN, degree_one, 4096 - degree_one);
  }
}

struct speed_values read_remote_values() {
  unsigned long rc_ch1 = pulseIn(RC_CHANNEL_ONE, HIGH, 25000);
  unsigned long rc_ch2 = pulseIn(RC_CHANNEL_TWO, HIGH, 25000);

  steering_remote_array[steering_remote_array_counter] = rc_ch2;
  steering_remote_array_counter = (steering_remote_array_counter + 1) % 6;
  double remote_angle = (steering_remote_array[0] + steering_remote_array[1] + steering_remote_array[2] + steering_remote_array[3] + steering_remote_array[4] + steering_remote_array[5]) / 6;

  //TODO Normalize velocity & angle results
  double remote_velocity = rc_ch1 = (rc_ch1 - 1100) / 600.0; 
  remote_angle = (1470 - remote_angle) / 5.0;
  remote_angle = constrain(remote_angle, -200, 200);
  remote_angle = remote_angle + command_array[2];

  speed_values speed_req = {remote_velocity, remote_angle};
  return speed_req;
}

void update_execute_pid() {
  int lr_encoder_diff = lr_encoder_counter - lr_encoder_old_counter;
  int rr_encoder_diff = rr_encoder_counter - rr_encoder_old_counter;
  lr_encoder_old_counter = lr_encoder_counter;
  rr_encoder_old_counter = rr_encoder_counter;
  
  lr_pid_input = WheelPerimeter * lr_encoder_diff / (millis() - time_) / PulsePerRotate;
  rr_pid_input = WheelPerimeter * rr_encoder_diff / (millis() - time_) / PulsePerRotate;
  steer_pid_input = double(command_array[2]);
  
  PID_left_motor.Compute();
  PID_right_motor.Compute();
  PID_steering.Compute();
  
  time_ = millis();

  execute_steering(steer_pid_output, second_steering_angle(steer_pid_output));

  if (lr_pid_setpoint == 0) left_motor.writeMicroseconds(1000);
  else left_motor.writeMicroseconds(lr_pid_output);
  if (rr_pid_setpoing == 0) right_motor.writeMicroseconds(1000);
  else right_motor.writeMicroseconds(rr_pid_output);
  
}

int update_command_array() {
  if (Serial.available() == 0) return 0;
  char buf[8];
  unsigned int* python_input = (int)buf;
  int output_value, output_command;
  
  Serial.readBytes(buf, 8);

  for (int counter = 0; counter < 4; counter++) {
    output_command = python_input[counter] >> 13;
    output_value = python_input[counter] & 4095;
  
    if (output_command == 2) {
      int old_heading = ((command_array[2] % 360) + 360) % 360;
      if ((output_value - old_heading) > 300) bno_turn += -1;
      else if ((output_value - old_heading) < -300) bno_turn += 1; 
      output_value = output_value + bno_turn * 360;
    }
    

  command_array[output_command] = output_value;
  }
  char python_msg[] = {(lf_encoder_counter%255) + 1, (rf_encoder_counter%255) + 1, (lr_encoder_counter%255) + 1, (rr_encoder_counter%255) + 1};
  Serial.write(python_msg, 4);
  debug_printer();
  return 1;
}

void update_speed_setpoints(speed_values speed_req) {
  lr_pid_setpoint = speed_req.linear * PulsePerRotate / (WheelPerimeter / 1000.0);
  rr_pid_setpoing = speed_req.linear* PulsePerRotate / (WheelPerimeter / 1000.0);
  steer_pid_setpoing = speed_req.angular;
}
