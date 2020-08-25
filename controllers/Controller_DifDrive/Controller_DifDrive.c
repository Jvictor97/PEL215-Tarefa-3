#include <webots/robot.h>
#include <webots/motor.h>
#include <stdio.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

static WbDeviceTag left_motor, right_motor;

double getVR(double VL, double R) {
  const double L = 0.0565;
  // double L = 0.115;
  
  double VR;
  
  VR = VL * (R + L/2) / (R - L/2);

  return VR;
}

double toAngularSpeed(double linearSpeed) {
  const double r = 0.0205;
  return linearSpeed / r;
}

int main(int argc, char **argv)
{
  wb_robot_init();
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  double VR;  // Velocidade da roda direita (VR)
  double VL;  // Velocidade da roda esquerda (VL)
  double R;  // Raio da circunferencia que o rob� deve fazer
  // double d; // metade da dist�ncia do eixo do rob� em metros

  while (wb_robot_step(TIME_STEP) != -1) {
    VL = 0.05;
    R = 0.5;
    VR = getVR(VL, R);
    
    // printf("R: %.2f, VR: %.2f\n", R, getVR(VL,R));
    
    double wl = toAngularSpeed(VL);
    double wr = toAngularSpeed(VR);
    
    printf("wl: %.5f, wr: %.5f\n", wl, wr);
        
    wb_motor_set_velocity(left_motor, wl);
    wb_motor_set_velocity(right_motor, wr);

  };
  wb_robot_cleanup();
  return 0;
}