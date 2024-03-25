#include <stdint.h>        /* Includes uint16_t definition                    */

struct st_impedance
{
//   volatile int  st;
//   volatile float desired_torque;
//   volatile int CST_CMD_now;

   int  st;
   float desired_torque;
   int CST_CMD_now;
};


//int controller(float angle, float ankle_velocity,int16_t ac_x,float current_factor);

//struct st_impedance controller_impedance(float angle, float ankle_velocity,int ac_x, float current);

struct st_impedance controller_impedance(float angle, float ankle_velocity,float Heel_pressure, float Toe_pressure,float IMU_acceleration, float hip_joint_angle);

//double PID_torque_control(double torque_desired,double torque_now, double Kp, double Kd);

//#include <stdint.h>        /* Includes uint16_t definition                    */
//
//struct st_impedance
//{
//   int  st ;
//   float impedance;
//};
//
//
//int controller(float angle, float ankle_velocity,int16_t ac_x,float current_factor);
//struct st_impedance controller_impedance(float angle, float ankle_velocity,int16_t ac_x, float current_factor);
