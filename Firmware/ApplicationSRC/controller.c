#include "controller.h"
#include "StateFormulas.h"
#include <stdbool.h>
#include <math.h>
#include "main.h"
//#include <libpic30.h>
#include "CAN.h"
#include "EPOS4.h"
#include "gpio_functions.h"



/*#define torque_const_kv100 0.095
#define gear_ratio_chain_drive 40*/

#define max_CST_CMD_EPOS 1200 //change here for 19 A =2375

#define torque_const_kv100lite 0.095 //Nm/A
#define gear_ratio_Chain_knee 40
#define peak_current 8   //A actually nominal current max current 19A


/*Switching parameter*/
#define ES_PSW_switch_heel_pressure 2290


#define ES_SWF_switch_heel_pressure 1950
#define ES_SWF_switch_toe_pressure 1930


//#define Toe_pressure_ES_SWF
#define SWF_SWE_switching_angle 38

#define PSW_SWF_switching_angle 20

#define SWE_I_switching_angle 10


#define IMU_sensor_Heel_Strike 7000 //previous 4200 1g
#define Loadcell_Heel_Strike 3810//1920 // heel pressure Test2_mV reading

// Greg start
#define ES_equilibrium -5   // position in degree
#define PSW_equilibrium -20   //-6 position in degree
#define SWF_equilibrium -45     // earlier  position in degree
#define SWE_equilibrium -5      // earlier  position in degree
#define IDLE_equilibrium -3     // position in degree
// Greg end

/*State 0:  Early Stance Parameter*/
#define ES_stiffness 2.5    // 0.35 Previous Eq.=  Present Eq.=
#define ES_damping 0.05


/*State 1:  PRE_SWING Parameter*/
#define PSW_stiffness 0          //max 0.60
#define PSW_damping 0

/*State 2: SW_FLEXION Parameter*/
#define SWF_stiffness .45     // Max 1.30
#define SWF_damping 0.02

/*State 3:  SW_EXTENSION Parameter*/
#define SWE_stiffness .35
#define SWE_damping 0.02  //0.001

/*State 4:  IDLE/Late swing Parameter*/
#define IDLE_stiffness .5//.5
#define IDLE_damping 0//0.01

// Greg start comment
//enum states{
//    ST_EARLY_STANCE,
//    ST_PRE_SWING,
//    ST_SW_FLEXION,
//    ST_SW_EXTENSION,
//    IDLE,
//};
//enum states state = IDLE;//ST_EARLY_STANCE;
// Greg end comment

//enum states{
//    ST_EARLY_STANCE,
//    ST_MIDDLE_STANCE,
//    ST_LATE_STANCE,
//    ST_EARLY_SWING,
//    ST_LATE_SWING,
//};
//enum states state=ST_EARLY_STANCE;


double impedance = 0, d_torque=0,n_torque=0;
double desired_force = 0.0, desired_current = 0;
float percent = 0,Percent=0,percent_new = 0,percent_old = 0;  // 0.5 percent means 50% duty cycle
int CST_CMD_EPOS_contrl=0;
int int_CST_CMD_EPOS=0;
int count_time=0;

bool flag_begin_SW = false, flag_begin_ES = true;

struct st_impedance my_st_impedance;

// Greg start
enum states{
    STANCE,
	SWING
};
enum states state = SWING;

#define ST_damping      0.00     // (Vanderbilt = 0 (N/m) / (deg/s))
#define SW_damping      0.05     // 0.05 used to get zero overshoot and 0.5 sec settling time (Vanderbilt = 0 (N/m) / (deg/s))

#define ST_stiffness    2.50     // 2.50 used to keep heat down in EPOS (Vanderbilt = 4.97 (N/m) / deg)
#define SW_stiffness    0.45     // 0.45 on the bench "feels" right (Vanderbilt = 0.65 (N/m) / deg)

#define ST_equilibrium -4.99     // (Vanderbilt = -4.99 deg)
#define SW_equilibrium -35.0     // (Vanderbilt = -35.0 deg)
// Greg end

//struct st_impedance controller_impedance(float angle, float knee_velocity,int ac_x, float current)

struct st_impedance controller_impedance(float angle, float knee_velocity,float Heel_pressure, float Toe_pressure,float IMU_acceleration,float hip_joint_angle)
{

    switch (state)
    {
	// Greg start
    case STANCE:
    	if(0) {
    		 state = SWING;
    	}
    	// Compute knee torque
    	d_torque = Impedance(angle, knee_velocity, ST_stiffness, ST_damping, ST_equilibrium); // earlier .7 Impedance(Angle,Velocity,K1,B,Theta_E)//1, 0.001 //previous equi angle=-10

    	// Compute motor torque
    	CST_CMD_EPOS_contrl=(d_torque/(torque_const_kv100lite*gear_ratio_Chain_knee*peak_current))*1000;

    	// Set motor torque limits
		if (CST_CMD_EPOS_contrl>=max_CST_CMD_EPOS) {
			CST_CMD_EPOS_contrl=max_CST_CMD_EPOS;
		}
		else if (CST_CMD_EPOS_contrl<-max_CST_CMD_EPOS) {
			CST_CMD_EPOS_contrl=-max_CST_CMD_EPOS;
		}

		// Send motor torque command
		EPOS4_CST_apply_torque(0x601,CST_CMD_EPOS_contrl); //100 means 10% +ve is extension -ve is flexion

		break;

    case SWING:
        	if(0) {
        		 state = STANCE;
        	}
        	// Compute knee torque
        	d_torque = Impedance(angle, knee_velocity, SW_stiffness, SW_damping, SW_equilibrium); // earlier .7 Impedance(Angle,Velocity,K1,B,Theta_E)//1, 0.001 //previous equi angle=-10

        	// Compute motor torque
        	CST_CMD_EPOS_contrl=(d_torque/(torque_const_kv100lite*gear_ratio_Chain_knee*peak_current))*1000;

        	// Set knee torque limits
    		if (CST_CMD_EPOS_contrl>=max_CST_CMD_EPOS)
    		{
    			CST_CMD_EPOS_contrl=max_CST_CMD_EPOS;
    		}

    		else if (CST_CMD_EPOS_contrl<-max_CST_CMD_EPOS)
    		{
    			CST_CMD_EPOS_contrl=-max_CST_CMD_EPOS;
    		}

    		// Send motor torque command
    		EPOS4_CST_apply_torque(0x601,CST_CMD_EPOS_contrl); //100 means 10% +ve is extension -ve is flexion

    		break;
    // Greg end

    // Greg start comment
//    // state 0
//    case ST_EARLY_STANCE:
//
//    	YELLOW_LED();
//    	RED_LED_OFF();
//
//        if (count_time>=300 && hip_joint_angle>=0) // && Toe_pressure<ES_SWF_switch_toe_pressure && && hip_joint_angle>=-3
//
////            if (Heel_pressure<=ES_SWF_switch_toe_pressure && hip_joint_angle>=-3 && count_time>=100) // && Toe_pressure<ES_SWF_switch_toe_pressure
////        if (hip_joint_angle>=10) // && Toe_pressure<ES_SWF_switch_toe_pressure
//        {
//           state = ST_SW_FLEXION;
//           count_time=0;
//           break;
//         }
//        count_time++;
//
//        //CONTROL ACTION  K1 = 2
//       d_torque = Impedance(angle, knee_velocity, ES_stiffness, ES_damping, ES_equilibrium); // earlier .7 Impedance(Angle,Velocity,K1,B,Theta_E)//1, 0.001 //previous equi angle=-10
//
//
//       CST_CMD_EPOS_contrl=(d_torque/(torque_const_kv100lite*gear_ratio_Chain_knee*peak_current))*1000;
//
//
//	  		if (CST_CMD_EPOS_contrl>=max_CST_CMD_EPOS)
//	  		{
//	  			CST_CMD_EPOS_contrl=max_CST_CMD_EPOS;
//	  		}
//
//	  		else if (CST_CMD_EPOS_contrl<-max_CST_CMD_EPOS)
//	  		{
//	  			CST_CMD_EPOS_contrl=-max_CST_CMD_EPOS;
//	  		}
//
//	  		EPOS4_CST_apply_torque(0x601,CST_CMD_EPOS_contrl); //100 means 10% +ve is extension -ve is flextion
//
//        /*
//        n_torque= -current*torque_const;
//        percent=PID_torque_control(d_torque,n_torque,0.030, 0.001); //PID_torque_control(double torque_desired,double torque_now, double Kp=.015, double Kd=0.006)
//        */
//        break;
//
//    //state 1
//    case ST_PRE_SWING:
//
//        if (angle > PSW_SWF_switching_angle)
//
//        {
////           state = ST_SW_FLEXION;
////           break;
//       }
//    	YELLOW_LED_OFF();
//    	RED_LED();
//        //CONTROL ACTION 1.9
//        d_torque = Impedance(angle, knee_velocity, PSW_stiffness, PSW_damping, PSW_equilibrium); // earlier .7 Impedance(Angle,Velocity,K1,B,Theta_E)//1, 0.001 //previous equi angle=-10
//
//
//        CST_CMD_EPOS_contrl=(d_torque/(torque_const_kv100lite*gear_ratio_Chain_knee*peak_current))*1000;
//
//
// 	  		if (CST_CMD_EPOS_contrl>=max_CST_CMD_EPOS)
// 	  		{
// 	  			CST_CMD_EPOS_contrl=max_CST_CMD_EPOS;
// 	  		}
//
// 	  		else if (CST_CMD_EPOS_contrl<-max_CST_CMD_EPOS)
// 	  		{
// 	  			CST_CMD_EPOS_contrl=-max_CST_CMD_EPOS;
// 	  		}
//
// 	  		EPOS4_CST_apply_torque(0x601,CST_CMD_EPOS_contrl); //100 means 10% +ve is extension -ve is flextion
//
//
//        break;
//
//    //state 2
//    case ST_SW_FLEXION:
//
//        if (angle>=SWF_SWE_switching_angle) // earlier angle <-18
//
//        {
//            state = ST_SW_EXTENSION;
//
//
//            break;
//        }
//    	YELLOW_LED();
//    	RED_LED();
//
//        d_torque = Impedance(angle, knee_velocity, SWF_stiffness, SWF_damping, SWF_equilibrium); // earlier .7 Impedance(Angle,Velocity,K1,B,Theta_E)//1, 0.001 //previous equi angle=-10
//
//
//        CST_CMD_EPOS_contrl=(d_torque/(torque_const_kv100lite*gear_ratio_Chain_knee*peak_current))*1000;
//
//
// 	  		if (CST_CMD_EPOS_contrl>=max_CST_CMD_EPOS)
// 	  		{
// 	  			CST_CMD_EPOS_contrl=max_CST_CMD_EPOS;
// 	  		}
//
// 	  		else if (CST_CMD_EPOS_contrl<-max_CST_CMD_EPOS)
// 	  		{
// 	  			CST_CMD_EPOS_contrl=-max_CST_CMD_EPOS;
// 	  		}
//
// 	  		EPOS4_CST_apply_torque(0x601,CST_CMD_EPOS_contrl); //100 means 10% +ve is extension -ve is flextion
//
//
//
//
//        break;
//
//      //state 3
//      case ST_SW_EXTENSION:
////        if ((angle>=ESW_LSW_switching_angle)&&(fabsf(knee_velocity)<= 5))  // knee_velocity < -5: did not transit, knee_velocity < -3: transit
//
//        if (angle<=SWE_I_switching_angle)  // knee_velocity < -5: did not transit, knee_velocity < -3: transit
//
//        {
//           state = IDLE;
//           break;
//        }
//
//        //CONTROL ACTION 1.5
//        d_torque = Impedance(angle, knee_velocity, SWE_stiffness, SWE_damping, SWE_equilibrium); // earlier .7 Impedance(Angle,Velocity,K1,B,Theta_E)//1, 0.001 //previous equi angle=-10
//
//
//        CST_CMD_EPOS_contrl=(d_torque/(torque_const_kv100lite*gear_ratio_Chain_knee*peak_current))*1000;
//
// 	  		if (CST_CMD_EPOS_contrl>=max_CST_CMD_EPOS)
// 	  		{
// 	  			CST_CMD_EPOS_contrl=max_CST_CMD_EPOS;
// 	  		}
//
// 	  		else if (CST_CMD_EPOS_contrl<-max_CST_CMD_EPOS)
// 	  		{
// 	  			CST_CMD_EPOS_contrl=-max_CST_CMD_EPOS;
// 	  		}
//
// 	  		EPOS4_CST_apply_torque(0x601,CST_CMD_EPOS_contrl); //100 means 10% +ve is extension -ve is flextion
//
//
//        break;
//
//      //state 4 //idle
//      case IDLE:
//
//          //if (fabsf(ac_x)>= 32000)  // 32767 is 2g
//	   if (fabsf(IMU_acceleration)>= IMU_sensor_Heel_Strike)
////     if (Heel_pressure+Toe_pressure>= Loadcell_Heel_Strike)
//      {
//              state = ST_EARLY_STANCE;
//
//              break;
//          }
//       YELLOW_LED_OFF();
//       RED_LED_OFF();
//
//       d_torque = Impedance(angle, knee_velocity, IDLE_stiffness, IDLE_damping, IDLE_equilibrium); // earlier .7 Impedance(Angle,Velocity,K1,B,Theta_E)//1, 0.001 //previous equi angle=-10
//
//       CST_CMD_EPOS_contrl=(d_torque/(torque_const_kv100lite*gear_ratio_Chain_knee*peak_current))*1000;
//
//
//	  		if (CST_CMD_EPOS_contrl>=max_CST_CMD_EPOS)
//	  		{
//	  			CST_CMD_EPOS_contrl=max_CST_CMD_EPOS;
//	  		}
//
//	  		else if (CST_CMD_EPOS_contrl<-max_CST_CMD_EPOS)
//	  		{
//	  			CST_CMD_EPOS_contrl=-max_CST_CMD_EPOS;
//	  		}
//
//
//	  		EPOS4_CST_apply_torque(0x601,CST_CMD_EPOS_contrl); //100 means 10% +ve is extension -ve is flextion
//
//        break;
    // Greg end comment
    }

    my_st_impedance.st = state;
    my_st_impedance.desired_torque = d_torque;
    my_st_impedance.CST_CMD_now = CST_CMD_EPOS_contrl;
    return my_st_impedance;
}
