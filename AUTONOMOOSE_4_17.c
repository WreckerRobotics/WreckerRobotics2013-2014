#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     gyro,           sensorAnalogInactive)
#pragma config(Sensor, S3,     IRSEEK,         sensorHiTechnicIRSeeker1200)
#pragma config(Sensor, S4,     TOUCHMUX,       sensorHiTechnicTouchMux)
#pragma config(Motor,  mtr_S1_C1_1,     ELEVATOR,      tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     ARM,           tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     DRIVEFL,       tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     DRIVEFR,       tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     DRIVEBL,       tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_2,     DRIVEBR,       tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    grab_servoL,          tServoStandard)
#pragma config(Servo,  srvo_S1_C4_2,    grab_servoR,          tServoStandard)
#pragma config(Servo,  srvo_S1_C4_3,    ring_servo,           tServoStandard)
#pragma config(Servo,  srvo_S1_C4_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//WRECKER ROBOTICS AUTONOMOUS PROGRAM || AUTHORS: MAX LIBEN, ALEC SOLDER || DATE: APRIL 5TH, 2013 ||
//DESCRIPTION: Utilizes encoders, a gyro sensor, and an infrared sensor to navigate the field accurately and score an autonomous ring.
//Function definitions can be found in "references4.h". Utilizes an FTC approved gyro driver.



#include "references4.h"

	void initializeRobot()
{
	servo[grab_servoL] = 0;
	servo[grab_servoR] = 255;
	servo[ring_servo] = 255;
	nMotorEncoder[ARM] = 0;
	nMotorEncoder[ELEVATOR] = 0;
  return;
}

task main{
	initializeRobot();
	setupAuto();


	if(IR == 5 || IR == 0){
		autoScoreMiddle();
	}

	if((IR < 5 && IR != 0) || IR == 4 || IR == 3){
		autoScoreLeft();
	}
	if(IR == 6 || IR > 5 || IR == 7){
		autoScoreRight();
	}
wait10Msec(100);
}