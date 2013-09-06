#include "hitechnic-gyro.h"
#define mux_button1 0x01
#define mux_button2 0x02
#define mux_button3 0x04 /////ELEVATOR
#define mux_button4 0x08 /////ARM
int positionSet = 0;
int IR = 0;
//TELE-OP FUNCTIONS

void driveForward(){
	motor[DRIVEFL]  = 100;
	motor[DRIVEFR]  = 100;
	motor[DRIVEBL]  = 100;
	motor[DRIVEBR]  = 100;
}

void driveBack(){
	motor[DRIVEFL]  = -100;
	motor[DRIVEFR]  = -100;
	motor[DRIVEBL]  = -100;
	motor[DRIVEBR]  = -100;
}

void driveRight(){
	motor[DRIVEFL]  = 100;
	motor[DRIVEFR]  = -100;
	motor[DRIVEBL]  = 100;
	motor[DRIVEBR]  = -100;
}

void driveLeft(){
	motor[DRIVEFL]  = -100;
	motor[DRIVEFR]  = 100;
	motor[DRIVEBL]  = -100;
	motor[DRIVEBR]  = 100;
}

void driveForwardSuperSlow(){
	motor[DRIVEFL] = 20;
	motor[DRIVEFR] = 20;
	motor[DRIVEBL] = 20;
	motor[DRIVEBR] = 20;
}

void driveBackSuperSlow(){
	motor[DRIVEFL] = -20;
	motor[DRIVEFR] = -20;
	motor[DRIVEBL] = -20;
	motor[DRIVEBR] = -20;
}

void driveRightSuperSlow(){
	motor[DRIVEFL] = 40;
	motor[DRIVEFR] = -40;
	motor[DRIVEBL] = 40;
	motor[DRIVEBR] = -40;
}

void driveLeftSuperSlow(){
	motor[DRIVEFL] = -40;
	motor[DRIVEFR] = 40;
	motor[DRIVEBL] = -40;
	motor[DRIVEBR] = 40;
}

void stopM(){
	motor[DRIVEFL]  = 0;
	motor[DRIVEFR]  = 0;
	motor[DRIVEBL]  = 0;
	motor[DRIVEBR]  = 0;
}

void elevatorUp(){
	motor[ELEVATOR] = 100;
}

void elevatorDown(){
	motor[ELEVATOR] = -100;
}

void stopE(){
	motor[ELEVATOR] = 0;
}

void stopA(){
	motor[ARM] = 0;
}

void armUp(){
	motor[ARM] = 100;
}

void armDown(){
	motor[ARM] = -100;
}

void armUpSlow(){
	motor[ARM] = 40;
}

void armDownSlow(){
	motor[ARM] = -40;
}

void openHatch(){
	servo[ring_servo] = 0;
}

void closeHatch(){
	servo[ring_servo] = 255;
}

//END TELE-OP FUNCTIONS

//AUTONOMOUS FUNCTIONS

//##########################################################################################################################
void encodeForward(int x, int z){ //Uses encoders to accurately drive forward. Integer x in this situation is the desired distance to travel in tics
	nMotorEncoder[DRIVEFR] = 0;//Setting the value initially to 0
	time1[T4] = 0;
	while (x - nMotorEncoder[DRIVEFR] > 800){//If its over 600 tics away from where it needs to go, motors move fast to get there quickly. Distance of 500 ticks
		if(time1[T4] > z){
			while(true){
				stopM();
			}
		}
		motor[DRIVEFL] = 100;
		motor[DRIVEFR] = 100;
		motor[DRIVEBL] = 100;
		motor[DRIVEBR] = 100;
	}
	while ((x - nMotorEncoder[DRIVEFR] <= 800) && (x - nMotorEncoder[DRIVEFR] > 0)){//When its within 600 tics, motors move more slowly to ensure that there momentum doesn't carry it past destination
		if(time1[T4] > z){
			while(true){
				stopM();
			}
		}
		motor[DRIVEFL] = 15;
		motor[DRIVEFR] = 15;
		motor[DRIVEBL] = 15;
		motor[DRIVEBR] = 15;
	}
	motor[DRIVEFL] = 0;//Once it reaches desired ditance, stop motors
	motor[DRIVEFR] = 0;
	motor[DRIVEBL] = 0;
	motor[DRIVEBR] = 0;
	nxtDisplayCenteredTextLine(3, "%d, %d", nMotorEncoder[DRIVEFR]);//Display current encoder values
	wait10Msec(50);//Pause (for testing currently)
}
//##########################################################################################################################
void simpleEncodeForward(int x, int y, int z){ //Uses encoders to accurately drive forward. Integer x in this situation is the desired distance to travel in tics
	nMotorEncoder[DRIVEFR] = 0;//Setting the value initially to 0
	time1[T4] = 0;
	while (x > nMotorEncoder[DRIVEFR]){
		if(time1[T4] >= z){
			while(true){
				stopM();
			}
		}
		motor[DRIVEFL] = y;
		motor[DRIVEFR] = y;
		motor[DRIVEBL] = y;
		motor[DRIVEBR] = y;
	}
	motor[DRIVEFL] = 0;//Once it reaches desired ditance, stop motors
	motor[DRIVEFR] = 0;
	motor[DRIVEBL] = 0;
	motor[DRIVEBR] = 0;
	nxtDisplayCenteredTextLine(3, "%d, %d", nMotorEncoder[DRIVEFR]);//Display current encoder values
	wait10Msec(50);//Pause (for testing currently)
}
//##########################################################################################################################
//##########################################################################################################################
void simpleEncodeBackward(int x, int y, int z){ //Uses encoders to accurately drive backward. Integer x in this situation is the desired distance to travel in tics integer y is the speed of the motors
	nMotorEncoder[DRIVEFR] = 0;//Setting the value initially to 0
	time1[T4] = 0;
	while ((-x) < nMotorEncoder[DRIVEFR]){//If its over 600 tics away from where it needs to go, motors move fast to get there quickly. Distance of 500 ticks
		if(time1[T4] >= z){
			while(true){
				stopM();
			}
		}
		motor[DRIVEFL] = -y;
		motor[DRIVEFR] = -y;
		motor[DRIVEBL] = -y;
		motor[DRIVEBR] = -y;
	}
	motor[DRIVEFL] = 0;//Once it reaches desired ditance, stop motors
	motor[DRIVEFR] = 0;
	motor[DRIVEBL] = 0;
	motor[DRIVEBR] = 0;
	nxtDisplayCenteredTextLine(3, "%d, %d", nMotorEncoder[DRIVEFR]);//Display current encoder values
	wait10Msec(50);//Pause (for testing currently)
}
//##########################################################################################################################
//##########################################################################################################################
void gyroRightSafely(int x, int z){ //Uses gyro to accurately rotate. Integer x in this situation is the desired degree masurement to rotate
	float rotSpeed = 0;//Zero-ing roating speed. To be given a value soon
	float heading = 0;//Zero-ing heading.
	HTGYROstartCal(gyro);//Calibrate gyro
	time1[T1] = 0;//Clear + start degree calculation timer
	time1[T4] = 0;
	while(heading < (x-11)){//While the robot is more than 9 degrees away from desired position, rotate at 100% speed
		if(time1[T4] >= z){
			while(true){
				stopM();
			}
		}
		motor[DRIVEFL] = 100;
		motor[DRIVEFR] = -100;
		motor[DRIVEBL] = 100;
		motor[DRIVEBR] = -100;
		if(time1[T1] >= 20){//Allow very small time to pass (dt)
			time1[T1]=0;//Clear + start degree calculation timer
   		rotSpeed = HTGYROreadRot(gyro);//Take current rotatoinal speed
 	  	heading += rotSpeed * 0.02;//Multiply by time in order to accurately find degrees rotated during that short interval of time (dt * speed)
    }
 	}
 	time1[T1] = 0;//Clear + start degree calculation timer
 	while(heading < x){//While between 0 and 9 degrees away from desired degree position, rotate at a slow speed towards desired rotational position
 		if(time1[T4] >= z){
			while(true){
				stopM();
			}
		}
 		motor[DRIVEFL] = 30;
		motor[DRIVEFR] = -30;
		motor[DRIVEBL] = 30;
		motor[DRIVEBR] = -30;
		if(time1[T1] >= 20){//Allow very small time to pass (dt)
			time1[T1]=0;//Clear + start degree calculation timer
   		rotSpeed = HTGYROreadRot(gyro);//Take current rotational speed
 	  	heading += rotSpeed * 0.02;//Multiply by time in order to accurately find degrees rotated during that short interval of time (dt * speed)
 		}
	}
	time1[T1] = 0;//Clear + start degree calculation timer
	time10[T2] = 0;//Clear + start saftey timer
	while(heading > x){
		if(time1[T4] >= z){
			while(true){
				stopM();
			}
		}
		if(time10[T2] > 150){//If its been 1.5 seconds and its not at the value, turn at higher speeds
			motor[DRIVEFL] = -40;
			motor[DRIVEFR] = 40;
			motor[DRIVEBL] = -40;
			motor[DRIVEBR] = 40;
	  }
		else{//If its been 1.5 seconds and its not at the value, go at high speeds
			motor[DRIVEFL] = -32;
			motor[DRIVEFR] = 32;
			motor[DRIVEBL] = -32;
			motor[DRIVEBR] = 32;
		}
		if(time1[T1] >= 20){//Allow very small time to pass (dt)
			time1[T1]=0;//Clear + start degree calculation timer
   		rotSpeed = HTGYROreadRot(gyro);//Take current rotational speed
 	  	heading += rotSpeed * 0.02;//Multiply by time in order to accurately find degrees rotated during that short interval of time (dt * speed)
 		}
 	}
 	motor[DRIVEFL] = 0;//You're close to the position! Yay! Stop motors
	motor[DRIVEFR] = 0;
	motor[DRIVEBL] = 0;
	motor[DRIVEBR] = 0;
 	nxtDisplayCenteredTextLine(3, "%d, %d", heading);//Display current rotational position
	wait10Msec(50);//Wait time (for testing purposes)
}
//##########################################################################################################################
//##########################################################################################################################
void gyroLeftSafely(int x, int z){ //Uses gyro to accurately rotate. Integer x in this situation is the desired degree masurement to rotate
	float rotSpeed = 0;//Zero-ing roating speed. To be given a value soon
	float heading = 0;//Zero-ing heading. To use as 0 in order to slow down as turning approaches desired value
	HTGYROstartCal(gyro);//Calibrate gyro
	time1[T1] = 0;//Clear + start degree calculation timer
	time1[T4] = 0;
	while(heading > ((-x)+11) ){//While the robot is more than 9 degrees away from desired position, rotate at 100% speed
		if(time1[T4] >= z){
			while(true){
				stopM();
			}
		}
		motor[DRIVEFL] = -100;
		motor[DRIVEFR] = 100;
		motor[DRIVEBL] = -100;
		motor[DRIVEBR] = 100;
		if(time1[T1] >= 20){//Allow very small time to pass (dt)
			time1[T1]= 0;//Clear + start degree calculation timer
   		rotSpeed = HTGYROreadRot(gyro);//Take current rotatoinal speed
 	  	heading += rotSpeed * 0.02;//Multiply by time in order to accurately find degrees rotated during that short interval of time (dt * speed)
    }
 	}
 	time1[T1] = 0;//Clear + start degree calculation timer
 	while(heading > (-x)){//While between 0 and 8 degrees away from desired degree position, rotate at a slow speed towards desired rotational position
 		if(time1[T4] >= z){
			while(true){
				stopM();
			}
		}
 		motor[DRIVEFL] = -30;
		motor[DRIVEFR] = 30;
		motor[DRIVEBL] = -30;
		motor[DRIVEBR] = 30;
		if(time1[T1] >= 20){//Allow very small time to pass (dt)
			time1[T1]=0;//Clear + start degree calculation timer
   		rotSpeed = HTGYROreadRot(gyro);//Take current rotational speed
 	  	heading += rotSpeed * 0.02;//Multiply by time in order to accurately find degrees rotated during that short interval of time (dt * speed)
 		}
	}
  time10[T2] = 0;//Clear + start timer saftey timer
	time1[T1] = 0;//Clear + start degree calculation timer
	while(heading < (-x)){//If desired degree position is overshot, rotate at a slow speed towards desired position
		if(time1[T4] >= z){
			while(true){
				stopM();
			}
		}
		if(time10[T2] > 150){//If its been 1.5 seconds and its not at the value, go at high speeds
			motor[DRIVEFL] = 37;
			motor[DRIVEFR] = -37;
			motor[DRIVEBL] = 37;
			motor[DRIVEBR] = -37;
		}
		else{//If its been 1.5 seconds and its not at the value, go at high speeds
			motor[DRIVEFL] = 32;
			motor[DRIVEFR] = -32;
			motor[DRIVEBL] = 32;
			motor[DRIVEBR] = -32;
		}
		if(time1[T1] >= 20){//Allow very small time to pass (dt)
			time1[T1]=0;//Clear + start degree calculation timer
   		rotSpeed = HTGYROreadRot(gyro);//Take current rotational speed
 	  	heading += rotSpeed * 0.02;//Multiply by time in order to accurately find degrees rotated during that short interval of time (dt * speed)
 		}
 	}
 	motor[DRIVEFL] = 0;//You're (close to) the position! Yay! Stop motors
	motor[DRIVEFR] = 0;
	motor[DRIVEBL] = 0;
	motor[DRIVEBR] = 0;
 	nxtDisplayCenteredTextLine(3, "%d, %d", heading);//Display current rotational position
	wait10Msec(50);//Wait time (for testing purposes)
}
//##########################################################################################################################
//##########################################################################################################################
void resetAE(){

	bool elevatorSet = false;
	bool armSet = false;

	servo[grab_servoL] = 0;
	servo[grab_servoR] = 255;

	while(true){
		if(SensorValue(TOUCHMUX) & mux_button3){
			stopE();
			elevatorSet = true;
		}
		else{
			elevatorDown();
		}
		if(SensorValue(TOUCHMUX) & mux_button4){
			stopA();
			armSet = true;
		}
		else{
			armDown();
		}
		if(elevatorSet == true && armSet == true){
			break;
		}
	}
}

//##########################################################################################################################
//##########################################################################################################################
void autoScoreMiddle(){
		int initialElevatorPosition = nMotorEncoder[ELEVATOR];
		int	initialArmPosition = nMotorEncoder[ARM];
		bool elevatorSet = false;
		bool armSet = false;
		positionSet = 1;
		while(positionSet == 1){
			if(nMotorEncoder[ELEVATOR] > -4440 && initialElevatorPosition > -4440){ //NUMBER for middle post height
				elevatorUp();
			}
			else if(nMotorEncoder[ELEVATOR] < -4440 && initialElevatorPosition < -4440){ //NUMBER for middle post height
				elevatorDown();
			}
			else{
				stopE();
				elevatorSet = true; //on target
			}
			servo[grab_servoL] = 125;  //NUMBER
			servo[grab_servoR] = 130;  //NUMBER
			if(nMotorEncoder[ARM] < -2900 && initialArmPosition < -2900){ //NUMBER for arm scoring angle
				armDown();
			}
			else if(nMotorEncoder[ARM] > -2900 && initialArmPosition > -2900){ //NUMBER for arm scoring angle
				armUp();
			}
			else{
				stopA();
				armSet = true; //on target
			}
			if(elevatorSet == true && armSet == true){
				servo[grab_servoL] = 230;  //NUMBER
				servo[grab_servoR] = 25;  //NUMBER
				positionSet = 0;
			}
		}
		wait10Msec(100);
		servo[ring_servo] = 0;
		simpleEncodeForward(1800, 35, 4000); //NUMBER
		initialElevatorPosition = nMotorEncoder[ELEVATOR];
		while(nMotorEncoder[ELEVATOR] < -1160  && initialElevatorPosition < -1160){
			elevatorDown();
		}
		stopE();
		simpleEncodeBackward(2600, 50, 4000); //NUMBER
		resetAE();
	}
//##########################################################################################################################
//##########################################################################################################################

void autoScoreLeft(){
	bool elevatorSet = false;
	bool armSet = false;
	simpleEncodeBackward(1800, 25, 4000); //NUMBER
	gyroLeftSafely(46, 4000); //NUMBER
	encodeForward(3050, 4000); //NUMBER
	gyroRightSafely(45, 4000); //NUMBER
	int initialElevatorPosition = nMotorEncoder[ELEVATOR];
	int	initialArmPosition = nMotorEncoder[ARM];
	positionSet = 2;
	while(positionSet == 2){
		if(nMotorEncoder[ELEVATOR] < -500 && initialElevatorPosition < -500){ //NUMBER for bottom post height
			elevatorDown();
		}
		else if(nMotorEncoder[ELEVATOR] > -500 && initialElevatorPosition > -500){ //NUMBER for bottom post height
			elevatorUp();
		}
		else{
			stopE();
			elevatorSet = true; //on target
		}
		if(nMotorEncoder[ARM] < -2100 && initialArmPosition < -2100){ //NUMBER for arm scoring angle
			armDown();
		}
		else if(nMotorEncoder[ARM] > -2100 && initialArmPosition > -2100){ //NUMBER for arm scoring angle
			armUp();
		}
		else{
			stopA();
			armSet = true; //on target
		}
		if(elevatorSet == true && armSet == true){
			servo[grab_servoL] = 187;  //NUMBER
			servo[grab_servoR] = 68;  //NUMBER
			positionSet = 0;
		}
	}
	wait10Msec(200);
	openHatch();
	simpleEncodeForward(1400, 30, 4000); //NUMBER
	initialArmPosition = nMotorEncoder[ARM];
	while(nMotorEncoder[ARM] < -1150 && initialArmPosition < -1150){
		armDown();
	}
	stopA();
	closeHatch();
	servo[grab_servoL] = 0;  //NUMBER
	servo[grab_servoR] = 255;  //NUMBER
	simpleEncodeBackward(800, 40, 4000); //NUMBER
	resetAE();
}
//##########################################################################################################################
//##########################################################################################################################

void autoScoreRight(){
	bool elevatorSet = false;
	bool armSet = false;
	simpleEncodeBackward(1800, 25, 4000); //NUMBER
	gyroRightSafely(48.8, 4000); //NUMBER
	encodeForward(3000, 4000); //NUMBER
	gyroLeftSafely(49, 4000); //NUMBER
	int initialElevatorPosition = nMotorEncoder[ELEVATOR];
	int	initialArmPosition = nMotorEncoder[ARM];
	positionSet = 2;
	while(positionSet == 2){
		if(nMotorEncoder[ELEVATOR] < -625 && initialElevatorPosition < -625){ //NUMBER for bottom post height
			elevatorDown();
		}
		else if(nMotorEncoder[ELEVATOR] > -625 && initialElevatorPosition > -625){ //NUMBER for bottom post height
			elevatorUp();
		}
		else{
			stopE();
			elevatorSet = true; //on target
		}
		if(nMotorEncoder[ARM] < -2100 && initialArmPosition < -2100){ //NUMBER for arm scoring angle
			armDown();
		}
		else if(nMotorEncoder[ARM] > -2100 && initialArmPosition > -2100){ //NUMBER for arm scoring angle
			armUp();
		}
		else{
			stopA();
			armSet = true; //on target
		}
		if(elevatorSet == true && armSet == true){
			servo[grab_servoL] = 187;  //NUMBER
			servo[grab_servoR] = 68;  //NUMBER
			positionSet = 0;
		}
	}
	wait10Msec(100);
	openHatch();
	simpleEncodeForward(1450, 30, 4000); //NUMBER
	initialArmPosition = nMotorEncoder[ARM];
	while(nMotorEncoder[ARM] < -1300 && initialArmPosition < -1300){
		armDown();
	}
	stopA();
	closeHatch();
	servo[grab_servoL] = 0;  //NUMBER
	servo[grab_servoR] = 255;  //NUMBER
	simpleEncodeBackward(800, 40, 4000); //NUMBER
	resetAE();
}

//###################################################################################################
void setupAuto(){
	positionSet = -1;
	encodeForward(2890, 4000); //NUMBER
	wait10Msec(50);
	gyroRightSafely(48.9, 4000); //NUMBER
	wait10Msec(50);
	while(nMotorEncoder[ELEVATOR] > -7000){
		elevatorUp();
	}
	stopE();
	simpleEncodeForward(1600, 30, 4000); //NUMBER
	IR = SensorValue[IRSEEK];
	nxtDisplayCenteredBigTextLine(3, "%d, %d",	IR);
	wait10Msec(100);
	while(nMotorEncoder[ELEVATOR] < -3900){
		elevatorDown();
	}
	stopE();
}
//END AUTONOMOUS FUNCTIONS
