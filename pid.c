/*
    This program drives the PID algorithm on the segway by generating the Angle from the sensor and then applying the PID

    The BerryIMUv3 sensor is used.

    Feel free to do whatever you like with this code
*/

#include <sys/time.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include "IMU.c"
#include <pigpio.h>

#define PWM_RANGE 255

//Motor Driver Pins
#define ENA 25
#define IN1 23
#define IN2 24
#define ENB 17
#define IN3 27
#define IN4 22


#define DT 0.02         // [s/loop] loop period. 20ms
#define AA 0.97         // complementary filter constant

#define A_GAIN 0.0573    // [deg/LSB]
#define G_GAIN 0.070     // [deg/s/LSB]
#define RAD_TO_DEG 57.29578
#define M_PI 3.14159265358979323846


void  INThandler(int sig)// Used to do a nice clean exit when Ctrl-C is pressed
{
	signal(sig, SIG_IGN);
	//When ending the program, turn off the motors
	gpioWrite(IN1, 0);
	gpioWrite(IN2, 0);
	gpioWrite(IN3, 0);
	gpioWrite(IN4, 0);
	exit(0);
}

//Used to get the difference in time later
int mymillis()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

int timeval_subtract(struct timeval *result, struct timeval *t2, struct timeval *t1)
{
    long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);
    result->tv_sec = diff / 1000000;
    result->tv_usec = diff % 1000000;
    return (diff<0);
}

//Used to set the motors according to the outcome of the PID and the necessary direction
void setMotor(int dir, float PWMSig){
	gpioPWM(ENA, PWMSig);
	gpioPWM(ENB, PWMSig);
	//Forward
	if(dir == 1) {
		gpioWrite(IN1, 1);
		gpioWrite(IN2, 0);
		gpioWrite(IN3, 1);
		gpioWrite(IN4, 0);
	}
	//Backward
	else{
		gpioWrite(IN1, 0);
		gpioWrite(IN2, 1);
		gpioWrite(IN3, 0);
		gpioWrite(IN4, 1);
	}
}

int main(int argc, char *argv[])
{


	//Setup Functions
	if(gpioInitialise()<0) return 1;
	gpioSetMode(ENA, PI_OUTPUT);
	gpioSetMode(IN1, PI_OUTPUT);
	gpioSetMode(IN2, PI_OUTPUT);
	gpioSetMode(ENB, PI_OUTPUT);
	gpioSetMode(IN3, PI_OUTPUT);
	gpioSetMode(IN4, PI_OUTPUT);
	gpioSetPWMrange(ENA, PWM_RANGE);
	gpioSetPWMrange(ENB, PWM_RANGE);
	
	//PID Values
	float CFangleY = 0.0;
	float error = 0.0;
	float lastError = 0.0;
	float target = 0.0;
	//These are the three constants that you will (painfully) have to tune
	float KP = 85;
	float KI = 3;
	float KD = 0.7;
	float output = 0.0; //The control signal
	float pTerm = 0.0;
	float iTerm = 0.0;
	float dTerm = 0.0;
	float PWMSig = 0.0;
	int dir = 0; //The direction, 1 = forward, 0 = backward
		
	float rate_gyr_y = 0.0;   // [deg/s]
	float rate_gyr_x = 0.0;   // [deg/s]
	float rate_gyr_z = 0.0;   // [deg/s]

	//The values from the various sensors are stored in an array of length 3, one element for each axis
	int  accRaw[3];
	int  magRaw[3]; //Not used but could be incorporated later?
	int  gyrRaw[3];

	float gyroXangle = 0.0;
	float gyroYangle = 0.0;
	float gyroZangle = 0.0;
	float AccYangle = 0.0;
	float AccXangle = 0.0;
	float CFangleX = 0.0;
	
	//Setup the timing
	int startInt  = mymillis();
	struct  timeval tvBegin, tvEnd,tvDiff;

	//Setup the interrupt handler
	signal(SIGINT, INThandler);

	//Setup the IMU
	detectIMU();
	enableIMU();

	gettimeofday(&tvBegin, NULL);

	//MAIN LOOP - I kept it as a for loop for now so eventually it will stop
	for(int i = 0; i<100000; i++)
	{
		startInt = mymillis();

		//read ACC and GYR data
		readACC(accRaw);
		readGYR(gyrRaw);

		//Convert Gyro raw to degrees per second
		rate_gyr_x = (float) gyrRaw[0]  * G_GAIN;
		rate_gyr_y = (float) gyrRaw[1]  * G_GAIN;
		rate_gyr_z = (float) gyrRaw[2]  * G_GAIN;



		//Calculate the angles from the gyro
		gyroXangle+=rate_gyr_x*DT;
		gyroYangle+=rate_gyr_y*DT;
		gyroZangle+=rate_gyr_z*DT;


		//Convert Accelerometer values to degrees
		AccXangle = (float) (atan2(accRaw[1],accRaw[2])+M_PI)*RAD_TO_DEG;
		AccYangle = (float) (atan2(accRaw[2],accRaw[0])+M_PI)*RAD_TO_DEG;

		//Change the rotation value of the accelerometer to -/+ 180 and move the Y axis '0' point to up.
		//Two different pieces of code are used depending on how your IMU is mounted.
		//If IMU is upside down
		/*
			if (AccXangle >180)
					AccXangle -= (float)360.0;

			AccYangle-=90;
			if (AccYangle >180)
					AccYangle -= (float)360.0;
		*/

		//If IMU is up the correct way, use these lines
		AccXangle -= (float)180.0;
		if (AccYangle > 90)
				AccYangle -= (float)270;
		else
			AccYangle += (float)90;


		//Apply the complementary filter
		CFangleY=AA*(CFangleY+rate_gyr_y*DT) +(1 - AA) * AccYangle;

			//Perform PID
			error = CFangleY - target;
			pTerm = KP*error;
			//iTerm = KI*(iTerm + (0.5f*DT*(error+lastError)));
			iTerm += KI*error*DT;
			dTerm = KD*((error-lastError)/DT);
			lastError = error;
			output = pTerm + iTerm + dTerm;
			
			//Clamp the integrator and output using static clamping
			if(iTerm < -256.0) {iTerm = -256.0;}
			if(iTerm > 256.0) {iTerm = 256.0;}
			if(output < -255.0) {output = -255.0;}
			if(output > 255.0) {output = 255.0;}
			
			//if the angle is too extreme don't do anything
			if(CFangleY > 25 || CFangleY < -25) {output = 0;}
			
			//Compute the PWM signal as the floating point absolute value of the control signal, then clamp it
			PWMSig = fabs(output);
			if(PWMSig > PWM_RANGE) {PWMSig = PWM_RANGE;} 
			
			//Compute Direction
			if(CFangleY > 0) {dir = 0; printf("CFangleY = %f PWM Signal = %f, Tilting Back\n", CFangleY, PWMSig);}
			if(CFangleY < 0) {dir = 1; printf("CFangleY = %f PWM Signal = %f, Tilting Forward\n", CFangleY, PWMSig);}
		
		//Set the motor based on the outcome of the PID
		setMotor(dir, PWMSig);
		
		//Each loop should be at least 20ms.
		while(mymillis() - startInt < (DT*1000)){
				usleep(100);
		}
		
		//Print the loop time to make sure that it is always 20ms
		printf("Loop Time %d\t", mymillis()- startInt);
    }
    
    //Terminate and shut off all the motors
    gpioWrite(IN1, 0);
	gpioWrite(IN2, 0);
	gpioWrite(IN3, 0);
	gpioWrite(IN4, 0);
    gpioTerminate();
    return 0;
}




