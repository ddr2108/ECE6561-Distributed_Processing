// message passing example for QNX, 
// Filename: message_passing_ex.c 

#include <stdio.h>
#include <pthread.h>
#include <mqueue.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <inttypes.h>
#include <sys/neutrino.h>
#include <sys/syspage.h>

//Structures of data to be passed
typedef struct _parametersTrajectory{ 
	float timeCur;
	float leftVel;
	float rightVel;
	float heading;
	float x;
	float y;
	int flag;
} parametersTrajectory;

typedef struct _parametersDesired{ 
	float timeCur;
	float velocity;
	float turningRate;
	int flag;
} parametersDesired;

//Function definitions
void* trajectoryThread(void*);
void parametersTrajectory getSensor();
parametersTrajectory getSensor();

//Global Variables
mqd_t sensorToTrajectoryQueue, trajectoryToVelocityQueue;	//Messeging queue

/*******************************************************
* main
* main thread
*
* params:
* none
*
* returns:
* none
*******************************************************/
int main(void){
	///////////OPEN MESSAGING QUEUES////////////////////////////////////////////////
	//Open messaging queues         
	sensorToTrajectoryQueue = mq_open("sensorToTrajectoryQ", O_RDONLY|O_CREAT, S_IRWXU, NULL);
	trajectoryToVelocityQueue = mq_open("trajectoryToVelocityQ", O_WRONLY|O_CREAT|O_NONBLOCK, S_IRWXU, NULL);
	//Error check messaging queue
	if(sensorToTrajectoryQueue < 0){
		printf("Trajectory Thread has Failed to Create Sensor Queue\n");
	}else{    
		printf("Trajectory Thread has Created Sensor Queue!\n");
	}
   //Error check messaging queue
	if(trajectoryToVelocityQueue < 0){
		printf("Trajectory Thread has Failed to Create Velocity Queue\n");
	}else{    
		printf("Trajectory Thread has Created Velocity Queue!\n");
	}
	////////////////////////////////////////////////////////////////////////////////////
	/////////THREADING/////////////////////////////////////////////////////////////////
   	//Give threads IDs 
   	pthread_t trajectoryThreadID;
	//Create threads
	pthread_create(&trajectoryThreadID, NULL, trajectoryThread, (void*)&param);	
	//Finish threads
	pthread_join(trajectoryThreadID, NULL );
	////////////////////////////////////////////////////////////////////////////////////
}

/*******************************************************
* trajectoryThread
* Process data and send commands to LL controller
*
* params:
* none
*
* returns:
* none
*******************************************************/
void* trajectoryThread(void* unUsed){
	//Final Goal
	float xGoal = 1;
	float yGoal = 1;
	float angle = 3.14/2;
	//Control System parameters
	float kp = 1;
	float kalpha = 1;
	float kphi = 1;
	//Variables for calculations
	float rho, alpha, phi, deltaX, deltaY;
	float vel, omega;

	int flag = 1;
	parametersTrajectory paramIn;		//Parameters to be passed
	parametersDesired paramOut; 
	FILE *fpWrite;					//File pointer
	           
	//Open file
	fpWrite = fopen("Trajectory.txt", "w");

	while(flag){
		//Recieve data
		paramIn = getSensor();
	              
	    //get flag
		flag = paramIn.flag;			
        //Do processing 
        deltaX = xGoal - paramIn.x;
        deltaY = yGoal - paramIn.y;

        phi = -1*paramIn.heading + angle;
		while (phi<-3.14/2){
			phi+=3.14;
		}
		while (phi>3.14/2){
			phi-=3.14;
		}

		rho = sqrt(pow(deltaX ,2) + pow(deltaY,2));
		if (deltaY!=0){
			alpha = -1*paramIn.heading + atan(deltaX/deltaY);
		}else{
			alpha = -1*paramIn.heading + 3.14/2;
		}
		
		
		//Find output values
		omega = kalpha*alpha + kphi*phi;
		vel = kp*rho;
		//Set output
		paramOut.timeCur = paramIn.timeCur;
		paramOut.velocity = vel;
		paramOut.turningRate = omega;

		//Send data
		sendVel(paramOut);

        //Output data
        fprintf(fpWrite, "%f %f %f\n", paramOut.timeCur,paramOut.velocity,paramOut.turningRate);
	}

	fclose(fpWrite);

	//Exit thread
	pthread_exit(0);
	return(NULL);
}

/*******************************************************
* getSensor
* gets data from sensor
*
* params:
* none
*
* returns:
* parametersTrajectory - the parameters from sensor
*******************************************************/
parametersTrajectory getSensor(){
	parametersTrajectory paramIn;		//Parameters to be passed in from sensor

	//Block until data recieved
	retVal = mq_receive(sensorToTrajectoryQueue, (char*)&paramIn, 4096, NULL);

	//Error on recieve
	if(retVal< 0){
		printf("Error receiving from queue\n" );
		perror("Trajectory Thread");
	}else{
		fprintf(stderr, "Trajectory Recieve From Sensor %f\n", paramIn.timeCur);
	}

	return paramIn;
}

/*******************************************************
* sendVel
* sends data to velocity thread
*
* params:
* parametersDesired paramOut - parameters to send
*
* returns:
* none
*******************************************************/
void sendVel(parametersDesired paramOut){
	//Send data
	retVal = mq_send(trajectoryToVelocityQueue, (char*)&paramOut, sizeof(paramOut), 0);
	if(retVal < 0){
		printf("Trajectory Send to Sensor Thread Failed!\n");		
	}

}