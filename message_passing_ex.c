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
typedef struct _parametersInit{ 
	float x;
	float y;
	float angle;
} parametersInit;


typedef struct _parametersTrajectory{ 
	float timeCur;
	float leftVel;
	float rightVel;
	float heading;
	float x;
	float y;
	int flag;
} parametersTrajectory;

typedef struct _parametersVelocity{ 
	float timeCur;
	float leftVel;
	float rightVel;
	int flag;
} parametersVelocity;

typedef struct _parametersDesired{ 
	float timeCur;
	float velocity;
	float turningRate;
	int flag;
} parametersDesired;

pthread_barrier_t barrier;  // for synchronizing threads

//Function definitions
void* trajectoryThread(void*);
void* velocityThread(void*);
void* sensorThread(void*);
float rightWheelSensor();
float leftWheelSensor();

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
	//Initial parameters
	parametersInit param = {1, 1, 3.14/4};

   	//Give threads IDs 
   	pthread_t trajectoryThreadID;
   	pthread_t velocityThreadID;
   	pthread_t sensorThreadID;

	//Create barrier for synchronization
	pthread_barrier_init( &barrier, NULL, 3);

	//Create threads
	pthread_create(&trajectoryThreadID, NULL, trajectoryThread, (void*)&param);
	pthread_create(&velocityThreadID, NULL, velocityThread, NULL);
	pthread_create(&sensorThreadID, NULL, sensorThread, NULL);

	//Finish threads
	pthread_join(trajectoryThreadID, NULL );
	pthread_join(velocityThreadID, NULL );
	pthread_join(sensorThreadID, NULL );
	//exit(0);
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
void* trajectoryThread(void* inputParam){
	//Input
	parametersInit* initial;
	//Final Goal
	float xGoal;
	float yGoal;
	float angle;
	//Control System parameters
	float kp = 1;
	float kalpha = 1;
	float kphi = 1;
	//Variables for calculations
	float rho, alpha, phi, deltaX, deltaY;
	float vel, omega;

	int flag = 1;
	int retVal;						//Return value from message queue
	mqd_t sensorToTrajectoryQueue, trajectoryToVelocityQueue;	//Messeging queue
	parametersTrajectory paramIn;		//Parameters to be passed
	parametersDesired paramOut; 
	FILE *fpWrite;					//File pointer

	//Initialize values
	initial = (parametersInit*) inputParam;
	xGoal =1;
	yGoal =1 ;
	angle =3.14/2;


	//Open file
	fpWrite = fopen("Trajectory.txt", "w");
	           
	//Open messaging queue           
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

	//Wait here until sensor thread has created its message queue
	pthread_barrier_wait(&barrier);

	while(flag){
		//Block until data recieved
		retVal = mq_receive(sensorToTrajectoryQueue, (char*)&paramIn, 4096, NULL);

		//Error on recieve
		if(retVal< 0){
			printf("Error receiving from queue\n" );
			perror("Trajectory Thread");
		}else{
			fprintf(stderr, "Trajectory Recieve From Sensor %f\n", paramIn.timeCur);
		}
	              
		flag = paramIn.flag;			//get flag

        //Do processing 
        deltaX = xGoal - paramIn.x;
        deltaY = yGoal - paramIn.y;
		rho = sqrt(pow(deltaX ,2) + pow(deltaY,2));
		alpha = -1*paramIn.heading + atan(deltaX/deltaY);
		phi = -1*paramIn.heading + angle;
		if (phi<-3.14/2){
			while (phi<-3.14/2){
				phi+=3.14;
			}
		}else if (phi>3.14/2){
			while (phi>3.14/2){
				phi-=3.14;
			}
		}	

		//Find output values
		 omega = kalpha*alpha + kphi*phi;
		vel = kp*rho;
		//Set output
		paramOut.timeCur = paramIn.timeCur;
		paramOut.velocity = vel;
		paramOut.turningRate = omega;

		//Send data
		retVal = mq_send(trajectoryToVelocityQueue, (char*)&paramOut, sizeof(paramOut), 0);
		if(retVal < 0){
				printf("Trajectory Send to Sensor Thread Failed!\n");		
		}

        //Output data
        fprintf(fpWrite, "%f %f %f\n", paramOut.timeCur,paramOut.velocity,paramOut.turningRate);
	}

	fclose(fpWrite);

	//Exit thread
	pthread_exit(0);
	return(NULL);
}

/*******************************************************
* velocityThread
* Process data and send commands to motor
*
* params:
* none
*
* returns:
* none
*******************************************************/
void* velocityThread(void* unUsed){
	//current targets
	float leftVel = 0;
	float rightVel = 0;
	float heading = 0;
	//Controller parameters
	float kp = 1;
	float ki = 1;
	float u, u1, e, e1;
	float leftCMD, rightCMD;
	//Wheel parameters
	int R = 1;			//wheel radius
	int L = 1; 			//wheel base length

	int flag = 1;
	int retVal;						//Return value from message queue
	mqd_t sensorToVelocityQueue, trajectoryToVelocityQueue;	//Messeging queue
	parametersVelocity paramInSensor;		//Parameters to be passed
	parametersDesired paramInControl; 
	FILE *fpWrite;					//File pointer

	//Initialize control system parameters
	u = u1 = e = e1 = 0;

	//Open file
	fpWrite = fopen("Velocity.txt", "w");
	           
	//Open messaging queue           
	trajectoryToVelocityQueue = mq_open("trajectoryToVelocityQ", O_RDONLY|O_CREAT|O_NONBLOCK, S_IRWXU, NULL);
	sensorToVelocityQueue = mq_open("sensorToVelocityQ", O_RDONLY|O_CREAT, S_IRWXU, NULL);
	//Error check messaging queue
	if(sensorToVelocityQueue < 0){
		printf("Velocity Thread has Failed to Create Sensor Queue\n");
	}else{    
		printf("Velocity Thread has Created Sensor Queue!\n");
	}
    if(trajectoryToVelocityQueue < 0){
		printf("Velocity Thread has Failed to Create Trajectory Queue\n");
	}else{    
		printf("Velocity Thread has Created Trajectory Queue!\n");
	}

	//Wait here until sensor thread has created its message queue
	pthread_barrier_wait(&barrier);

	while(flag){
		//Block until data recieved
		retVal = mq_receive(sensorToVelocityQueue, (char*)&paramInSensor, 4096, NULL);

		//Error on recieve
		if(retVal< 0){
			printf("Error receiving from queue\n" );
		}else{
			fprintf(stderr, "Velocity Recieve From Sensor %f\n", paramInSensor.timeCur);
		}


		//See if any data to be recieved from Trajectory
		retVal = mq_receive(trajectoryToVelocityQueue, (char*)&paramInControl, 4096, NULL);
		if(retVal >= 0){
			leftVel = paramInControl.velocity/R - paramInControl.turningRate*L/(2*R);
			rightVel = paramInControl.velocity/R + paramInControl.turningRate*L/(2*R);
			heading = paramInControl.turningRate;
			fprintf(stderr, "Velocity Recieve From Trajectory %f\n", paramInControl.timeCur);
		}

		flag = paramInSensor.flag;			//get flag

        //Do processing
        e =  paramInSensor.leftVel - paramInSensor.rightVel;
        u = u1 + ki*(e + e1);
        e1 = e;
        u1 = u;
		leftCMD = (leftVel-paramInSensor.leftVel-u)*kp;
		rightCMD = (rightVel-paramInSensor.rightVel+u)*kp;

        //Output data
        fprintf(fpWrite, "%f %f %f %f %f  %f\n", paramInSensor.timeCur, leftVel, rightVel, heading, rightCMD, leftCMD);
	}

	fclose(fpWrite);		//close file

	//Exit thread
	pthread_exit(0);
	return(NULL);
 }

/*******************************************************
* sensorThread
* Gets data from sensor
*
* params:
* none
*
* returns:
* none
*******************************************************/
void* sensorThread(void* unUsed){
	float leftVel, rightVel;
	float distance, heading, prevTime;
	int flag = 1;	
	int retVal;

	int ticks = 0;		//Clock
	uint64_t cps, cycle0;
	float timeCur;

	parametersVelocity paramToVelocity;			//Parameters to be passed
	parametersTrajectory paramToTrajectory; 	
	mqd_t sensorToTrajectoryQueue, sensorToVelocityQueue;	//Message Queues

	//Do some initial clock setup
    cps = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
    cycle0 = ClockCycles();

    //Initialize Location
    prevTime = cycle0;
    distance = 0;
    heading = 3.14/4;

	//Open messaging queues	           
	sensorToTrajectoryQueue = mq_open("sensorToTrajectoryQ", O_WRONLY|O_CREAT|O_NONBLOCK, S_IRWXU, NULL);
	sensorToVelocityQueue = mq_open("sensorToVelocityQ", O_WRONLY|O_CREAT|O_NONBLOCK, S_IRWXU, NULL);
	//Error checking on message queue creation
	if(sensorToTrajectoryQueue < 0){
		printf("Sensor Thread has Failed to Create Trajectory Queue\n");
	}else{    
		printf("Sensor Thread has Created Trajectory Queue!\n");
	}
	if(sensorToVelocityQueue < 0){
		printf("Sensor Thread has Failed to Create Velocity Queue\n");
	}else{    
		printf("Sensor Thread has Created Velocity Queue!\n");
	}

	// wait here until sensor thread has created its message queue
	pthread_barrier_wait(&barrier);
	
	timeCur = (float)(ClockCycles()-cycle0)/(float)(cps);
	prevTime = timeCur;
	
	while (flag){
		//Clock
		timeCur = (float)(ClockCycles()-cycle0)/(float)(cps);

		//Get Data
		leftVel = rightWheelSensor();
		rightVel = leftWheelSensor();

		//Set the parameters for velocity
		paramToVelocity.timeCur = timeCur;			
		paramToVelocity.leftVel = leftVel; 					
		paramToVelocity.rightVel = rightVel;
		paramToVelocity.flag = 1;

		//calculate values for trajectory
		heading = 3.14/4;			//Calculations would happen if there was real data
		distance +=(rightVel+leftVel)/2*(timeCur-prevTime);
	
		//Set the parameters for trajectory
		paramToTrajectory.timeCur = timeCur;
		paramToTrajectory.leftVel = leftVel;
		paramToTrajectory.rightVel = rightVel;
	 	paramToTrajectory.heading = heading;
	 	paramToTrajectory.x = distance * cos(heading);
		paramToTrajectory.y = distance * sin(heading);
		paramToTrajectory.flag = 1;
		
		//At 10Hz
		if (ticks%10 == 0){
			//Times up - 10s test
			if (timeCur>10){
				paramToTrajectory.flag = 0;
				paramToVelocity.flag = 0;
				flag = 0;
			}
			//Send data to trajectory controller
			retVal = mq_send(sensorToTrajectoryQueue, (char*)&paramToTrajectory, sizeof(paramToTrajectory), 0);
			if(retVal < 0){
				printf("Sensor Send to Trajectory Thread Failed!\n");
			}
		}


		//At 100Hz
		//Send data to velocity controller
		retVal = mq_send(sensorToVelocityQueue, (char*)&paramToVelocity, sizeof(paramToVelocity), 0);
		if(retVal < 0){
				perror("Sensor");	
				printf("Sensor Send to Velocity Thread Failed!\n");
		}
		
		//Clock
		prevTime = timeCur;
		ticks++;	
		usleep(10000);				//Delay for 10ms
	}
}

/*******************************************************
* rightWheelSensor
* Gets velocity from right wheel
*
* params:
* none
*
* returns:
* float - velocity
*******************************************************/
float rightWheelSensor(){
	return 0.5;				//return 0.5m/s as per specs
}

/*
******************************************************
* leftWheelSensor 
* Gets velocity from left wheel
*
* params:
* none
*
* returns:
* float - velocity
*****************************************************
**/
float leftWheelSensor(){
	return 0.5;				//return 0.5m/s as per specs
}
