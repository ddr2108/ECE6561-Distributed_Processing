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

//Function definitions
void* velocityThread(void*);
void* sensorThread(void*);
float rightWheelSensor();
float leftWheelSensor();
parametersVelocity getSensor();
parametersDesired getTrajectory();
void sendVelocity(parametersVelocity paramToVelocity);
void sendTrajectory(parametersTrajectory paramToTrajectory);
void setupProxy();

//Global Variables
pthread_barrier_t barrier;  // for synchronizing threads
mqd_t sensorToVelocityQueueIn, trajectoryToVelocityQueue;	//Messeging queue
mqd_t sensorToVelocityQueueOut, sensorToTrajectoryQueue;	//Message Queues


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
	//Setup Proxy
	setupProxy();

	/////////THREADING/////////////////////////////////////////////////////////////////
   	//Give threads IDs 
   	pthread_t velocityThreadID;
   	pthread_t sensorThreadID;
	//Create barrier for synchronization
	pthread_barrier_init( &barrier, NULL, 2);
	//Create threads
	pthread_create(&velocityThreadID, NULL, velocityThread, NULL);
	pthread_create(&sensorThreadID, NULL, sensorThread, NULL);
	//Finish threads
	pthread_join(velocityThreadID, NULL );
	pthread_join(sensorThreadID, NULL );
	//exit(0);
	////////////////////////////////////////////////////////////////////////////////////
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
	float kb = 1;
	float u, u1, e, e1;
	float leftCMD, rightCMD;
	//Wheel parameters
	int R = 1;			//wheel radius
	int L = 1; 			//wheel base length

	int flag = 1;
	int retVal;						//Return value from message queue
	parametersVelocity paramInSensor;		//Parameters to be passed
	parametersDesired paramInControl; 
	FILE *fpWrite;					//File pointer

	//Initialize control system parameters
	u = u1 = e = e1 = 0;

	//Open file
	fpWrite = fopen("Velocity.txt", "w");
	           
	//Wait here until sensor thread has created its message queue
	pthread_barrier_wait(&barrier);

	while(flag){
		paramInSensor = getSensor();

		paramInControl = getTrajectory();
		if(paramInControl.timeCur != -1){
			leftVel = paramInControl.velocity/R - paramInControl.turningRate*L/(2*R);
			rightVel = paramInControl.velocity/R + paramInControl.turningRate*L/(2*R);
			heading = paramInControl.turningRate;
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

	//Do some initial clock setup
    cps = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
    cycle0 = ClockCycles();

    //Initialize Location
    prevTime = cycle0;
    distance = 0;
    heading = 3.14/4;

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
		
		//At 1Hz
		if (ticks%10 == 0){
			//Times up - 10s test
			if (timeCur>10){
				paramToTrajectory.flag = 0;
				paramToVelocity.flag = 0;
				flag = 0;
			}
			sendTrajectory(paramToTrajectory);
		}


		//At 10Hz
		sendVelocity(paramToVelocity);
		
		//Clock
		prevTime = timeCur;
		ticks++;	
		delay(100);				//Delay for 10ms
	}
}

/*******************************************************
* getSensor
* Gets data from sensors
*
* params:
* none
*
* returns:
* parametersVelocity - parameters from sensor
*******************************************************/
parametersVelocity getSensor(){
	parametersVelocity paramInSensor;
	int retVal;
	
	//Block until data recieved
	retVal = mq_receive(sensorToVelocityQueueIn, (char*)&paramInSensor, 4096, NULL);

	//Error on recieve
	if(retVal< 0){
		printf("Error receiving from queue\n" );
	}else{
		fprintf(stderr, "Velocity Recieve From Sensor %f\n", paramInSensor.timeCur);
	}

	return paramInSensor;
}

/*******************************************************
* getTrajectory
* Gets data from trajectory thread
*
* params:
* none
*
* returns:
* parametersDesired - data from trajectory thread
*******************************************************/
parametersDesired getTrajectory(){
	parametersDesired paramInControl; 
	int retVal;
	
	//See if any data to be recieved from Trajectory
	retVal = mq_receive(trajectoryToVelocityQueue, (char*)&paramInControl, 4096, NULL);

	if (retVal < 0){
		paramInControl.timeCur = -1;
	}else{
		fprintf(stderr, "Velocity Recieve From Trajectory %f\n", paramInControl.timeCur);
	}
	
	return paramInControl;
}

/*******************************************************
* sendVelocity
* Sends data to velocity thread
*
* params:
* parametersVelocity paramToVelocity - data to send
*
* returns:
* none
*******************************************************/
void sendVelocity(parametersVelocity paramToVelocity){
	int retVal;
	
//Send data to velocity controller
	retVal = mq_send(sensorToVelocityQueueOut, (char*)&paramToVelocity, sizeof(paramToVelocity), 0);
	if(retVal < 0){
		printf("Sensor Send to Velocity Thread Failed!\n");
	}
}

/*******************************************************
* sendTrajectory
* Sends data to trajectory thread
*
* params:
* parametersTrajectory paramToTrajectory - data to send
*
* returns:
* none
*******************************************************/
void sendTrajectory(parametersTrajectory paramToTrajectory){
	int retVal;
	
	//Send data to trajectory controller
	retVal = mq_send(sensorToTrajectoryQueue, (char*)&paramToTrajectory, sizeof(paramToTrajectory), 0);
	if(retVal < 0){
		printf("Sensor Send to Trajectory Thread Failed!\n");
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

/*
******************************************************
* setupProxy
* Setups up Proxy
*
* params:
* none
*
* returns:
* none
*****************************************************
**/
void setupProxy(){
///////////OPEN MESSAGING QUEUES////////////////////////////////////////////////
	//Open messaging queue           
	trajectoryToVelocityQueue = mq_open("/net/c440ece010/home/tessal/trajectoryToVelocityQ1", O_RDONLY|O_CREAT|O_NONBLOCK, S_IRWXU, NULL);
	sensorToVelocityQueueIn = mq_open("sensorToVelocityQ", O_RDONLY|O_CREAT, S_IRWXU, NULL);
	//Error check messaging queue
	if(sensorToVelocityQueueIn < 0){
		printf("Velocity Thread has Failed to Create Sensor Queue\n");
	}else{    
		printf("Velocity Thread has Created Sensor Queue!\n");
	}
    if(trajectoryToVelocityQueue < 0){
		printf("Velocity Thread has Failed to Create Trajectory Queue\n");
	}else{    
		printf("Velocity Thread has Created Trajectory Queue!\n");
	}

	//Open messaging queues	           
	sensorToTrajectoryQueue = mq_open("/net/c440ece010/home/tessal/sensorToTrajectoryQ1", O_WRONLY|O_CREAT|O_NONBLOCK, S_IRWXU, NULL);
	sensorToVelocityQueueOut = mq_open("sensorToVelocityQ", O_WRONLY|O_CREAT|O_NONBLOCK, S_IRWXU, NULL);
	//Error checking on message queue creation
	if(sensorToTrajectoryQueue < 0){
		printf("Sensor Thread has Failed to Create Trajectory Queue\n");
	}else{    
		printf("Sensor Thread has Created Trajectory Queue!\n");
	}
	if(sensorToVelocityQueueOut < 0){
		printf("Sensor Thread has Failed to Create Velocity Queue\n");
	}else{    
		printf("Sensor Thread has Created Velocity Queue!\n");
	}
	////////////////////////////////////////////////////////////////////////////////////
}