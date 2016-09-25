#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <math.h>
#include <cstring>
#include <ctime>

#include "rtwtypes.h"

#include <sstream>
#include <serial/serial.h>

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define RESET "\033[0m"

#define PI 3.14

//_____________________________________________________________________________
//
// FCC Serial Communications
//_____________________________________________________________________________
static float fcc_x = 0, fcc_y = 0, fcc_z = 0, fcc_xd = 0, fcc_yd = 0, fcc_zd = 0;

#define     DFLT_FCC_PORT           "/dev/ttyUSB0"
#define     BAUD_MACRO              B921600
//function list
void print_sending_msg ();
void print_sread();
int receving_message();
int add_Four(int a);
int minus_Four(int a);

//********************Global variable define
//********************variable define
//********************freq
const int single_loop_rate = 20;
double rest_after_sync = single_loop_rate/1000.0 * 1.1;//half time of the loop 
const int NodeNo = 3;

//********************the global variable used
static bool synced = 0;
static bool updated = 0;
static bool speaking = 0;
static bool listening = 1;
static bool listener_time_out = 0;
//static bool time_reached = 0;
static int package_loss_nu = 0;




//********************Frame's initials
#define local_MSG_LENGTH            32
const double receve_time_out  = 32 * 0.002 + 0.01; // 2ms for each byte and 10 ms for extra wait
//define of read part
static uint8_t swrite[local_MSG_LENGTH];
static uint8_t sread[local_MSG_LENGTH];
static uint8_t sread_bak[local_MSG_LENGTH];
//*******************OUT to Here : O2H
#define O2H_HEADER1               (*(uint8_T *)(sread + 0)) //'U'
#define O2H_HEADER2               (*(uint8_T *)(sread + 1)) //'W'

#define O2H_ID                    (*(uint8_T *)(sread + 2))
#define O2H_LENGTH_BYTE           (*(uint8_T *)(sread + 3))
#define O2H_X                     (*(float *)(sread + 4))
#define O2H_Y                     (*(float *)(sread + 8))
#define O2H_Z                     (*(float *)(sread + 12))
#define O2H_DX                    (*(float *)(sread + 16))
#define O2H_DY                    (*(float *)(sread + 20))
#define O2H_DZ                    (*(float *)(sread + 24))
#define O2H_NODECOUNT             (*(uint8_T *)(sread + 28))
#define O2H_LOSS                  (*(uint8_T *)(sread + 29))

#define O2H_CS1                   (*(uint8_T *)(sread + 30))
#define O2H_CS2                   (*(uint8_T *)(sread + 31))

//*************************define of sending part

#define local_HEADER1               (*(uint8_T *)(swrite + 0)) //'U'
#define local_HEADER2               (*(uint8_T *)(swrite + 1)) //'W'

#define local_ID                    (*(uint8_T *)(swrite + 2))
#define local_LENGTH_BYTE           (*(uint8_T *)(swrite + 3))
#define local_X                     (*(float *)(swrite + 4))
#define local_Y                     (*(float *)(swrite + 8))
#define local_Z                     (*(float *)(swrite + 12))
#define local_DX                    (*(float *)(swrite + 16))
#define local_DY                    (*(float *)(swrite + 20))
#define local_DZ                    (*(float *)(swrite + 24))
#define local_NODECOUNT             (*(uint8_T *)(swrite + 28))
#define local_LOSS                  (*(uint8_T *)(swrite + 29))

#define local_CS1                   (*(uint8_T *)(swrite + 30))
#define local_CS2                   (*(uint8_T *)(swrite + 31))
#define local_SUMCHECK_LENGTH       28
#define local_DATA_LENGTH           26

using namespace std;

serial::Serial fd;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "serialdemo");
    ros::NodeHandle serialdemo_hdlr("~");
    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(single_loop_rate); // previous it is 40
   
    
    //FCC interfacing
    string fccSerialPort = string("/dev/ttyUSB0");
    //check for the FCC pot and the ID for the UAV
    if(serialdemo_hdlr.getParam("fccSerialPort", fccSerialPort))
        printf(KBLU"Retrieved value %s for param 'fccSerialPort'!\n"RESET, fccSerialPort.data());
    else
        printf(KRED "Couldn't retrieve param 'fccSerialPort', using default port %s!\n"RESET, fccSerialPort.data());
    int ID_read;
    if (serialdemo_hdlr.getParam("ID", ID_read)){
		printf(KBLU"Retrived ID = %d for param ID\n"RESET, ID_read );
		local_ID = ID_read;
	}else{
		printf(KRED"can't retrive the ID, please try again.\n"RESET);
		  exit (EXIT_FAILURE);
	}

    fd.setPort(fccSerialPort.data());
    fd.setBaudrate(57600);
    fd.setTimeout(5, 10, 2, 10, 2);
    fd.open();
    if (fd.isOpen())
    {
        fd.flushInput();
        printf(KBLU "Connection established\n\n" RESET);
    }
    else
    {
        printf(KRED "serialInit: Failed to open port %s\n" RESET, fccSerialPort.data());
        return 0;
    }
	//CREATE TIMER;
	double startTime = (double)ros::Time::now().toSec();
	double Time_loop = startTime;
	double Time_loop2 = startTime;
	int loop_count = 0;
	

	
	//variables define
	int que_ID = minus_Four(local_ID);
	O2H_ID = NodeNo - 1; // default for ID = 0 to work, need to receive NodeNo - 1
    while (ros::ok())
    {
        double time = ros::Time::now().toSec(); // counting the time
        printf(KMAG"time used = %f \n"RESET, time - Time_loop );
        Time_loop = time;     
        //**********************************updating the infomation
        //generate sin functions
        float x = sin(time);//sin(2*PI*10*time);
        float y = cos(time);//sin(2*PI*10*time + PI/2);
        float z = sin(time/10);//sin(2*PI*10*time + 3*PI/2);

        //populate the header values
        local_HEADER1 = 'U';
        local_HEADER2 = 'W';
        //local_ID = 0; defeined at the top when retriving ID
        local_LENGTH_BYTE = local_DATA_LENGTH;
        local_X = x;
        local_Y = y;
        local_Z = z;
        local_DX = y;
        local_DY = -x;
        local_DZ = y/10;
        local_NODECOUNT = 4;
        local_LOSS = 0;

        //load the crc
        unsigned char CSA = 0, CSB = 0;
        for (unsigned char i = 0; i < local_SUMCHECK_LENGTH; i++)
        {
            CSA = CSA + swrite[2+i]; //4 - 31
            CSB = CSB + CSA; // 32
        }
        local_CS1 = CSA;
        local_CS2 = CSB;

	
		while (listening){ // listen first
			//static listen_timer = ros::Time::now().ToSec();
			receving_message();
			//print_sread();
			if ( (O2H_ID == que_ID) && (updated || listener_time_out)  ){ // under receive ID = que_ID, if receive ID is updated or listener has time out, send again
				speaking = 1;
				listening = !speaking;
				updated = 0;    // reset both back to 0 and wait for the next listen period.
				listener_time_out = 0;
				}
		}
		while(speaking){ //immediately talk after listen
			fd.write(swrite, local_MSG_LENGTH);
			//print_sending_msg();
			speaking = 0;
			listening = !speaking;
		}
		loop_count++;

        	printf ("package loss numebr: %d, \n", package_loss_nu );
		printf ("loops: %d, \n",loop_count);
        	//loop_rate.sleep();
	}
    	return 0;
  }

int receving_message(){// idea to sync and receive message , at least receive a synced message or time_out
	//sync
	double receve_timer = ros::Time::now().toSec(); // counting the time
	
	while(1){
		if (fd.available() >= local_MSG_LENGTH){ // >= enough bytes
				while(!synced){ 
					fd.read((uint8_T *)(sread + 0), 1);
					//try sync
					if (O2H_HEADER1 == 'U') {//UW the hearder	
						fd.read((uint8_T *)(sread + 1), 1);
						if(O2H_HEADER2 == 'W'){
							// back up the previous data 
							memcpy(sread_bak,sread, local_MSG_LENGTH );// dest: sread_bak, source: sread
							fd.read((uint8_T *)(sread + 2),local_MSG_LENGTH -2 );
							unsigned char CSA = 0, CSB = 0;
							for (unsigned char i = 0; i < local_SUMCHECK_LENGTH; i++)
							{
								CSA = CSA + sread[2+i]; //4 - 31
								CSB = CSB + CSA; // 32
							}
							printf (KYEL"the CSA = %d , CSB = %d, the recevied: CS1 = %d, CS2 = %d:"RESET, CSA, CSB, O2H_CS1, O2H_CS2 );
							if (CSA == O2H_CS1 && CSB == O2H_CS2){
								synced = true;
								updated = true; // updated
								//print_sread();// two of this in receve
								printf (KGRN"synced\n"RESET);
								return 1;
							}else{ // failed, by default, synced = false, update = false
								memcpy(sread + 2,sread_bak + 2, local_MSG_LENGTH - 2 );
								break; // jump out
								//no return, will try resync
								receve_timer = ros::Time::now().toSec(); // update timer for another count
							}
						}//W
					}//U
					//after trying to sync, if buffer become empty
					if (fd.available() == 0){ //break back to receive function, update the timer for time out
						break; // jump out
						//no return, will try resync
						receve_timer = ros::Time::now().toSec();
					}
			}
			if (synced){
				memcpy(sread_bak,sread, local_MSG_LENGTH ); //back up sread to sread_bak
				fd.read(sread, local_MSG_LENGTH);	
				unsigned char CSA = 0, CSB = 0;
				for (unsigned char i = 0; i < local_SUMCHECK_LENGTH; i++)
				{
						CSA = CSA + sread[2+i]; //4 - 31
						CSB = CSB + CSA; // 32
				}
				printf (KYEL"the CSA = %d , CSB = %d, the recevied: CS1 = %d, CS2 = %d:"RESET, CSA, CSB, O2H_CS1, O2H_CS2 );
				if (O2H_HEADER1 == 'U' && O2H_HEADER2 == 'W' && CSA == O2H_CS1 && CSB == O2H_CS2){ // check all 4 directly
					// if correct, updated, return to main function and see if ID match the que
					printf(KGRN"Matched\n"RESET);
					updated = true; // updated
					//print_sread();//two of this in receive
					return 1;
				}else{ // 1. restore, 2.package loss count, 3. synced turn off 4 update the time out and retry the receiving
					printf("lost pakage, try sync again \n");
					memcpy(sread + 2,sread_bak + 2, local_MSG_LENGTH - 2 );
					package_loss_nu ++;
					synced = false;
					//no return, will check length again and try resync
					receve_timer = ros::Time::now().toSec(); // update the timer for another count
				}
			}
		}
		// if not enough bety in the buffer
		if ((ros::Time::now().toSec() - receve_timer) > receve_time_out){
				listener_time_out = 1;
				printf(KRED"time_out\n"RESET);
				return -1;
		}
	}	// under while (1)
}

void print_sending_msg(){ // used for print msg for sending or local info
		printf(KBLU"the sending message: ---------------------\n");
        printf ("Header1 = %c\n", local_HEADER1);
		printf ("Header2 = %c\n", local_HEADER2);
		printf ("ID = %d\n", local_ID);
		printf ("LENGTH = %d\n",local_LENGTH_BYTE);
		printf ("x  = %f, y  = %f, z  = %f\n", local_X, local_Y, local_Z);
		printf ("Dx = %f, Dy = %f, Dz = %f\n", local_DX, local_DY, local_DZ);
		printf ("Nodecount = %d\n", local_NODECOUNT);
		printf ("loss = %d\n", local_LOSS);
		printf ("CS1 = %d, CS2 = %d\n"RESET, local_CS1, local_CS2);
	}

void print_sread(){ //used for print readed msg from others
	    printf ("the received message: ************************\n");
        printf ("Header1 = %c\n", O2H_HEADER1);
		printf ("Header2 = %c\n", O2H_HEADER2);
		printf ("ID = %d\n", O2H_ID);
		printf ("LENGTH = %d\n",O2H_LENGTH_BYTE);
		printf ("x  = %f, y  = %f, z  = %f\n", O2H_X, O2H_Y, O2H_Z);
		printf ("Dx = %f, Dy = %f, Dz = %f\n", O2H_DX, O2H_DY, O2H_DZ);
		printf ("Nodecount = %d\n", O2H_NODECOUNT);
		printf ("loss = %d\n", O2H_LOSS);
		printf ("CS1 = %d, CS2 = %d\n", O2H_CS1, O2H_CS2);
	}
	
int add_Four(int a){ // loop inside 0,1,2,3... NodeNo -1
	if (a < 0 || a > (NodeNo -1)){
		printf(KRED"use a invalid input in the function add_four"RESET);
		}
		if (a == (NodeNo -1)){
			return 0;
		}else{
			return a + 1;	
	}
}
int minus_Four(int a){ // loop inside 0,1,2,3 .... NodeNo -1
	if (a < 0 || a > (NodeNo -1)){
		printf(KRED"use a invalid input in the function add_four"RESET);
		}
		if (a == 0){
			return (NodeNo -1 );
		}else{
			return a - 1;	
	}
}



