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
static unsigned long long fccTimestamp = 0;
static unsigned long long fccFirstTimestamp = 0;

#define     DFLT_FCC_PORT           "/dev/ttyUSB0"
#define     BAUD_MACRO              B921600
//function list
void print_sending_msg ();
void print_sread();
int receving_message();
int add_Four(int a);
int minus_Four(int a);

#define U2F00_MSG_LENGTH            32
#define U2F01_MSG_LENGTH            58
#define U2F_MSG_MAX_LENGTH          U2F01_MSG_LENGTH

static uint8_T uwb2FccBuff[U2F_MSG_MAX_LENGTH];

static int package_loss_nu = 0;
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

//define of sending part

#define U2F00_HEADER1               (*(uint8_T *)(uwb2FccBuff + 0)) //'U'
#define U2F00_HEADER2               (*(uint8_T *)(uwb2FccBuff + 1)) //'W'

#define U2F00_ID                    (*(uint8_T *)(uwb2FccBuff + 2))
#define U2F00_LENGTH_BYTE           (*(uint8_T *)(uwb2FccBuff + 3))
#define U2F00_X                     (*(float *)(uwb2FccBuff + 4))
#define U2F00_Y                     (*(float *)(uwb2FccBuff + 8))
#define U2F00_Z                     (*(float *)(uwb2FccBuff + 12))
#define U2F00_DX                    (*(float *)(uwb2FccBuff + 16))
#define U2F00_DY                    (*(float *)(uwb2FccBuff + 20))
#define U2F00_DZ                    (*(float *)(uwb2FccBuff + 24))
#define U2F00_NODECOUNT             (*(uint8_T *)(uwb2FccBuff + 28))
#define U2F00_LOSS                  (*(uint8_T *)(uwb2FccBuff + 29))

#define U2F00_CS1                   (*(uint8_T *)(uwb2FccBuff + 30))
#define U2F00_CS2                   (*(uint8_T *)(uwb2FccBuff + 31))
#define U2F00_SUMCHECK_LENGTH       28
#define U2F00_DATA_LENGTH           26

using namespace std;

//freq
const int single_loop_rate = 5;
double rest_after_sync = single_loop_rate/1000.0 * 1.1;//half time of the loop 
//define of read part
static uint8_t sread[32];
static bool synced = 0;
static bool updated = 0;
//static bool time_reached = 0;
static int NodeNo = 3;
int empty_loop = 0;


serial::Serial fd;

void bufferCheck();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serialdemo");
    ros::NodeHandle serialdemo_hdlr("~");
    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    
  
    ros::Rate loop_rate(single_loop_rate); // previous it is 40
    double rest_after_send = single_loop_rate/2000.0;//half time of the loop 
    
    //FCC interfacing
    string fccSerialPort = string("/dev/ttyUSB0");
    //check for the FCC port name
    if(serialdemo_hdlr.getParam("fccSerialPort", fccSerialPort))
        printf(KBLU"Retrieved value %s for param 'fccSerialPort'!\n"RESET, fccSerialPort.data());
    else
        printf(KRED "Couldn't retrieve param 'fccSerialPort', using default port %s!\n"RESET, fccSerialPort.data());
    int ID_read;
    if (serialdemo_hdlr.getParam("ID", ID_read)){
		printf(KBLU"Retrived ID = %d for param ID\n"RESET, ID_read );
		U2F00_ID = ID_read;
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
	int whole_info_loop_count = 0;
	int pkg_loss_100;
	double secondPassed;

	//variables define
	int que_ID = minus_Four(U2F00_ID);
	O2H_ID = NodeNo- 1; // default for ID = 0 to work for ID 0 need to receive ID  = NodeNo - 1;
    while (ros::ok())
    {
		//startTime = clock();
        //check the buffer and display the data
        //bufferCheck();

        double time = ros::Time::now().toSec();
        printf(KMAG"time used = %f \n"RESET, time - Time_loop );
        Time_loop = time;     
        //generate sin functions
        float x = sin(time);//sin(2*PI*10*time);
        float y = cos(time);//sin(2*PI*10*time + PI/2);
        float z = sin(time/10);//sin(2*PI*10*time + 3*PI/2);

        //populate the header values
        U2F00_HEADER1 = 'U';
        U2F00_HEADER2 = 'W';
        //U2F00_ID = 0; defeined at the top when retriving ID
        U2F00_LENGTH_BYTE = U2F00_DATA_LENGTH;
        U2F00_X = x;
        U2F00_Y = y;
        U2F00_Z = z;
        U2F00_DX = y;
        U2F00_DY = -x;
        U2F00_DZ = y/10;
        U2F00_NODECOUNT = 4;
        U2F00_LOSS = 0;

        //load the crc
        unsigned char CSA = 0, CSB = 0;
        for (unsigned char i = 0; i < U2F00_SUMCHECK_LENGTH; i++)
        {
            CSA = CSA + uwb2FccBuff[2+i]; //4 - 31
            CSB = CSB + CSA; // 32
        }
        U2F00_CS1 = CSA;
        U2F00_CS2 = CSB;
		

        printf ("looping \n");
		//fd.write(uwb2FccBuff, F2U00_MSG_LENGTH);
		receving_message();
		if(synced){
			print_sread();
		}		
		// sending,
		printf("O2H_ID = %d, que_ID = %d\n",O2H_ID, que_ID);
        if (O2H_ID == que_ID){// or timout_flag
		
			if(updated){
				fd.write(uwb2FccBuff, U2F00_MSG_LENGTH);
				print_sending_msg();
				whole_info_loop_count++;// may let this start after 4 zigbee connection complete
				printf(KMAG"time for one info pass used = %f \n"RESET, ros::Time::now().toSec() - Time_loop2 );
				Time_loop2 = ros::Time::now().toSec(); 
				//ros::Duration(rest_after_send).sleep();// added rest time for waiting 
			}else {
				static int loop_accum = 0;
				loop_accum ++;
				if (loop_accum > 3){
					printf (KRED "no response, try send again\n"RESET);
					fd.write(uwb2FccBuff, U2F00_MSG_LENGTH);
					print_sending_msg();
					loop_accum = 0;
					}
			
			}
			
			
		}else{
			printf(KRED"did not send since is not my queue\n"RESET);
		}

		
       /* 
        //ros::Time timeUsed = ros::Time::now() - startTime;
		double loop_time = (ros::Time::now().toSec() - time);
        if (loop_time < 0.02){
			ros::Duration(0.02 - loop_time).sleep();
		}   
		//ros:: Duration(0.5).sleep(); // for test in slow speed 
		// if shift following two line above, the speed will be 50 Hz.     
		printf ("time used: %f, accumulated time: %f \n, no of loops: %d", (ros::Time::now().toSec() - time), (ros::Time::now().toSec() - startTime), count);
		*/
		loop_count++;
		if ((loop_count % 100) == 0 )
			pkg_loss_100 = package_loss_nu;
        printf ("package loss numebr: %d, percentage : %f%%, percentage for every 100 loops: %d%% \n", package_loss_nu, ((package_loss_nu/(double)loop_count)*100), package_loss_nu - pkg_loss_100 );
		printf ("loops: %d, empty loops: %d, whole loops count: %d\n",loop_count, empty_loop, whole_info_loop_count);
        loop_rate.sleep();


    }


    return 0;
}
int receving_message(){// idea to sync and receive message 
	updated = 0;
	//sync
	size_t bytes_avail = 0;
	bytes_avail = fd.available();
	if (bytes_avail >= U2F00_MSG_LENGTH * 2){
		int cut_no =  (bytes_avail-(U2F00_MSG_LENGTH * 2) + 1);
		for (int i = 0; i < cut_no; i++){
			fd.read((uint8_T *)(sread + 1), 1);
			}
		printf (KRED"##################################################################cutting cut: %d\n"RESET,cut_no);
		synced = false;
	}
	//while(!synced){ 
		
		bytes_avail = fd.available();
		if (bytes_avail == 0){//search for buffering data, if no buffering data, next loop
			printf("no buffering data.\n");
			empty_loop++;
			return -1;
			}
		fd.read((uint8_T *)(sread + 0), 1);
		if (O2H_HEADER1 == 'U') {//UW the hearder	
			fd.read((uint8_T *)(sread + 1), 1);
			if(O2H_HEADER2 == 'W'){
				fd.read((uint8_T *)(sread + 2),U2F00_MSG_LENGTH);
				unsigned char CSA = 0, CSB = 0;
				for (unsigned char i = 0; i < U2F00_SUMCHECK_LENGTH; i++)
				{
					CSA = CSA + sread[2+i]; //4 - 31
					CSB = CSB + CSA; // 32
				}
				printf (KYEL"the CSA = %d , CSB = %d, the recevied: CS1 = %d, CS2 = %d:"RESET, CSA, CSB, O2H_CS1, O2H_CS2 );
				if (CSA == O2H_CS1 && CSB == O2H_CS2){
					synced = true;
					updated = true; // updated
					printf (KGRN"synced\n"RESET);
					return 1;
				}
			}
		}
	//}
	//if synced
	/*if(fd.available() != 0){ // need at least 32
		fd.read(sread, U2F00_MSG_LENGTH);	
		unsigned char CSA = 0, CSB = 0;
		for (unsigned char i = 0; i < U2F00_SUMCHECK_LENGTH; i++)
		{
				CSA = CSA + sread[2+i]; //4 - 31
				CSB = CSB + CSA; // 32
		}
		printf (KYEL"the CSA = %d , CSB = %d, the recevied: CS1 = %d, CS2 = %d:"RESET, CSA, CSB, O2H_CS1, O2H_CS2 );
		if (O2H_HEADER1 == 'U' && O2H_HEADER2 == 'W' && CSA == O2H_CS1 && CSB == O2H_CS2){
			printf(KGRN"Matched\n"RESET);
			updated = true; // updated
			return 1;
		}else{
			printf("lost package, try sync again \n");
			package_loss_nu ++;
			synced = false;
			return 1;
		}
	}else {
		printf("no buffering data. try sync again\n");
		package_loss_nu ++;
		synced = false;
		//ros::Duration(rest_after_sync).sleep(); //
		return -1;
	}
	*/
		
}

void print_sending_msg(){ // used for print msg for sending or local info
		printf(KBLU"the sending message: ---------------------\n");
        printf ("Header1 = %c\n", U2F00_HEADER1);
		printf ("Header2 = %c\n", U2F00_HEADER2);
		printf ("ID = %d\n", U2F00_ID);
		printf ("LENGTH = %d\n",U2F00_LENGTH_BYTE);
		printf ("x  = %f, y  = %f, z  = %f\n", U2F00_X, U2F00_Y, U2F00_Z);
		printf ("Dx = %f, Dy = %f, Dz = %f\n", U2F00_DX, U2F00_DY, U2F00_DZ);
		printf ("Nodecount = %d\n", U2F00_NODECOUNT);
		printf ("loss = %d\n", U2F00_LOSS);
		printf ("CS1 = %d, CS2 = %d\n"RESET, U2F00_CS1, U2F00_CS2);
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
	
int add_Four(int a){ // loop inside 0,1,2,3 ...... NodeNo - 1
	if (a < 0 || a > (NodeNo-1)){
		printf(KRED"use a invalid input in the function add_four"RESET);
		}
		if (a == (NodeNo-1)){
			return 0;
		}else{
			return a + 1;	
	}
}
int minus_Four(int a){ // loop inside 0,1,2,3 ... NodeNo -1
	if (a < 0 || a > (NodeNo-1)){
		printf(KRED"use a invalid input in the function add_four"RESET);
		}
		if (a == 0){
			return (NodeNo-1);
		}else{
			return a - 1;	
	}
}



