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
/*F2U not use
#define F2U00_MSG_LENGTH            38
#define F2U01_MSG_LENGTH            58
#define F2U_MSG_MAX_LENGTH          F2U01_MSG_LENGTH
#define F2U_MSG_MIN_LENGTH          F2U00_MSG_LENGTH


static uint8_T fcc2UwbBuff[F2U_MSG_MAX_LENGTH]; //58
static uint8_T testString[10];
/* fcc2Uwb not use
//Frame's initials
#define F2U_HEADER1                 (*(uint8_T *)(fcc2UwbBuff + 0)) //'F'
#define F2U_HEADER2                 (*(uint8_T *)(fcc2UwbBuff + 1)) //'C'
#define F2U_ID                      (*(uint8_T *)(fcc2UwbBuff + 2)) //0x00 or 0x01
#define F2U_LENGTH_BYTE             (*(uint8_T *)(fcc2UwbBuff + 3)) //F2U00_DATA_LENGTH or F2U01_DATA_LENGTH (32 or 52)

//Frame content msg 0x00
#define F2U00_X                     (*(float *)(fcc2UwbBuff + 4))
#define F2U00_Y                     (*(float *)(fcc2UwbBuff + 8))
#define F2U00_Z                     (*(float *)(fcc2UwbBuff + 12))
#define F2U00_XD                    (*(float *)(fcc2UwbBuff + 16))
#define F2U00_YD                    (*(float *)(fcc2UwbBuff + 20))
#define F2U00_ZD                    (*(float *)(fcc2UwbBuff + 24))
#define F2U00_TIMESTAMP             (*(unsigned long long *)(fcc2UwbBuff + 28))
#define F2U00_CS1                   (*(uint8_T *)(fcc2UwbBuff + 36))
#define F2U00_CS2                   (*(uint8_T *)(fcc2UwbBuff + 37))

#define F2U00_SUMCHECK_LENGTH       34
#define F2U00_DATA_LENGTH           32

//Frame content msg 0x01
#define F2U01_ANCID1                (*(uint8_T *)(fcc2UwbBuff + 4))
#define F2U01_ANCX1                 (*(float *)(fcc2UwbBuff + 5))
#define F2U01_ANCY1                 (*(float *)(fcc2UwbBuff + 9))
#define F2U01_ANCZ1                 (*(float *)(fcc2UwbBuff + 13))
#define F2U01_ANCID2                (*(uint8_T *)(fcc2UwbBuff + 17))
#define F2U01_ANCX2                 (*(float *)(fcc2UwbBuff + 18))
#define F2U01_ANCY2                 (*(float *)(fcc2UwbBuff + 22))
#define F2U01_ANCZ2                 (*(float *)(fcc2UwbBuff + 26))
#define F2U01_ANCID3                (*(uint8_T *)(fcc2UwbBuff + 30))
#define F2U01_ANCX3                 (*(float *)(fcc2UwbBuff + 31))
#define F2U01_ANCY3                 (*(float *)(fcc2UwbBuff + 35))
#define F2U01_ANCZ3                 (*(float *)(fcc2UwbBuff + 39))
#define F2U01_ANCID4                (*(uint8_T *)(fcc2UwbBuff + 43))
#define F2U01_ANCX4                 (*(float *)(fcc2UwbBuff + 44))
#define F2U01_ANCY4                 (*(float *)(fcc2UwbBuff + 48))
#define F2U01_ANCZ4                 (*(float *)(fcc2UwbBuff + 52))

#define F2U01_CS1                   (*(uint8_T *)(fcc2UwbBuff + 56))
#define F2U01_CS2                   (*(uint8_T *)(fcc2UwbBuff + 57))

#define F2U01_SUMCHECK_LENGTH       54
#define F2U01_DATA_LENGTH           52

uint8_T F2U01_ANCID(int id)
{
    return (*(uint8_T *)(fcc2UwbBuff + 4 + 13*id));
}
float F2U01_ANCPOS(int id, int var)
{
    return (*(float *)(fcc2UwbBuff + 5 + 13*id + 4*var));
}
*/

#define U2F00_MSG_LENGTH            32
#define U2F01_MSG_LENGTH            58
#define U2F_MSG_MAX_LENGTH          U2F01_MSG_LENGTH

static uint8_T uwb2FccBuff[U2F_MSG_MAX_LENGTH];
//define of read part

static uint8_t sread[32];
static bool synced = 0;
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

serial::Serial fd;

void bufferCheck();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serialdemo");
    ros::NodeHandle serialdemo_hdlr("~");
    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10); // previous it is 40
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
	int count = 0;
	double secondPassed;
	//variables define
	int que_ID = minus_Four(U2F00_ID);
	O2H_ID = 4; // default for ID = 0 to work
    while (ros::ok())
    {
		//startTime = clock();
        //check the buffer and display the data
        //bufferCheck();

        double time = ros::Time::now().toSec();
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
		// sending,
		printf("O2H_ID = %d, que_ID = %d\n",O2H_ID, que_ID);
        if (O2H_ID == que_ID){// or timout_flag
			fd.write(uwb2FccBuff, U2F00_MSG_LENGTH);
			print_sending_msg();
		}else{
			printf(KRED"did not send since is not my queue\n"RESET);
		}
		
		receving_message();
		if(synced){
			print_sread();
		}
		

	
        //loop_rate.sleep();

        count ++;
        //ros::Time timeUsed = ros::Time::now() - startTime;
		double loop_time = (ros::Time::now().toSec() - time);
        if (loop_time < 0.02){
			ros::Duration(0.02 - loop_time).sleep();
		}   
		ros:: Duration(4).sleep(); // for test in slow speed 
		// if shift following two line above, the speed will be 50 Hz.     
		printf ("time used: %f, accumulated time: %f \n, no of loops: %d", (ros::Time::now().toSec() - time), (ros::Time::now().toSec() - startTime), count);
        printf ("package loss numebr: %d\n", package_loss_nu);
    }


    return 0;
}
int receving_message(){// idea to sync and receive message 
	//sync
	while(!synced){ 
		size_t bytes_avail = 0;
		bytes_avail = fd.available();
		if (bytes_avail == 0){//search for buffering data, if no buffering data, next loop
			printf("no buffering data.\n");
			return -1;
			}
		fd.read((uint8_T *)(sread + 0), 1);
		if (O2H_HEADER1 == 'U') {//UW the hearder	
			fd.read((uint8_T *)(sread + 1), 1);
			if(O2H_HEADER2 == 'W'){
				fd.read((uint8_T *)(sread + 2),30);
				unsigned char CSA = 0, CSB = 0;
				for (unsigned char i = 0; i < U2F00_SUMCHECK_LENGTH; i++)
				{
					CSA = CSA + sread[2+i]; //4 - 31
					CSB = CSB + CSA; // 32
				}
				printf (KYEL"the CSA = %d , CSB = %d, the recevied: CS1 = %d, CS2 = %d:"RESET, CSA, CSB, O2H_CS1, O2H_CS2 );
				if (CSA == O2H_CS1 && CSB == O2H_CS2){
					synced = true;
					printf (KGRN"synced\n"RESET);
					return 1;
				}
			}
		}
	}
	//if synced
	if(fd.available() != 0){ // need at least 32
		fd.read(sread, 32);	
		unsigned char CSA = 0, CSB = 0;
		for (unsigned char i = 0; i < U2F00_SUMCHECK_LENGTH; i++)
		{
				CSA = CSA + sread[2+i]; //4 - 31
				CSB = CSB + CSA; // 32
		}
		printf (KYEL"the CSA = %d , CSB = %d, the recevied: CS1 = %d, CS2 = %d:"RESET, CSA, CSB, O2H_CS1, O2H_CS2 );
		if (O2H_HEADER1 == 'U' && O2H_HEADER2 == 'W' && CSA == O2H_CS1 && CSB == O2H_CS2){
			printf(KGRN"Matched\n"RESET);
			return 1;
		}else{
			printf("lost pakage, try sync again \n");
			package_loss_nu ++;
			synced = false;
			return 1;
		}
	}else {
		printf("no buffering data. try sync again\n");
		package_loss_nu ++;
		synced = false;
		return -1;
	}
		
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
	
int add_Four(int a){ // loop inside 0,1,2,3
	if (a < 0 || a > 3){
		printf(KRED"use a invalid input in the function add_four"RESET);
		}
		if (a == 3){
			return 0;
		}else{
			return a + 1;	
	}
}
int minus_Four(int a){ // loop inside 0,1,2,3
	if (a < 0 || a > 3){
		printf(KRED"use a invalid input in the function add_four"RESET);
		}
		if (a == 0){
			return 4;
		}else{
			return a - 1;	
	}
}


/*
void bufferCheck()
{
    static uint8_T synched = false;
    uint32_T bytes_avail = 0;

    bytes_avail = fd.available();

    //go on if there's no data to process
    if(bytes_avail != 0)
    {
        //If data stream is not in sync then we have to resync
        if(!synched)
        {
            //If buffered data is long enough, then can start re-sync, otherwise must return
            if(bytes_avail < F2U_MSG_MIN_LENGTH*2)
                return;
            else
            {
                //Sliding the frame byte by byte and search for header
                while(ros::ok() && bytes_avail >= F2U_MSG_MIN_LENGTH)
                {
                    //Shift and inject new byte to the 4th header byte then continue
                    fcc2UwbBuff[0] = fcc2UwbBuff[1];
                    fcc2UwbBuff[1] = fcc2UwbBuff[2];
                    fcc2UwbBuff[2] = fcc2UwbBuff[3];

                    fd.read(fcc2UwbBuff + 3, 1);

                    if(F2U_HEADER1 == 'F' && F2U_HEADER2 == 'C' && (F2U_ID == 0x00 || F2U_ID == 0x01)
                            && (F2U_LENGTH_BYTE == F2U00_DATA_LENGTH || F2U_LENGTH_BYTE == F2U01_DATA_LENGTH))
                    {
                        printf("Sync header found: %2x %2x %2x %2x\n", fcc2UwbBuff[0], fcc2UwbBuff[1], fcc2UwbBuff[2], fcc2UwbBuff[3]);

                        fd.read(fcc2UwbBuff + 4, F2U_LENGTH_BYTE + 2);
                        uint8_t CSA = 0, CSB = 0;
                        for (uint8_T i = 0; i < F2U_LENGTH_BYTE + 2; i++)
                        {
                            CSA = CSA + fcc2UwbBuff[2+i];
                            //printf("%2d:%02x ",i, fcc2UwbBuff[2+i]);
                            printf("%02x ", fcc2UwbBuff[2+i]);
                            CSB = CSB + CSA;
                        }
                        printf("%2x %2x\n", fcc2UwbBuff[F2U_LENGTH_BYTE + 4], fcc2UwbBuff[F2U_LENGTH_BYTE + 5]);

                        //if the CRC is valid then we can declare stream is synchronized
                        if(CSA == fcc2UwbBuff[F2U_LENGTH_BYTE + 4] && CSB == fcc2UwbBuff[F2U_LENGTH_BYTE + 5])
                        {
                            synched = true;
                            break;
                        }
                        else
                            printf("sync crc failed: %x/%x; %x/%x\n", CSA, fcc2UwbBuff[F2U_LENGTH_BYTE + 4], CSB, fcc2UwbBuff[F2U_LENGTH_BYTE + 5]);
                    }
                    bytes_avail = fd.available();
                }

                //If we have broken out of the while loop but still haven't synced, then the buffer has depleted
                if(!synched)
                    return;
            }
        }

        bytes_avail = fd.available();

        while(bytes_avail >= F2U_MSG_MIN_LENGTH)
        {
            printf("Buffered data = %d\n", bytes_avail);
            //first check the headers and id
            fd.read(fcc2UwbBuff, 4);
            if(F2U_HEADER1 == 'F' && F2U_HEADER2 == 'C' && F2U_ID == 0x00 && F2U_LENGTH_BYTE == F2U00_DATA_LENGTH)
            {
                //printf("Recieved an imu message.\n");
                fd.read(fcc2UwbBuff + 4, F2U00_MSG_LENGTH - 4);
                bytes_avail = fd.available();
                uint8_t CSA = 0, CSB = 0;
                for (uint8_T i = 0; i < F2U00_SUMCHECK_LENGTH; i++)
                {
                    CSA = CSA + fcc2UwbBuff[2+i];
                    CSB = CSB + CSA;
                }
                //printf("CSA = %d|%d, CSB = %d|%d\n", CSA, F2U00_CS1, CSB, F2U00_CS2);
                if(CSA == F2U00_CS1 && CSB == F2U00_CS2)
                {
                    printf("IMU update CRC valid\n");
                    for(int i = 0; i < F2U00_MSG_LENGTH; i++)
                        printf("%x ", fcc2UwbBuff[i]);
                    printf("\n");
                    for(int i = 0; i < 6; i++)
                    {
                        float temp = (*(float *)(fcc2UwbBuff + 4 + i*4));
                        printf("%4.2f ", temp);
                    }
                    if(fccTimestamp == F2U00_TIMESTAMP)
                    {
                        printf(KYEL"Timestamp not changed, frame ignored!\n"RESET);
                    }
                    else
                    {
                        printf("%d\n", F2U00_TIMESTAMP);
                        fcc_x = F2U00_X;
                        fcc_x = F2U00_Y;
                        fcc_z = F2U00_Z;
                        fcc_xd = F2U00_XD;
                        fcc_yd = F2U00_YD;
                        fcc_zd = F2U00_ZD;
                        fccTimestamp = F2U00_TIMESTAMP;

                        printf("%f; %f; %f; %f; %f; %f; %f; %f", fcc_x, fcc_y, fcc_z, fcc_xd, fcc_yd, fcc_zd, fccTimestamp);
//                        if(fccFirstTimestamp == 0)
//                        {
//                            fccFirstTimestamp = fccTimestamp;
//                            fccLog << "tf0=" << getLocalTimeNow();
//                        }
//                        else
//                        {
//                            if(logEnable)
//                                fccLog << "fx=[fx " << fcc_x << "];fy=[fy "<< fcc_y << "];fz=[fz " << fcc_z
//                                       << "];fxd=[fxd " << fcc_xd << "];fyd=[fyd " << fcc_yd << "];fzd=[fzd " << fcc_zd << "];"
//                                       << "tf=[tf " << (fccTimestamp - fccFirstTimestamp)/1000.0 << "];" << endl;
//                        }
                    }
                }
                else
                {
                    printf(KRED"CRC failed!\n"RESET);
                    //                    for(int i = 0; i < F2U00_MSG_LENGTH; i++)
                    //                        printf("%x ", fcc2UwbBuff[i]);
                    //                    printf("\n");
                    return;
                }

            }
//            else if(F2U_HEADER1 == 'F' && F2U_HEADER2 == 'C' && F2U_ID == 0x01 && F2U_LENGTH_BYTE == F2U01_DATA_LENGTH)
//            {
//                printf("Recieved an anchor message.\n");
//                fd.read(fcc2UwbBuff + 4, F2U01_MSG_LENGTH - 4);
//                bytes_avail = fd.available();
//                uint8_t CSA = 0, CSB = 0;
//                for (uint8_T i = 0; i < F2U01_SUMCHECK_LENGTH; i++)
//                {
//                    CSA = CSA + fcc2UwbBuff[2+i];
//                    CSB = CSB + CSA;
//                }
//                printf("CSA = %d|%d, CSB = %d|%d\n", CSA, F2U01_CS1, CSB, F2U01_CS2);
//                if(CSA == F2U01_CS1 && CSB == F2U01_CS2)
//                {
//                    printf("Anchor update CRC valid\n");
//                    for(int i = 0; i < F2U01_MSG_LENGTH; i++)
//                        printf("%x ", fcc2UwbBuff[i]);
//                    printf("\n");
//                    for(int i = 0; i < 4; i++)
//                    {
//                        ancsId[i] = F2U01_ANCID(i);
//                        for(int j = 0; j < 3; j++)
//                            ancsPos[i*3 + j] = F2U01_ANCPOS(i, j);
//                    }

//                    for(int i = 0; i < 3; i++)
//                        for(int j = 0; j < 4; j++)
//                        {
//                            //printf("Transposing i=%d, j=%d.\n", i, j);
//                            ancs[i*4 + j] = ancsPos[j*3 + i];
//                        }

//                    for(int i = 0; i < 12; i++)
//                        printf("%4.2f # ", ancsPos[i]);
//                    printf("\n");

//                    //send back the frame to FCC to confirm
//                    F2U_HEADER1 = 'U';
//                    F2U_HEADER2 = 'W';
//                    //tcdrain(fd);
//                    fd.write(fcc2UwbBuff, F2U01_MSG_LENGTH);
//                }
//                else
//                {
//                    printf(KRED"CRC failed!\n"RESET);
//                    for(int i = 0; i < F2U01_MSG_LENGTH; i++)
//                        printf("%x ", fcc2UwbBuff[i]);
//                    printf("\n");
//                    return;
//                }

//            }
            else
            {
                //if anything other than those above then we have got a misalignment, turn off 'synched' variable to re-sync in next calls
                printf(KRED"Unrecognized headers! Data stream out of sync\n"RESET);
                synched = false;
                return;
            }
        }
    }
}
*/

