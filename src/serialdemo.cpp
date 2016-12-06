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
int loop_add(int a);
int loop_minus(int a);

//********************Global variable define
//********************variable define
//********************freq
static int single_loop_rate = 5;
double rest_after_sync = single_loop_rate/1000.0 * 1.1;//half time of the loop
static int NodeNo = 3;

//********************the global variable used
static bool synced = 0;
static bool updated = 0;
static bool speaking = 0;
static bool listening = 1;
static bool listener_time_out = 0;
static bool flag_timeout_timer = 0;
//static bool time_reached = 0;
static int package_loss_nu = 0;
static double receive_time_out  = 32 * 0.002 + 0.01; // 2ms for each byte and 10 ms for extra wait



//********************Frame's initials


#define local_MSG_LENGTH            43

//define of read part
static uint8_T swrite[local_MSG_LENGTH];



#define local_SD          		    (*(uint8_T *)(swrite + 0)) //0x7E
#define local_Length1               (*(uint16_T *)(swrite + 1)) // 1.2   0xXX XX total bytes - 4 (most likely, start delimiter(1), length(2) and checksum(1) ),  42 (add the 0) - 4=38 = 0x26
#define local_Length2               (*(uint16_T *)(swrite + 2))
#define local_FrameType          	(*(uint8_T *)(swrite + 3)) // 3  0x10 Transmit Request
#define local_FrameID				(*(uint8_T *)(swrite + 4)) // 4 0x00  or other number use 00 means no response

#define local_64Addr1				(*(uint32_T *)(swrite + 5)) //5678 9abc 5 to 8
#define local_64Addr2               (*(uint32_T *)(swrite + 9)) //5678 9abc 9 to 12
#define local_16Addr				(*(uint16_T *)(swrite + 13)) // 13,14 0xFF FE
#define local_Broadcast_r			(*(uint8_T *)(swrite + 15)) // 15, 0x00 00 max hop times
#define local_Options				(*(uint8_T *)(swrite + 16)) //16 0x00 by default
//RF data
#define local_ID                    (*(uint8_T *)(swrite + 17)) // 17
#define local_X                     (*(float *)(swrite + 18)) //17 18 19 20 + 1
#define local_Y                     (*(float *)(swrite + 22)) //21 22 23 24 +1
#define local_Z                     (*(float *)(swrite + 26)) //25 26 27 28 +1
#define local_DX                    (*(float *)(swrite + 30)) //29 30 31 32 +1
#define local_DY                    (*(float *)(swrite + 34)) //33 34 35 36 +1
#define local_DZ                    (*(float *)(swrite + 38)) //37 38 39 40 +1

#define local_CS                   (*(uint8_T *)(swrite + 42)) //42

#define local_SUMCHECK_LENGTH       39  // from the Frame type 3 to local_DZ 40
#define local_length                0x27

//define of the read port
#define receive_MSG_LENGTH         41
// don have the Frame ID and the broadcast_r    O2H -> out 2 here
static uint8_T sread[receive_MSG_LENGTH];
static uint8_T sread_bak[receive_MSG_LENGTH];
#define O2H_SD          		    (*(uint8_T *)(sread + 0)) //0x7E
#define O2H_Length1                 (*(uint8_T *)(sread + 1)) // 1.2   0xXX XX total bytes - 4 (most likely, start delimiter(1), length(2) and checksum(1) ),  40 (add the 0) - 4=36 = 0x0024
#define O2H_Length2                 (*(uint8_T *)(sread + 2))
#define O2H_FrameType           	(*(uint8_T *)(sread + 3)) // 3  0x90 Transmit Request
#define O2H_64Addr1             	(*(uint32_T *)(sread + 4)) //4-7
#define O2H_64Addr2                 (*(uint32_T *)(sread + 8)) // 8-11
#define O2H_16Addr                  (*(uint16_T *)(sread + 12)) // 12,13 0xFF FE
#define O2H_Options             	(*(uint8_T *)(sread + 14)) //14 0xC1
//RF data
#define O2H_ID                      (*(uint8_T *)(sread + 15)) // 15
#define O2H_X                       (*(float *)(sread + 16)) //15 16 17 18 + 1
#define O2H_Y                       (*(float *)(sread + 20)) //19 20 21 22 +1
#define O2H_Z                       (*(float *)(sread + 24)) //23 24 25 26 +1
#define O2H_DX                      (*(float *)(sread + 28)) //27 28 29 30 +1
#define O2H_DY                      (*(float *)(sread + 32)) //31 32 33 34 +1
#define O2H_DZ                      (*(float *)(sread + 36)) //35 36 37 38 +1

#define O2H_CS                      (*(uint8_T *)(sread + 40)) //40

#define O2H_SUMCHECK_LENGTH       37  // from the Frame type 3 to local_DZ 38

using namespace std;

serial::Serial fd;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "serialdemo");
    ros::NodeHandle serialdemo_hdlr("~");
    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);



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
    // get param NodeNo, freq, timeout 
    int NodeNo_read;
    if (serialdemo_hdlr.getParam("nodeNo",NodeNo_read )){
        printf(KBLU"Retrived NodeNo = %d for param Node number\n"RESET, NodeNo_read );
        NodeNo = NodeNo_read;
    }else{
        printf(KRED"can't retrive the NodeNo, please try again.\n"RESET);
        exit (EXIT_FAILURE);
    }
    int rate_read;
    if (serialdemo_hdlr.getParam("rate",rate_read )){
        printf(KBLU"Retrived rate = %d for param single_loop_rate\n"RESET, rate_read );
        single_loop_rate = rate_read;
    }else{
        printf(KRED"can't retrive the rate, please try again.\n"RESET);
        exit (EXIT_FAILURE);
    }
    double timeout_read;
    if (serialdemo_hdlr.getParam("timeout",timeout_read )){
        printf(KBLU"Retrived timeout_read = %f for param receive_time_out\n"RESET, timeout_read );
        receive_time_out = timeout_read;
    }else{
        printf(KRED"can't retrive the timeout_read, please try again.\n"RESET);
        exit (EXIT_FAILURE);
    }
    ros::Rate loop_rate(single_loop_rate); // previous it is 40
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

    int loop_count = 0;
    double receive_timer = 0;


    //variables define
    int que_ID = loop_minus(local_ID);
    O2H_ID = NodeNo - 1; // default for ID = 0 to work, need to receive NodeNo - 1
     if ( O2H_ID == que_ID  )
    {
         flag_timeout_timer = 1;
     }
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

        local_SD = 0x7E;
        local_Length1 = 0x00;
        local_Length2 = local_length;
        local_FrameType = 0x10;
        local_FrameID = 0x00; // no feed back to the sender
        local_64Addr1 = 0x00000000; // broadcast to all Mac address
        local_64Addr2 = 0xFFFF0000; // remenber to swap.
        local_16Addr = 0xFEFF;       // remember to swap.
        local_Broadcast_r = 0x00;
        local_Options = 0x00;
        local_X = x;
        local_Y = y;
        local_Z = z;
        local_DX = x;
        local_DY = y;
        local_DZ = z;
        unsigned char CSA = 0;
        for (unsigned char i = 0; i < local_SUMCHECK_LENGTH; i++)
        {
            CSA = CSA + swrite[3+i]; //3 - 40
        }
        local_CS = 0xFF - CSA;
        //***************************end of Info Update


        //Check buffer
        if (fd.available() >= local_MSG_LENGTH) // >= enough bytes
        {
            //read
            //printf("test1, in avail()");
            if (receving_message()) //only when message is successfully
            {
                //printf("test2, in receive");
                //check ID
                if ( (O2H_ID == que_ID)  )//receive the prece ID
                {
                    fd.write(swrite, local_MSG_LENGTH);
                    print_sending_msg();
                    for (int i = 0; i < local_MSG_LENGTH; i++)
                    {
                        printf("%02x ", swrite[i]);
                    }
                    //flag time_out on
                    flag_timeout_timer = 1;
                    receive_timer = ros::Time::now().toSec();
                }
                else if(O2H_ID == loop_add(local_ID)) // received next ID
                {
                    //flag time_out close
                    flag_timeout_timer = 0;
                }
                else
                {
                    printf(KRED"incorrect ID received \n"RESET);
                }

            }

        }
        else if(flag_timeout_timer) //check time out when not enough data in buffer
        {
            if ((ros::Time::now().toSec() - receive_timer) > receive_time_out)
            { // if timeout, resend, and reset timeout

                printf(KRED"time_out\n"RESET);
                //send
                fd.write(swrite, local_MSG_LENGTH);
                print_sending_msg();
                for (int i = 0; i < local_MSG_LENGTH; i++)
                {
                    printf("%02x ", swrite[i]);
                }

                flag_timeout_timer = 1;
                receive_timer = ros::Time::now().toSec();
            }
        }

        loop_count++;

        printf ("package loss numebr: %d, \n", package_loss_nu );
        printf ("loops: %d, \n",loop_count);
        loop_rate.sleep();
    }
    return 0;
}

int receving_message(){
    //sync
    // already have enough bytes
    while(!synced)
    {
        fd.read((uint8_T *)(sread + 0), 1);
        //try sync
        if (O2H_SD == 0x7E)    // SD = 7E
        {
            // back up the previous data
            memcpy(sread_bak,sread, local_MSG_LENGTH );// dest: sread_bak, source: sread
            fd.read((uint8_T *)(sread + 1),receive_MSG_LENGTH - 1 );
            unsigned char CSA = 0;
            for (unsigned char i = 0; i < O2H_SUMCHECK_LENGTH; i++)
            {
                CSA = CSA + sread[3+i]; //3 - 38
            }
            CSA = 0xFF - CSA;
            printf (KYEL"the CSA = %d , the recevied: CS1 = %d"RESET, CSA, O2H_CS );
            if (CSA == O2H_CS){
                synced = true;
                print_sread();// two of this
                printf (KGRN"synced\n"RESET);
                return 1;
            }else{ // failed, by default, synced = false, restore the memory, return 0 indicate unsuccessful
                memcpy(sread ,sread_bak , receive_MSG_LENGTH );
                return 0;
            }

        }//SD
        //after trying to sync, if buffer become empty
        if (fd.available() == 0)
        { // return 0 indicate unsuccessful receive
            return 0;
        }
    }
    if (synced)
    {
        memcpy(sread_bak,sread, local_MSG_LENGTH ); //back up sread to sread_bak
        fd.read(sread, local_MSG_LENGTH);
        unsigned char CSA = 0;
        for (unsigned char i = 0; i < O2H_SUMCHECK_LENGTH; i++)
        {
            CSA = CSA + sread[3+i]; //3 - 38
        }
        CSA = 0xFF - CSA;
        printf (KYEL"the CSA = %d , the recevied: CS1 = %d\n"RESET, CSA, O2H_CS );
        if (CSA == O2H_CS && O2H_SD == 0x7E){ // check both 2 directly
            // if correct, updated, return to main function and see if ID match the que
            printf(KGRN"Matched\n"RESET);
            print_sread();//two of this in receive
            return 1;
        }
        else
        { // 1. restore, 2.package loss count, 3. synced turn off 4. return 0 to indicate the unsuccessful receive
            printf("lost pakage, try sync again \n");
             memcpy(sread ,sread_bak , receive_MSG_LENGTH  );
            package_loss_nu ++;
            synced = false;
            return 0;

        }
    }

}

void print_sending_msg()
{ // used for print msg for sending or local info
    printf(KBLU"the sending message: ---------------------\n");
    printf ("Header = %c\n", local_SD);
    printf ("ID = %d\n", local_ID);
    printf ("x  = %f, y  = %f, z  = %f\n", local_X, local_Y, local_Z);
    printf ("Dx = %f, Dy = %f, Dz = %f\n", local_DX, local_DY, local_DZ);
    printf ("CS = %d\n"RESET, local_CS);
}

void print_sread()
{ //used for print readed msg from others
    printf ("the received message: ************************\n");
    printf ("Header = %c\n", O2H_SD);

    printf ("ID = %d\n", O2H_ID);

    printf ("x  = %f, y  = %f, z  = %f\n", O2H_X, O2H_Y, O2H_Z);
    printf ("Dx = %f, Dy = %f, Dz = %f\n", O2H_DX, O2H_DY, O2H_DZ);


    printf ("CS = %d\n"RESET, O2H_CS);
}


int loop_add(int a)
{ // loop inside 0,1,2,3... NodeNo -1
    if (a < 0 || a > (NodeNo -1)){
        printf(KRED"use a invalid input in the function loop_add"RESET);
    }
    if (a == (NodeNo -1)){
        return 0;
    }else{
        return a + 1;
    }
}
int loop_minus(int a)
{ // loop inside 0,1,2,3 .... NodeNo -1
    if (a < 0 || a > (NodeNo -1)){
        printf(KRED"use a invalid input in the function small_add"RESET);
    }
    if (a == 0){
        return (NodeNo -1 );
    }else{
        return a - 1;
    }
}




