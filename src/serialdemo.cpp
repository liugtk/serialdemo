#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <math.h>
#include <cstring>
#include <ctime>
#include <fstream>

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

struct LOG_SET {
    int package_loss_nu;
    int suc_receive_nu;
    int fail_receive_nu;
    int skip_nu;

} Log;

using namespace std;
typedef unsigned char uint8_T;
typedef unsigned short uint16_T;  // NOLINT
typedef unsigned int uint32_T;  // NOLINT





static bool synced = 0;
int receving_message();

//********************Frame's initials
#define local_MSG_LENGTH            42

//define of read part
static uint8_T swrite[local_MSG_LENGTH];



#define local_SD          		   (*(uint8_T *)(swrite + 0)) //0x7E
#define local_Length1               (*(uint8_T *)(swrite + 1)) // 1.2   0xXX XX total bytes - 4 (most likely, start delimiter(1), length(2) and checksum(1) ),  42 (add the 0) - 4=38 = 0x26
#define local_Length2               (*(uint8_T *)(swrite + 2))
#define local_FrameType          	(*(uint8_T *)(swrite + 3)) // 3  0x10 Transmit Request
#define local_FrameID				(*(uint8_T *)(swrite + 4)) // 4 0x00  or other number use 00 means no response

#define local_64Addr1				(*(uint32_T *)(swrite + 5)) //5678 9abc 5 to 8
#define local_64Addr2               (*(uint32_T *)(swrite + 9)) //5678 9abc 9 to 12
#define local_16Addr				(*(uint16_T *)(swrite + 13)) // 13,14 0xFF FE
#define local_Broadcast_r			(*(uint8_T *)(swrite + 15)) // 15, 0x00 00 max hop times
#define local_Options				(*(uint8_T *)(swrite + 16)) //16 0x00 by default
//RF data

#define local_X                     (*(float *)(swrite + 17)) //17 18 19 20
#define local_Y                     (*(float *)(swrite + 21)) //21 22 23 24
#define local_Z                     (*(float *)(swrite + 25)) //25 26 27 28
#define local_DX                    (*(float *)(swrite + 29)) //29 30 31 32
#define local_DY                    (*(float *)(swrite + 33)) //33 34 35 36
#define local_DZ                    (*(float *)(swrite + 37)) //37 38 39 40

#define local_CS                   (*(uint8_T *)(swrite + 41)) //41

#define local_SUMCHECK_LENGTH       38  // from the Frame type 3 to local_DZ 40

//define of the read port
#define receive_MSG_LENGTH         40
// don have the Frame ID and the broadcast_r
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

#define O2H_X                       (*(float *)(sread + 15)) //15 16 17 18
#define O2H_Y                       (*(float *)(sread + 19)) //19 20 21 22
#define O2H_Z                       (*(float *)(sread + 23)) //23 24 25 26
#define O2H_DX                      (*(float *)(sread + 27)) //27 28 29 30
#define O2H_DY                      (*(float *)(sread + 31)) //31 32 33 34
#define O2H_DZ                      (*(float *)(sread + 35)) //35 36 37 38

#define O2H_CS                      (*(uint8_T *)(sread + 39)) //39

#define O2H_SUMCHECK_LENGTH       36  // from the Frame type 3 to local_DZ 38


serial::Serial fd;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "serialdemo");
    ros::NodeHandle serialdemo_hdlr("~");

    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(20); // previous it is 40
    /*p

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
*/

    ofstream myfile ("Logfile.txt");
    if (!myfile.is_open())
        printf(KRED"Unable to open file"RESET);


    string fccSerialPort = string("/dev/ttyUSB0");
    //check for the FCC pot and the ID for the UAV
    if(serialdemo_hdlr.getParam("fccSerialPort", fccSerialPort))
        printf(KBLU"Retrieved value %s for param 'fccSerialPort'!\n"RESET, fccSerialPort.data());
    else
        printf(KRED "Couldn't retrieve param 'fccSerialPort', using default port %s!\n"RESET, fccSerialPort.data());
    int mac_addr1_read;
    if (serialdemo_hdlr.getParam("MAC_Addr1", mac_addr1_read))
    {
        printf(KBLU"Retrived mac_addr1_read = %08x for param MAC_Addr1\n"RESET, mac_addr1_read );
        local_64Addr1 = mac_addr1_read;
    }
    else
    {
        printf(KRED"can't retrive the mac_addr1_read, please try again.\n"RESET);
        exit (EXIT_FAILURE);
    }
    int mac_addr2_read;
    if (serialdemo_hdlr.getParam("MAC_Addr2", mac_addr2_read))
    {
        printf(KBLU"Retrived mac_addr2_read = %08x for param MAC_Addr2\n"RESET, mac_addr2_read );
        local_64Addr2 = mac_addr2_read;
    }
    else
    {
        printf(KRED"can't retrive the mac_addr2_read, please try again.\n"RESET);
        exit (EXIT_FAILURE);
    }

    // need to get the MAC address for destination

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
    /*
    //CREATE TIMER;
    double startTime = (double)ros::Time::now().toSec();
    double Time_loop = startTime;
    double Time_loop2 = startTime;
    int loop_count = 0;
    double receive_timer = 0;


    //variables define
    int que_ID = loop_minus(local_ID);
    O2H_ID = NodeNo - 1; // default for ID = 0 to work, need to receive NodeNo - 1
     if ( O2H_ID == que_ID  )
    {
         flag_timeout_timer = 1;
     }
     */

    //CREATE TIMER;
    double startTime = (double)ros::Time::now().toSec();
    double Time_loop = startTime;
    while (ros::ok())
    {
        /*
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
        //***************************end of Info Update


        //Check buffer
        if (fd.available() >= local_MSG_LENGTH) // >= enough bytes
        {
            //read
            if (receving_message()) //only when message is successfully
            {
                //check ID
                if ( (O2H_ID == que_ID)  )//receive the prece ID
                {
                    fd.write(swrite, local_MSG_LENGTH);
                    print_sending_msg();
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
                //flag time_out on
                flag_timeout_timer = 1;
                receive_timer = ros::Time::now().toSec();
            }
        }

        loop_count++;

        printf ("package loss numebr: %d, \n",package_loss_nu );
        printf ("loops: %d, \n",loop_count);
        loop_rate.sleep();
*/

        double time = ros::Time::now().toSec(); // counting the time
        printf(KMAG"time used = %f \n"RESET, time - Time_loop );
        Time_loop = time;
        float x = 10.0;
        float y = 20.0;
        float z = 30.0;
        local_SD = 0x7E;
        local_Length1 = 0x00;
        local_Length2 = 0x26;
        local_FrameType = 0x10;
        local_FrameID = 0x00;
        //local_64Addr1 = 0x00A21300; // remember to swap,  0013A20040A1C930
        //local_64Addr2 = 0x30C9A140; // remenber to swap.
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
        printf ("********sending********** \n");

        for (int i = 0; i < local_MSG_LENGTH; i++)
        {
            printf("%02x ", swrite[i]);
        }
        printf("\n");
        fd.write(swrite, local_MSG_LENGTH);


        if (fd.available() >= receive_MSG_LENGTH) // >= enough bytes
        {
            //read
            if (receving_message()) //only when message is successfully
            {
                printf("good\n");
                Log.suc_receive_nu ++;
            }
            else
            {
                printf("bad\n");
                Log.fail_receive_nu++;
            }
        }
        else
        {
           printf("No enough bytes in buffer\n");
           //if (Log.suc_receive_nu && Log.suc_receive_nu <= 1000)
           if (Log.suc_receive_nu)
           {
               Log.skip_nu ++;
           }
        }
        printf(KRED"good: %d, bad: %d, skip: %d\n"RESET,Log.suc_receive_nu,Log.fail_receive_nu, Log.skip_nu);

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
            memcpy(sread_bak,sread, receive_MSG_LENGTH );// dest: sread_bak, source: sread
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
                //print_sread();// two of this in receive
                printf (KGRN"synced\n"RESET);
                return 1;
            }else{ // failed, by default, synced = false, restore the memory, return -1 indicate unsuccessful
                memcpy(sread ,sread_bak , receive_MSG_LENGTH );
                return 0;
            }

        }//U
        //after trying to sync, if buffer become empty
        if (fd.available() == 0)
        { // return -1 indicate unsuccessful receive
            return 0;
        }
    }
    if (synced)
    {
        memcpy(sread_bak,sread, receive_MSG_LENGTH ); //back up sread to sread_bak
        fd.read(sread, receive_MSG_LENGTH);
        unsigned char CSA = 0;
        for (unsigned char i = 0; i < O2H_SUMCHECK_LENGTH; i++)
        {
            CSA = CSA + sread[3+i]; //3 - 38
        }
        CSA = 0xFF - CSA;
        printf (KYEL"the CSA = %d , the recevied: CS1 = %d"RESET, CSA, O2H_CS );
        if (CSA == O2H_CS && O2H_SD == 0x7E){ // check all 4 directly
            // if correct, updated, return to main function and see if ID match the que
            printf(KGRN"Matched\n"RESET);
            //print_sread();//two of this in receive
            return 1;
        }
        else
        { // 1. restore, 2.package loss count, 3. synced turn off 4. return -1 to indicate the unsuccessful receive
            printf("lost pakage, try sync again \n");
            memcpy(sread ,sread_bak , receive_MSG_LENGTH  );
            Log.package_loss_nu ++;
            synced = false;
            return 0;

        }
    }

}
/*
void print_sending_msg()
{ // used for print msg for sending or local info
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

void print_sread()
{ //used for print readed msg from others
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

*/


