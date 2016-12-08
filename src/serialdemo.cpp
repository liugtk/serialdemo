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
static int single_loop_rate = 20;
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




//********************Frame's initials
#define local_MSG_LENGTH            32
static double receive_time_out  = 32 * 0.002 + 0.01; // 2ms for each byte and 10 ms for extra wait
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
#define O2H_Token                 (*(uint8_T *)(sread + 28))
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
#define local_Token                 (*(uint8_T *)(swrite + 28))
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
        //local_Token = 4;
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
                    local_Token = O2H_Token + 1;
                    unsigned char CSA = 0, CSB = 0;
                    for (unsigned char i = 0; i < local_SUMCHECK_LENGTH; i++)
                    {
                        CSA = CSA + swrite[2+i]; //4 - 31
                        CSB = CSB + CSA; // 32
                    }
                    local_CS1 = CSA;
                    local_CS2 = CSB;
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
                    print_sread();// two of this in receive
                    printf (KGRN"synced\n"RESET);
                    return 1;
                }else{ // failed, by default, synced = false, restore the memory, return 0 indicate unsuccessful
                    memcpy(sread + 2,sread_bak + 2, local_MSG_LENGTH - 2 );
                    return 0;
                }
            }//W
        }//U
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
            print_sread();//two of this in receive
            return 1;
        }
        else
        { // 1. restore, 2.package loss count, 3. synced turn off 4. return 0 to indicate the unsuccessful receive
            printf("lost pakage, try sync again \n");
            memcpy(sread + 2,sread_bak + 2, local_MSG_LENGTH - 2 );
            package_loss_nu ++;
            synced = false;
            return 0;

        }
    }

}

void print_sending_msg()
{ // used for print msg for sending or local info
    printf(KBLU"the sending message: ---------------------\n");
    printf ("Header1 = %c\n", local_HEADER1);
    printf ("Header2 = %c\n", local_HEADER2);
    printf ("ID = %d\n", local_ID);
    printf ("Token = %d\n", local_Token);
    printf ("LENGTH = %d\n",local_LENGTH_BYTE);
    printf ("x  = %f, y  = %f, z  = %f\n", local_X, local_Y, local_Z);
    printf ("Dx = %f, Dy = %f, Dz = %f\n", local_DX, local_DY, local_DZ);
    printf ("loss = %d\n", local_LOSS);
    printf ("CS1 = %d, CS2 = %d\n"RESET, local_CS1, local_CS2);
}

void print_sread()
{ //used for print readed msg from others
    printf ("the received message: ************************\n");
    printf ("Header1 = %c\n", O2H_HEADER1);
    printf ("Header2 = %c\n", O2H_HEADER2);
    printf ("ID = %d\n", O2H_ID);
    printf ("Token = %d\n", O2H_Token);
    printf ("LENGTH = %d\n",O2H_LENGTH_BYTE);
    printf ("x  = %f, y  = %f, z  = %f\n", O2H_X, O2H_Y, O2H_Z);
    printf ("Dx = %f, Dy = %f, Dz = %f\n", O2H_DX, O2H_DY, O2H_DZ);
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




