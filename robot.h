#ifndef ROBOT_H
#define ROBOT_H

#include "mbed.h"
#include "MPU6050.h"
#include "nRF24L01P.h"


//#define BaudRate_bt    115200      //Baud rate of bluetooth
#define BaudRate_bt    9600        //Baud rate of 9600
#define tx_bt       PTA2        //Bluetooth connection pins
#define rx_bt       PTA1        //Bluetooth connection pins
#define tx_mpu      PTE0    //MPU connection pins
#define rx_mpu      PTE1    //MPU connection pins
#define mpu_bandwidth MPU6050_BW_20 //set the MPU low pass filter bandwidth to 20hz

#define LED PTE3        //status LED pin
#define CURRENTSENSOR_PIN PTC2
#define VOLTAGESENSOR_PIN PTB0

#define CURRENT_R1 180 //160.0     //values of the current sensor opamp resistors
#define CURRENT_R2 10
#define CURRENT_R3 80
#define CURRENT_R4 84.7
#define VREF3_3 3.3        //digital logic voltage
#define VREF5 5.0       //5v voltage        //NOTE: 5v voltage is consistent when using new batts, but not using old blue batts

#define IRsense PTB1
#define IRsense2 PTB2
#define IRsense3 PTB3

#define MOT_PWMA_PIN   PTA4    //Motor control pins    
#define MOT_PWMB_PIN   PTA5
#define MOT_STBY_PIN   PTA15
#define MOT_AIN1_PIN   PTA14
#define MOT_AIN2_PIN   PTA13
#define MOT_BIN1_PIN    PTA16
#define MOT_BIN2_PIN   PTA17

#define M_PI 3.14159265359  //PI
#define gyroCorrect 3720    //divide raw gyro data by this to get result in RAD/SECOND (if gyroRange is 500 rad/s)

//Correct direction of motors. If number = 1, forward. If number = 0, backwards (for if motors are wired backwards)
#define MOTOR_R_DIRECTION   1       
#define MOTOR_L_DIRECTION   1

#define MOTOR_INTERVAL 20     //defines the interval (in milliseconds) between when motor can be set
                                //NOTE: Can't make this less than 20ms, because that is the PWM frequency

//Key bindings for remote control robot - for the future try to use arrow keys instead of 'asdw'
#define ctrl_forward    'w'         //forward
#define ctrl_backward   's'         //back
#define ctrl_left       'a'         //turn left
#define ctrl_right      'd'         //turn right
#define ctrl_calibrate  'c'         //re-calibrate the accelerometer and gyro

// The nRF24L01+ supports transfers from 1 to 32 bytes, but Sparkfun's
//  "Nordic Serial Interface Board" (http://www.sparkfun.com/products/9019)
//  only handles 4 byte transfers in the ATMega code.
#define TRANSFER_SIZE   4


class Robot{
public:
    /**
    *   Constructor - does nothing atm
    */
    Robot();
    
    
    /**
    *   RF CONNECTION - not connected atm
    */
    
    /**
    *   Bluetooth Connection
    *   bt_Init(); -sets default bluetooth connection
    *   bt_Init(rx,tx); -allows user to set baud rate
    *   user cannot change baud rate because baud rate change is done on the bluetooth firmware
    *   
    *   THE FOLLOWING FUNCTIONS ARE NOT YET IMPLEMENTED::
    *   bt_OFF  -turn bluetooth off
    *   bt_ON   -turn bluetooth on
    */
    //Serial bt();//(int tx, int rx);
    //int bt_OFF();
    //int bt_ON();
    
    /**
    *   MOTOR CONTROLS
    */
    void motor_control(int Lspeed, int Rspeed);  //Input speed for motors. Integer between 0 and 100
    void stop();                           //stop motors
    //void motor_Turn(double angle);                  //Change direction of robot. Between 0 and 360 degrees.
    void set_direction(double angle);            //set angle for the robot to face (from origin)
    void set_direction_deg(double angle);
    //void driveToLocation();                      //drive to specified location (can't implement yet) 
    void set_speed(int Speed );      //set speed for robot to travel at
    void auto_enable(bool x);
    /**
    *   MPU CONTROLS
    */
    
    /**
    *   UPDATE (this must be called repeatedly so the robot will sample accelerometer to find position)
    */
    void update(int print); //print variable decides which values to print
                            //print = 0: nothing
                            //print = 1: Accelerometer and gyro
                            //print = 2: Current and voltage
    
    //  calibrate the gyro and accelerometer  //
    void calibrate();
    
    /**
    *   Status: find the distance, orientation, battery values, etc of the robot
    */
    //void distanceTravelled(double x[3])
    //void orientation(something quaternion? on xy plane?)
    
    double getCurrent();   //Get the current drawn by the robot
    double getCurrent(int n); //get the current, averaged over n samples
    double getVoltage();    //get the battery voltage (ask connor for completed function)
    //double getBatteryPercent(); //get the level of charge of battery (incomplete - ask Erik for advice)
    /**
    *   Delete Robot
    */
    
    //Wireless connections
     Serial bt;  //bluetooth connection
     nRF24L01P rf24;  //RF network connection
     
     //RF24 network functions//
     void rf24_power(int status);   //power on or off the RF network
     char rf24_read();
     int rf24_write(char letter);   //write a letter to RF
     int rf24_write(char* buffer, int length);   //write 
     //int rf24_setID();
     
  /* IR sensor*/
  double IRsensor(); 
  double IRsensor2(); 
  double IRsensor3(); 
 
    //-------------------PRIVATE VARIABLES AND FUNCTIONS-----------------------------------------------//
private: 
    
    double currentAvg;
    
    void remote_ctrl();     //commands for remote control robot
    
    MPU6050 mpu;   //MPU connection
    DigitalOut myled;   //(PTE3) Processor LED (1 = off, 0 = on)
    DigitalOut btSwitch;
    AnalogIn currentSensor;
    AnalogIn IR_pin;
    AnalogIn IR_pin2;
    AnalogIn IR_pin3;
    AnalogIn voltageSensor;
    
    double dx;  //distance travelled in x direction
    double dy;  //distance travelled in y direction
    double dz;  //distance travelled in z direction (zero?)
    double origin;   //location of robot origin (or can be set if robot starting location is known.
    
    int accdata[3];     //data from accelerometer (raw)
    int gyrodata[3];    //data from gyro (raw)
    //double gyroCorrect; //= 3720;  //divide by this to get gyro data in RAD/SECOND. This is above as a #DEFINE
    int gyroOffset[3];  //Correction value for each gyroscope to zero the values.
    int accOffset[3];  //correction value for each accelerometer
    
    /**Angle is always measured in clockwise direction, looking down from the top**/
    /*Angle is measured in RADIANS*/
    double rz;          //Direction robot is facing
    double Irz;         //integral of the rotation offset from target. (Optionally) Used for PID control of direction
    double angle_origin;      //Angle of origin (can be changed later, or set if robot starts at known angle)
    double target_angle;    //direction that we want the robot to face (radians)
    bool AUTO_ORIENT; //if this flag is 1, the robot automatically orients itself to selected direction
    bool REMOTE_CONTROL;    //if this flag is 1, the robot will be controlled over bluetooth
    
    ///////////   Motor control variables   ///////////
    PwmOut PWMA;//(MOT_PWMA_PIN);
    PwmOut PWMB;//(MOT_PWMB_PIN);
    DigitalOut AIN1;//(MOT_AIN1_PIN);
    DigitalOut AIN2;//(MOT_AIN2_PIN);
    DigitalOut BIN1;//(MOT_BIN1_PIN);
    DigitalOut BIN2;//(MOT_BIN2_PIN);
    DigitalOut STBY;//(MOT_STBY_PIN);
    double timeNext;    //next time that the motor is allowed to be updated

    
    bool MPU_OK;
    
    Timer t;    //timer
    
    double time;    //time of current iteration
    double timePrev;    //time of previous iteration
    
    int speed;  //set the speed of robot
    
    
};




#endif