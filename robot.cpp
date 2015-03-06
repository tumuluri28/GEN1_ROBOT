/* mbed ROBOT Library, for SUTD evobot project, Generation 1
 * Copyright (c) 2013, SUTD
 * Author: Mark VanderMeulen
 *
 * Dec 18, 2013
 *
 * You may want to rewrite this entire library, as this one is only half-finished
 *
 * This library allows the user to use each module on the Evobot Generation 1
 * Functionality:
 *      -Bluetooth connection
 *      -MPU (accelerometer/gyroscope) connection
 *      -Orientation in the XY plane (using Z-axis gyroscope only)
 *      -Remote control functionality (via bluetooth)
 *
 * Future functions:
 *      -Switch to turn bluetooth on/off
 *      -Switch to turn MPU on/off (in case of connection errors with the MPU: restart and reconnect
 *      -Calculation of distance travelled (use integration of accelerometer, maybe use Kalman filter)
 *      -Access to the DMP (Digital motion processor) on the MPU to calculate 3D orientation.
 *      -RF mesh network so robots can connect to eachother
 *      -Camera (requires an add-on to the circuit board at the moment)
 */


#include "robot.h"

//#define MOT_STBY_PIN 'PTA15'

//*********************************CONSTRUCTOR*********************************//
Robot::Robot() : bt(tx_bt,rx_bt),
                 rf24(PTD2, PTD3, PTD1, PTD0, PTD5, PTD4),
                 mpu(PTE0, PTE1),
                 myled(LED),
                 btSwitch(PTE25),
                 currentSensor(CURRENTSENSOR_PIN),
                 IR_pin(IRsense),
                 IR_pin2(IRsense2),
                 IR_pin3(IRsense3),
                 voltageSensor(VOLTAGESENSOR_PIN),
                 PWMA(MOT_PWMA_PIN),
                 PWMB(MOT_PWMB_PIN),
                 AIN1(MOT_AIN1_PIN),
                 AIN2(MOT_AIN2_PIN),
                 BIN1(MOT_BIN1_PIN),
                 BIN2(MOT_BIN2_PIN),
                 STBY(MOT_STBY_PIN){
               
   stop();  //Initialize motors as stopped//
   
   btSwitch = 1;    //turn on bluetooth
   
   myled = 0;   //turn ON status LED (0 = on, 1 = 0ff)
   timeNext = 0;
   
   currentAvg = 0;          /////////////REMOVE currentAvg LATER. TESTING PURPOSES ONLY////////
   
   target_angle = 0;    //direction we want the robot to be pointed
   angle_origin = 0;    //you can use this to modify the angle of origin
   origin = 0;          //the (x,y) location of the origin (this should be a point, not a double)
   rz = 0;          //The current rotation in the Z-axis
   dx = 0;          //The current displacement in the x-axis    (side-side)
   dy = 0;          //The current displacement in the y-axis    (forward-back)
   dz = 0;          //The current displacement in the z-axis    (up-down)
   
   AUTO_ORIENT = 1;     //robot will automatically orient iteslf using the gyroscope
   REMOTE_CONTROL = 1;  //robot can be controlled over bluetooth
   
   t.start();   //start timer
   mpu.testConnection();
   wait(1);
   MPU_OK = 0;
   
   //initialize MPU
   if (mpu.testConnection()){
        mpu.setBW(MPU6050_BW_10);   //default set low pass filter bandwidth to 10HZ
        mpu.setGyroRange(MPU6050_GYRO_RANGE_500); //default set the gyro range to 500deg/s (robot turning exceeds 250deg/s)
        mpu.setAcceleroRange(MPU6050_ACCELERO_RANGE_2G);        //default set accelero range to 2g
        MPU_OK=1;
        myled = 0;  //turn on LED
        calibrate();
   }
   else if(0)   //this section is current disabled. change to (1) if you want to retry the accelerometer connection.
   {
        myled = 1;  //turn off LED
        for (int i = 0; i<25; i++)
        {
            myled = !myled;
            wait_ms(50);
            if (mpu.testConnection()){
                i = 25;
                myled = 0;
                MPU_OK=1;
            }
            else
                myled = 1;
            wait_ms(50);
        }
   }
   myled = MPU_OK;  //If LED is off, it is ok. If LED is on, there was a MPU error. Board needs to be restarted.
   //In the future, at a transistor to switch on/off the mpu in case there is an MPU error. Right now it needs a manual restart.
   
   bt.baud(BaudRate_bt);    //Set bluetooth baud rate
   
   //Initialize RF chip
//char txData[TRANSFER_SIZE], rxData[TRANSFER_SIZE];
// int txDataCnt = 0;
// int rxDataCnt = 0;
   rf24.powerUp();
   rf24.setTransferSize( TRANSFER_SIZE );

   rf24.setReceiveMode();
   rf24.enable();
   
   
   
}

//*********************************MOTORS*********************************//
void Robot::motor_control(int Lspeed, int Rspeed){
    //Controls the motors. 0 = stopped, 100 = full speed, -100 = reverse speed
    if (!Lspeed && !Rspeed)     //stop//
        STBY = 0;
    else
        STBY = 1;
    
    //make sure 'speeds' are between -100 and 100, and not between abs(0 to 10)
    if(Lspeed > 100) Lspeed = 100;
    else if (Lspeed < -100) Lspeed = -100;
    else if (Lspeed < 0 && Lspeed > -15)   Lspeed = -15;    //prevent speed from being less than 15
    else if (Lspeed > 0 &&  Lspeed < 15)    Lspeed = 15;
    if(Rspeed > 100) Rspeed = 100;
    else if (Rspeed < -100) Rspeed = -100;
    else if (Rspeed < 0 && Rspeed > -15)   Rspeed = -15;
    else if (Rspeed > 0 &&  Rspeed < 15)    Rspeed = 15;
    
    
    if (!Rspeed){   //if right motor is stopped
        AIN1 = 0;
        AIN2 = 0;
        PWMA = 0;
    }
    else if (!Lspeed){   //if left motor is stopped
        BIN1 = 0;
        BIN2 = 0;
        PWMB = 0;
    }
    //RIGHT MOTOR//
    if(Rspeed > 0){      //Right motor fwd
        AIN1 = MOTOR_R_DIRECTION;   //set the motor direction
        AIN2 = !AIN1;
    }
    else{        //Right motor reverse
        AIN2 = MOTOR_R_DIRECTION;
        AIN1 = !AIN2;
    }
    PWMA = abs(Rspeed/100.0);
    //LEFT MOTOR//
    if(Lspeed >0){
        BIN1 = MOTOR_L_DIRECTION;
        BIN2 = !BIN1;
    }
    else{
        BIN2 = MOTOR_L_DIRECTION;
        BIN1 = !BIN2;
    }
    PWMB = abs(Lspeed/100.0);
}

void Robot::stop(){
    motor_control(0,0);
}
void Robot::set_speed(int Speed){
    speed = Speed;
    motor_control(speed,speed);
}

//************************Setting direction of robot****************************//
void Robot::set_direction(double angle){
    //Set the direction the robot should be facing from origin(in radians)
    target_angle = angle;
}
void Robot::set_direction_deg(double angle){
    //Set the direction the robot should be facing from origin (in degrees)
    target_angle = angle*M_PI/180;
}
void Robot::auto_enable(bool x){
    AUTO_ORIENT = x;
}

//************************UPDATE: To be called in main loop***************************//
    //This calculates position and angle
    //Also orients robot in the correct direction
    //if 'print' is 1, print acceleration and gyro data over bluetooth
void Robot::update(int print){   
    double SP;
    int rotationSpeed;
//    double avCurrent = 0;
    int n=50;
    
    double Kp = 100; //weighting values for PID controller. Only Proportional component is used.
    double Ki = 0;  //Integral component is not used
    double Kd = 0;  //derivative component is not used
    
    if(MPU_OK){ //only do this if MPU is connected
        mpu.getAcceleroRaw(accdata);
        mpu.getGyroRaw(gyrodata);
        time = t.read();
        
        if(print==1){  //print accel and gyro data to screen
            bt.printf("%f\t%d\t%d\t%d\t", time,accdata[0], accdata[1], accdata[2]);
            bt.printf("%d\t%d\t%d\t%d\n\r", gyrodata[0], gyrodata[1], gyrodata[2]-gyroOffset[2],STBY.read());
        }else if(print==2){     //print voltage and current to screen
            bt.printf("Voltage= %f V, \tCurrent = %f mA \t average = %f mA\n\r",getVoltage(),getCurrent(),currentAvg);
            
            currentAvg += (getCurrent() - currentAvg)/n;
        }
        
        if(REMOTE_CONTROL){     //if remote control over bluetooth is enabled
            remote_ctrl();      //call the romote control function
        }   
        if(AUTO_ORIENT){        //enable PID control of angle
            SP = target_angle;  //desired angle (radians)
            rz = rz + ((gyrodata[2]-gyroOffset[2])*(time-timePrev)/gyroCorrect);    //rz is the rotation(radians) from start
            Irz = Irz + (rz-SP)*(time-timePrev);    //Irz needs to be reset every so often, or it should be ignored
            rotationSpeed = (Kp*(rz-SP) + Ki*Irz + Kd*gyrodata[2]/gyroCorrect);
            
            //TODO: pull "rotationspeed" up to 10 if it is less than 10. This will(should) improve the ability to drive straight
            
            if(time > timeNext || (speed==0 && rotationSpeed ==0)){    //prevent the motor control from being set too often
                timeNext = time + MOTOR_INTERVAL/1000.0;   //only set the motor speed every 10ms
                motor_control(speed+rotationSpeed, speed-rotationSpeed);    //Set motor speeds
            }
        }    
        
        
        timePrev = time;
    }
}

void Robot::remote_ctrl(){
    //This is a private function
    char c;
    if(bt.readable())     //if something can be read from bluetooth
    {
        Irz = 0;
        c = bt.getc();  //read a character from the bluetooth
        switch (c)
        {
            case ctrl_forward:
                speed = 100;
                break;
            case ctrl_backward:
                speed = -100;
                break;
            case ctrl_right:
                target_angle += 90*M_PI/180;
                break;
            case ctrl_left:
                target_angle -= 90*M_PI/180;
                break;
            case ctrl_calibrate:
                calibrate();
                break;
            default:
                speed = 0;
                stop();
                break;
        }
        
    }
}

//*********************************CALIBRATE*********************************//
void Robot::calibrate(){
    stop();
    wait(1.5);
    double timeNOW = t.read();
    int count=0;
    int i;
    
        //set the accelerometer and GYRO offsets
        while(t.read()<timeNOW+1.5) //calculate gyro offset
        {
            mpu.getGyroRaw(gyrodata);
            mpu.getAcceleroRaw(accdata);
            for(i=0;i<3;i++)
            {
                gyroOffset[i] += gyrodata[i];
                accOffset[i] += accdata[i];     
            }
            count++;
        }
        for(i=0;i<3;i++)
        {
            gyroOffset[i] = gyroOffset[i]/count; //rxOffset
            accOffset[i] = accOffset[i]/count;
        }
        accOffset[2] = 0;   //we don't want to remove GRAVITY from the z-axis accelerometer.
}

//*********************************ROBOT-SENSORS*********************************//

double Robot::getCurrent(){

    double Vsensor = currentSensor.read();
    //Vsensor=Vsensor*3.3;
    //Vsensor = (VREF3_3*(CURRENT_R1+CURRENT_R2)/CURRENT_R2*CURRENT_R4/(CURRENT_R4+CURRENT_R3)-Vsensor*VREF3_3)
    //        *CURRENT_R2/CURRENT_R1;
    //The above math did not work because the ACS712 did not give an output that is 185mV/A. It gave something else.
    Vsensor = 1000*((.8998-Vsensor)/.6411);
    return Vsensor;//(Vsensor-VREF3_3*0.5)/0.000185;
}
double Robot::getVoltage()
{
    float voltage = 3.3*(voltageSensor.read()); // convert analog value to voltage at node
    voltage *= 1.5; // inverse of voltage divider
    //bt.printf("%lf, %f, 0\n\r", t.read(), voltage);
    return voltage;
}
    
//******************RF24 CHIP FUNCTIONS****************************//
void Robot::rf24_power(int x){
    if(x)   //power up
        rf24.powerUp();
    else    //power down
        rf24.powerDown();
}

double Robot::IRsensor(){
    double reading = IR_pin.read();
    return reading;
    
    }
    
double Robot::IRsensor2()
{
    double reading2 = IR_pin2.read();
    return reading2;
    }
    
double Robot::IRsensor3()
{
    double reading3 = IR_pin3.read();
    return reading3;
    }
