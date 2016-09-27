
#include "I2Cdev.h"
#include <Servo.h>
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif



MPU6050 mpu;



#define OUTPUT_READABLE_YAWPITCHROLL



#define ROLL_PID_MIN  -20.0
#define ROLL_PID_MAX  20.0
#define PITCH_PID_MIN  -20.0
#define PITCH_PID_MAX  20.0
#define YAW_PID_MIN  -10.0
#define YAW_PID_MAX  10.0


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)




bool blinkState = false;




bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer





Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector






uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };




volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high


void dmpDataReady() 
{
    mpuInterrupt = true;
}


 
double yaw, pitch, roll ;
double mean_y,mean_p,mean_r ;
double buff_y=0,buff_p=0,buff_r=0 ;

void setup() 
{


    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif


    Serial.begin(9600);
    
//  pinMode(7, OUTPUT);
//  pinMode(8, OUTPUT);
//  pinMode(9, OUTPUT);
//  pinMode(10, OUTPUT);
    
    //while (!Serial); 

    
    
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();



    //Serial.println(F("Testing device connections..."));


    
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


    
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    
    

    
    
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();


    
    
    if (devStatus == 0) 
    {
        
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

    
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        
        packetSize = mpu.dmpGetFIFOPacketSize();

        //Serial.print("PacketSize : ") ;
        //Serial.println(packetSize) ;
    } else 
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }

    
    pinMode(LED_PIN, OUTPUT);



    calib() ;
    motors_initialize();
    pid_initialize();
    
    motors_arm();
}




int incomingByte;
int arm=0;
double pitchs=0,yaws=0,rolls=0,thrusts=55;
void setpoints_update()
{
 

  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();
   
    if (incomingByte == 'w')
    {pitchs=pitchs+5;}
    else if (incomingByte == 's')
    {pitchs=pitchs-5;}
    
     
    else if (incomingByte == 'e')
    {rolls=rolls+5;}
    else if (incomingByte == 'd')
    {rolls=rolls-5;}
    else if (incomingByte == 'r')
    {thrusts=thrusts+5;}
    else if (incomingByte == 'f')
    {thrusts=thrusts-5;}
     else if (incomingByte == 't')
    {yaws=yaws+5;}
    else if (incomingByte == 'g')
    {yaws=yaws-5;}
    else if (incomingByte == 'o')
    arm=1;
    else if (incomingByte == 'p')
    arm=0;
    if (rolls>76)
    rolls=75;
    else if (rolls<-76)
    rolls=-75;
    
     if (pitchs>76)
    pitchs=75;
    else if (pitchs<-76)
    pitchs=-75;
            if (thrusts>91)
    thrusts=90;
    else if (thrusts<54)
    thrusts=55;
  }
}




int m0, m1, m2, m3;
Servo motor0;
Servo motor1;
Servo motor2;
Servo motor3;

void motors_initialize(){
  motor0.attach(6);
  motor1.attach(9);
  motor2.attach(10);
  motor3.attach(11);
  motor0.write(20);
  motor1.write(20);
  motor2.write(20);
  motor3.write(20);
}

void motors_arm(){
  motor0.write(20);
  motor1.write(20);
  motor2.write(20);
  motor3.write(20);
}

void update_motors(int m0, int m1, int m2, int m3)
{
  motor0.write(m0);
  motor1.write(m1);
  motor2.write(m2);
  motor3.write(m3);
}


double pid_roll_in,   pid_roll_out;
double pid_pitch_in,  pid_pitch_out;
double pid_yaw_in,    pid_yaw_out;

PID roll_controller(&pid_roll_in,   &pid_roll_out,  &rolls,  1.0, 0.0, 0.0, REVERSE);
PID pitch_controller(&pid_pitch_in, &pid_pitch_out, &pitchs, 1.0, 0.0, 0.0, REVERSE);
PID yaw_controller(&pid_yaw_in,     &pid_yaw_out,   &yaws,   0.2, 0.0, 0.0, DIRECT); 


void pid_initialize() {
  roll_controller.SetOutputLimits(ROLL_PID_MIN,ROLL_PID_MAX);
  pitch_controller.SetOutputLimits(PITCH_PID_MIN,PITCH_PID_MAX);
  yaw_controller.SetOutputLimits(YAW_PID_MIN,YAW_PID_MAX);
  roll_controller.SetMode(AUTOMATIC);
  pitch_controller.SetMode(AUTOMATIC);
  yaw_controller.SetMode(AUTOMATIC);
  roll_controller.SetSampleTime(10);
  pitch_controller.SetSampleTime(10);
  yaw_controller.SetSampleTime(10);
}

void pid_update(){
  pid_roll_in = roll;
  pid_pitch_in = pitch;
  pid_yaw_in = yaw; 
}

void pid_compute() {
   roll_controller.Compute();
   pitch_controller.Compute();
   yaw_controller.Compute();
}





void calib()
{

  double error = 9999, iter  = 10, i ;

  mean_y = 0 ;
  mean_p = 0 ;
  mean_r = 0 ;

  buff_y = 0 ;
  buff_p = 0 ;
  buff_r = 0 ;
  
  while(error > 0.50)
  {

    error = 0 ;
    buff_y = 0 ;
    buff_p = 0 ;
    buff_r = 0 ;
    
    for(i=1;i<=iter;i++)
    {
      getValues() ;
      
      buff_y+=yaw/iter ;
      buff_p+=pitch/iter ;
      buff_r+=roll/iter ;

      error+=abs(mean_y-yaw) ;
      error+=abs(mean_p-pitch) ;
      error+=abs(mean_r-roll) ;

      /*      
      //Serial.print(yaw) ;
      //Serial.print("  ") ;
      //Serial.print(pitch) ;
      //Serial.print("  ") ;
      //Serial.print(roll) ;
      //Serial.println("  ") ;
      */
      delay(5) ;
      
    }

    mean_y=buff_y ;
    mean_p=buff_p ;
    mean_r=buff_r ;

    /*
      //Serial.print(mean_y) ;
      //Serial.print("  ") ;
      //Serial.print(mean_p) ;
      //Serial.print("  ") ;
      //Serial.print(mean_r) ;
      //Serial.print("  ") ;
*/
    //Serial.println(error) ;

    
    
  }

  //Serial.println("Calibrated") ;
  
}


void loop()
{
  getValues() ;
   setpoints_update();
   pid_update();
   pid_compute();
   
     m0 = thrusts + pid_pitch_out ;//+ pid_yaw_out;
  m1 = thrusts + pid_roll_out ;//- pid_yaw_out;
  m2 = thrusts - pid_pitch_out ;//+ pid_yaw_out;
  m3 = thrusts - pid_roll_out ;//- pid_yaw_out;
  
  
              if (m0>101)
    m0=100;
    else if (m0<53)
    m0=53;
   
              if (m1>101)
    m1=100;
    else if (m1<53)
    m1=53;
     
              if (m2>101)
    m2=100;
    else if (m2<53)
    m2=53;
     
              if (m3>101)
    m3=100;
    else if (m3<53)
    m3=53;
    
    if (arm==0)
   {     m0=20;
         m1=20;
         m2=20;
         m3=20;
     }
 Serial.print(yaw) ;
 Serial.print("  ") ;
 Serial.print(pitch) ;
 Serial.print("  ") ;
 Serial.print(roll) ;
Serial.println("  ") ;
 Serial.print(m0) ;
 Serial.print("  ") ;
 Serial.print(m1) ;
 Serial.print("  ") ;
 Serial.print(m2) ;
Serial.println("  ") ;
 Serial.print(m3) ;
Serial.println("  ") ;
//    if (pitchs>20)
//  digitalWrite(7,HIGH);
//  else
//  digitalWrite(7,LOW);
//  if (rolls>20)
//  digitalWrite(8,HIGH);
//  else
//  digitalWrite(8,LOW);
//  if (yaw>20)
//  digitalWrite(9,HIGH);
//  else
//  digitalWrite(9,LOW);
  

   
////Serial.print(rolls);
////Serial.println(" ");

  delay(500) ;   
}


void getValues() 
{

    mpu.resetFIFO() ;
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    delay(2) ;
 
    while (!mpuInterrupt && fifoCount < packetSize) {

    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
    
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));  

    } else if (mpuIntStatus & 0x02)
    {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        fifoCount -= packetSize;

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            yaw = ypr[0] * 180/M_PI ;
            
            pitch = ypr[1] * 180/M_PI ;
            
            roll = ypr[2] * 180/M_PI ;

        
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

        
    
    }
    else
    {
      //Serial.println("Update Failed") ;
    }

    delay(10) ;
        


}
