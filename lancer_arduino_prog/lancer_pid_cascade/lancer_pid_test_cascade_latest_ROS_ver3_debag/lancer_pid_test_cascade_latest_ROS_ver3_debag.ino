#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include<TimerThree.h>
#include<TimerOne.h>
//pin設定-----------------
const int Dir_pin1 = 5;
const int Dir_pin2 = 8;
const int Step_pin1 = 6;
const int Step_pin2 = 9;
//-------------------------

int output1 = 1;
int output2 = 1;

volatile int Timer1_start;
volatile int Timer2_start;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 7  // use pin 2 on Arduino Uno & most boards
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void delay_microsec(){
  __asm__ __volatile__(
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop");
  }

//タイマー割り込み----------------------------
void flash_timer2(){
  if(Timer2_start == -1){return ;}
  if(Timer2_start == 1){
    Timer2_start = 0;
  }else{
  output2 = !output2;
  digitalWrite(Step_pin2, output2);
  //delayMicroseconds(1);
  delay_microsec();
  output2 = !output2;
  digitalWrite(Step_pin2, output2);
  }
}

void flash_timer1(){
  if(Timer1_start == -1){return ;}
  if(Timer1_start == 1){
    Timer1_start = 0;
  }else{
  output1 = !output1;
  digitalWrite(Step_pin1, output1);
  //delayMicroseconds(1);
  delay_microsec();
  output1 = !output1;
  digitalWrite(Step_pin1, output1);
  }
}
//-----------------------------------------

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
int dir1 = 1;//方向1
int dir2 = 1;//方向2
volatile float contl_L_pps  = 0.0; //左車輪の追加速度pps
volatile float contl_R_pps  = 0.0; //右車輪の追加速度pps
volatile float contl_pps    = 0.0;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000L); // 400kHz I2C clock. Comment this line if having compilation difficulties
//    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


    // wait for ready => 何か送られてくるまでスタートしない=============================================
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again
    //===============================================================================================

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }
    pinMode(Dir_pin1,OUTPUT);
    pinMode(Dir_pin2,OUTPUT);
    pinMode(Step_pin1,OUTPUT);
    pinMode(Step_pin2,OUTPUT);
    digitalWrite(Dir_pin1,dir1);
    digitalWrite(Dir_pin2,dir2);
    digitalWrite(Step_pin1,output1);
    digitalWrite(Step_pin2,output2);
    Timer1.initialize();
    Timer1_start = -1;
    Timer1.attachInterrupt(flash_timer1);
    Timer3.initialize();
    Timer2_start = -1;
    Timer3.attachInterrupt(flash_timer2);}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
//速度制御pid
float e            = 0;//偏差(p成分)
float old_e        = 0; //前回の偏差
float e_speed      = 0; //偏差の微分(d成分)
float e_integ      = 0; //偏差の蓄積(i成分)
//傾きの制御
float e2=0;
float old_e2       = 0;
float e2_speed     = 0;
float e2_integ     = 0;

float k_pps        = 0.0019635; //ppsから速度[rad/s]に変換する
float parpas_velo  = 0; //速度の目標値
float parpas_angle = 0;
float velo         = 0; //左右の平均値
float wheel_r      = 0.045; //車輪半径[m]

float velo_L       = 0; //左車輪の目標値

float velo_R       = 0; //右車輪の目標値


float pps          = 0;
float dt           = 0.005;//周期

float old_velo              = 0;
float pitch_data_old        = 0;
float angle_velo            = 0;
float estimate_speed_filter = 0;
float estimate_speed        = 0;

//PIDゲイン値=======================================================
//old pid
/*
//速度制御pid

float kp1 = -2900;
float ki1 = -100000;
float kd1 = 0.0;
//傾き制御pid
float kp2 = 0.00025;
float ki2 = 0.0;
float kd2 = 0.0000002;
*/

float kp1 = 0.002;
float ki1 = 0.1;
float kd1 = 0.0;
//傾き制御pid
float kp2 = 0.002;
float ki2 = 0.0;
float kd2 = 0.00001;

 //==================================================================

int time_set1 = 0;
int time_set2 = 0;

float initial_angle = 3;

void timer_set1(float res1){

    //int data=10;
    res1 = res1-contl_L_pps;
    if(res1 < 0){
        dir1 = 0;
    }else{
        dir1 = 1;
    }

/*  
    if(counter <=100){
      counter +=1;
      }else{
        contl_L_pps=0;
        }
*/
    /*
    if(res1 <= 0.0153 && res1 >= -0.0153){
      res1 = 0;
    }
    */
    velo_L = res1;
    /*
    if(res1 < 0){
      res1 = res1 * -1;
    }
    */

 /*
    if(res1 >= 0.03){
      time_set1 = int(40);
    }else if(res1 == 0){
      time_set1 = int(10000);
    }else{
    time_set1 = int(1/res1);
    }
    */
    if(res1 != 0){
    time_set1 = int(100.0/res1);

    /*
    if(counter <=100){
      counter +=1;
      }else{
        data=0;
        }*/

    if(time_set1 < 0){
      time_set1 *= -1;
    }
    if(time_set1 <= 200) {time_set1 = 200;}//40~50
    if(time_set1 >= 100000){time_set1 = 100000;}
    digitalWrite(Dir_pin1,dir1); 
    Timer1_start = 1;
    Timer1.initialize(time_set1);
    Timer1.restart();
    Timer1.attachInterrupt(flash_timer1); 
    }
  }

void timer_set2(float res2){
 
    //int data2=10;
    //int res2_abs=0;
    res2 = res2 - contl_R_pps;
    if(res2 < 0){
      dir2 =1;
    }else{
      dir2 = 0;
    }
/*        
    if(counter2 <=1000){
      counter2 +=1;
      }else{
        contl_R_pps=0;       }
  */      
    /*
    if(res2 <= 0.0153 && res2 >= -0.0153){
      res2 = 0;
    }
    */
    //Serial.println(counter2);
    velo_R = res2;
    //res2 = abs(res2);
    if(res2 != 0){
    time_set2 = int(100.0/res2);
    if(time_set2 < 0){
        time_set2 *= -1;
      }
    if(time_set2 <= 200) {time_set2 = 200;}//40~50
    if(time_set2 >= 100000){time_set2 = 100000;}
    digitalWrite(Dir_pin2,dir2);
    Timer2_start = 1;
    Timer3.initialize(time_set2);//ms
    Timer3.restart();
    Timer3.attachInterrupt(flash_timer2);
    }

}


void pid_controler(float pitch_data){
  //機体速度の推定===================================
    //velo = -(velo_L + velo_R)/2;//負が前
    //old_velo = velo;//もしかしたら必要かも
    velo = (-0.1125)*(velo_L + velo_R)/2; //[pps] to [deg/s]
    Serial.print("velo : ");
    Serial.println(velo);
    angle_velo = -(pitch_data - pitch_data_old);//どっちが正か確認すること場合によっては係数をかける
    Serial.print("angle_velo : ");
    Serial.println(angle_velo);
    pitch_data_old = pitch_data;
    //estimate_speed = -old_velo - angle_velo;
    estimate_speed = velo + angle_velo;
    estimate_speed_filter = estimate_speed_filter * 0.95 + estimate_speed * 0.05; //lowpassfilter
    Serial.print("estimate_speed : ");
    Serial.println(estimate_speed_filter);
 //=================================================
 //PI制御===========================================
    parpas_velo = contl_pps;
    //e = estimate_speed_filter - parpas_velo;
    e = (parpas_velo - estimate_speed_filter);
    e_speed  = (e-old_e)/dt;
    e_integ += (e-old_e)/2 * dt;
    old_e = e;
    parpas_angle = kp1 * e + kd1 * e_speed + ki1 * e_integ;
    Serial.print("parpas_angle : ");
    Serial.println(parpas_angle);
 //================================================
 //PD制御==========================================
    //Serial.print("parpas_angle : ");
    //Serial.println(parpas_angle);
    //e2 = pitch_data - parpas_angle - initial_angle;
    e2 = parpas_angle - pitch_data;
    e2_speed = (e2-old_e2)/dt;
    e2_integ += (e2-old_e2)/2 * dt;
    old_e2 = e2;
    pps += kp2 * e2 + kd2 * e2_speed + ki2 * e2_integ;
    Serial.println(pps);
 //================================================
    //Serial.println(pps);
    timer_set2(pps);
    timer_set1(pps);
}

float deg = 180/M_PI;
int counter = 0;
void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    //mpuIntStatus = false;
    //Serial.println(mpuIntStatus);

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    //mpuInterrupt = true;

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("ypr\t");
            //Serial.print(ypr[0] * 180/M_PI);
            //Serial.print("\t");
            //Serial.print(ypr[1] * 180/M_PI);
            //Serial.print("\t");
            Serial.println(ypr[2]);
        #endif
        if(counter > 400){
        //contl_L_pps = 0;
        //contl_R_pps = 0;
        pid_controler(ypr[2]);
        }else if(counter == 400){
          pitch_data_old = ypr[2];
          counter +=1;
        }else{
          counter += 1;
        }
        mpu.resetFIFO();
        delay(5);
    }
}

