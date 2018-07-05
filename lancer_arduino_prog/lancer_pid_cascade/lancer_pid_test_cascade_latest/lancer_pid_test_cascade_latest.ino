#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include<TimerThree.h>
#include<TimerOne.h>

// ================================================================
// ===                    グローバル変数                         ===
// ================================================================
//pin設定-------------------------------
const int Dir_pin1 = 5;
const int Dir_pin2 = 8;
const int Step_pin1 = 6;
const int Step_pin2 = 9;
//---------------------------------------

//ステッピングモータ操作用変数-------------
int dir1 = 1;//方向1
int dir2 = 1;//方向2
int output1 = 1;
int output2 = 1;
//---------------------------------------
//==========制御用変数=============================================
//速度制御関連
float e            = 0;         //速度の偏差(p成分)
float old_e        = 0;         //前回の速度の偏差
float e_speed      = 0;         //速度の偏差の微分(d成分)
float e_integ      = 0;         //速度の偏差の積分(i成分)
//傾き制御関連
float e2           = 0;         //角度の偏差
float old_e2       = 0;         //前回の角度の偏差
float e2_speed     = 0;         //角度の偏差の微分
float e2_integ     = 0;         //角度の偏差の積分

float k_pps        = 0.0019635; //ppsから速度[rad/s]に変換する
float parpas_velo  = 0;         //速度の目標値
float parpas_angle = 0;         //傾きの目標値
float velo         = 0;         //左右の平均値
float wheel_r      = 0.045;     //車輪半径[m]

float velo_L       = 0;         //左車輪の目標値
float contl_L_pps  = 0;         //左車輪の追加速度pps
float velo_R       = 0;         //右車輪の目標値
float contl_R_pps  = 0;         //右車輪の追加速度pps

float pps          = 0;         //制御量
float dt           = 0.002;     //周期
//====================================================================

//===========PIDゲイン値===============================================
//速度制御pid
float kp1     = 1.0;
float ki1     = 0.00001;
float kd1     = 0.00001;
//傾き制御pid
float kp2     = 8;
float ki2     = 0.00001;
float kd2     = 0.00001;
//====================================================================

unsigned long time_set1 = 0;      //タイマー1の時間設定
unsigned long time_set2 = 0;      //タイマー3の時間設定
unsigned long Max_value = 4294967295;  //タイマーの最大の値=>これ以上はオーバーフローする

MPU6050 mpu;
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

//=======================================================================

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
  output2 = !output2;
  digitalWrite(Step_pin2, output2);
  //delayMicroseconds(1);
  delay_microsec();
  output2 = !output2;
  digitalWrite(Step_pin2, output2);
}

void flash_timer1(){
  output1 = !output1;
  digitalWrite(Step_pin1, output1);
  //delayMicroseconds(1);
  delay_microsec();
  output1 = !output1;
  digitalWrite(Step_pin1, output1);
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    pinMode(Dir_pin1,OUTPUT);
    pinMode(Dir_pin2,OUTPUT);
    pinMode(Step_pin1,OUTPUT);
    pinMode(Step_pin2,OUTPUT);
    digitalWrite(Dir_pin1,dir1);
    digitalWrite(Dir_pin2,dir2);
    digitalWrite(Step_pin1,output1);
    digitalWrite(Step_pin2,output2);
  
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================


void timer_set1(float res1){
    if(res1 < 0){
        dir1 = 0;
    }else{
        dir1 = 1;
    }
    digitalWrite(Dir_pin1,dir1);
    res1 = res1 + contl_L_pps;
    /*
    if(res1 <= 0.00024 && res1 >= -0.00024){
      res1 = 0;
    }
    */
    velo_L = k_pps * res1 * wheel_r;
    res1 = abs(res1);
    /*
    if(res1 == 0){
      time_set1 = Max_value;
    }else{
    time_set1 = (unsigned long)(1000000/res1);
    }
    */
    time_set1 = (1000/res1);
    Timer1.initialize(time_set1);
    Timer1.attachInterrupt(flash_timer1);  
  }

void timer_set2(float res2){
    int res2_abs=0;
    if(res2 < 0){
      dir2 =1;
    }else{
      dir2 = 0;
    }
    
    digitalWrite(Dir_pin2,dir2);
    res2 = res2 + contl_R_pps;
    /*
    if(res2 <= 0.00024 && res2 >= -0.00024){
      res2 = 0;
    }
    */
    velo_R = k_pps * res2 * wheel_r;
    res2 = abs(res2);
    /*
    if(res2 == 0){
      time_set2 = Max_value;
    }else{
    time_set2 = (unsigned long)(1000000/res2);
    }
    */
    time_set2 = (1000/res2);
    Timer3.initialize(time_set2);//ms
    Timer3.attachInterrupt(flash_timer2);
}


void pid_controler(float pitch_data){
    velo = (velo_L + velo_R)/2;
    e = velo - parpas_velo;
    e_speed  = (e-old_e)/dt;
    e_integ += (e-old_e)/2 * dt;
    old_e = e;
    parpas_angle = kp1 * e + kd1 * e_speed + ki1 * e_integ;
    e2 = pitch_data - parpas_angle;
    e2_speed = (e2-old_e2)/dt;
    e2_integ += (e2-old_e2)/2 * dt;
    old_e2 = e2;
    pps = kp2 * e2 + kd2 * e2_speed + ki2 * e2_integ;
    timer_set2(pps);
    timer_set1(pps);
}


void loop() {
    int counter = 0;
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {

    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif
        if(counter > 400){
        pid_controler(ypr[2]);
        }else{
          counter += 1;
        }
        delay(2);


    }
}
