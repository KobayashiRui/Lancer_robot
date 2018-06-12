//MPU6050テスト

#include <Wire.h>
#define MPU6050_ADDR 0x68
#define MPU6050_AX  0x3B
#define MPU6050_AY  0x3D
#define MPU6050_AZ  0x3F
#define MPU6050_TP  0x41    //  data not used
#define MPU6050_GX  0x43
#define MPU6050_GY  0x45
#define MPU6050_GZ  0x47

#define k1 15
#define k2 10
#define k3 0.005
#define k4 0.001

#define step_num 0.1125 //ステッピングモータのoneステップの角度[deg]
#define step1_1 25 //ステッピングモータ(右)の速度決定のピン番号
#define step1_2 26 //ステッピングモータ(右)の回転方向決定のピン番号
#define step2_1 16 //ステッピングモータ(左)の速度決定のピン番号
#define step2_2 17 //ステッピングモータ(左)の回転方向決定のピン番号

short int AccX, AccY, AccZ;
short int Temp;
short int GyroX, GyroY, GyroZ;
float pre_param[3] = {0,0,0};

float acc_x,acc_y,acc_z;
float gyro_x,gyro_y,gyro_z;
float old_gyro_x=0.0, old_gyro_y=0.0, old_gyro_z=0.0;
float acc_angle_x, acc_angle_y, acc_angle_z; //加速度センサから求める対地角度
float gyro_angle_x, gyro_angle_y, gyro_angle_z;//ジャイロセンサから求める角度
unsigned long old_time=0,now_time=0;
float time_data=0;
float angle_data_y=0;
unsigned long old_time_pid=0,now_time_pid=0;
float time_data_pid=0;
float velocity=0;
float desired_speed=0;
float e_speed=0,old_e_speed=0;
float e_angle=0,old_e_angle=0;
float tilt_angle=0;
float accel = 0; 


//mpu6050の初期設定
void start_up_mpu6050(){
  //  wake it up
  //mpu6050の初期設定
  Wire.beginTransmission(MPU6050_ADDR);//デバイスアドレス
  Wire.write(0x6B);//クロック設定などのレジスタ
  Wire.write(0x00);//0なのでクロックは内部クロックを使用
  Wire.endTransmission();
  //ジャイロの設定
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);//ジャイロの設定レジスタ
  Wire.write(0x08);//00001000 => 角速度に変換するもの レンジ : 500deg/s, SF : 65.5LSB/(deg/s)
  Wire.endTransmission();

  //加速度センサーの初期設定
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); //加速度センサの初期設定レジスタ
  Wire.write(0x10); //00010000 => レンジ : ±8G, SF : 4096LSB/G
  Wire.endTransmission();

  //ローパスフィルタの設定
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
}

void get_data(){
  //  send start address
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_AX);
  Wire.endTransmission();  
  //  request 14bytes (int16 x 7)
  Wire.requestFrom(MPU6050_ADDR, 14);
  //  get 14bytes
  AccX = Wire.read() << 8;  AccX |= Wire.read();
  AccY = Wire.read() << 8;  AccY |= Wire.read();
  AccZ = Wire.read() << 8;  AccZ |= Wire.read();
  Temp = Wire.read() << 8;  Temp |= Wire.read();  //  (Temp-12421)/340.0 [degC]
  GyroX = Wire.read() << 8; GyroX |= Wire.read();
  GyroY = Wire.read() << 8; GyroY |= Wire.read();
  GyroZ = Wire.read() << 8; GyroZ |= Wire.read();
}

//mpu6050ジャイロの初期値設定
void inital_gyro(){
  for(int k=0; k < 100; k++){
    get_data();
    pre_param[0] += GyroX;
    pre_param[1] += GyroY;
    pre_param[2] += GyroZ;
    delay(5);
  }
  pre_param[0] /= 100;
  pre_param[1] /= 100;
  pre_param[2] /= 100;
  old_time = millis();
  get_data();
  old_gyro_x = (GyroX-pre_param[0])/65.5;//[deg/s]
  old_gyro_y = (GyroY-pre_param[0])/65.5;
  old_gyro_z = (GyroZ-pre_param[0])/65.5;
}

int j =0;
void control_stepper_motor(float velo){
  int pps_velo = 0;
  pps_velo = abs(velo);
  if(velo > 0){
    digitalWrite(step1_2, LOW);
    digitalWrite(step2_2, HIGH);
  }else{

    digitalWrite(step1_2, HIGH);
    digitalWrite(step2_2, LOW);
  }
  ledcSetup(0,pps_velo,8);
  ledcSetup(1,pps_velo,8);
  ledcAttachPin(step1_1,0);
  ledcAttachPin(step2_1,1);
  ledcWrite(0,128);
  ledcWrite(1,128);
}



void timer_pid() {
  now_time = millis();
  get_data();  
  acc_x = AccX/4096.0;//[G]
  //acc_y = AccY/4096.0;
  acc_z = AccZ/4096.0;
  
  //gyro_x = (GyroX-pre_param[0])/65.5;//[deg/s]
  gyro_y = (GyroY-pre_param[0])/65.5;
  //gyro_z = (GyroZ-pre_param[0])/65.5;
  //ジャイロから角度を求める
  time_data = (now_time - old_time) / 1000.0;
  //gyro_angle_x += (gyro_x + old_gyro_x) / 2 * (time_data);
  gyro_angle_y += (gyro_y + old_gyro_y) / 2 * (time_data);
  //gyro_angle_z += (gyro_z + old_gyro_z) / 2 * (time_data);
  old_time = now_time;
  //old_gyro_x = gyro_x;
  old_gyro_y = gyro_y;
  //old_gyro_z = gyro_z;
  
  //加速度から角度を出す
  acc_angle_y = atan2(acc_x, acc_z) * 360 / 2.0 / PI; //[deg]
  //acc_angle_x = atan2(acc_y, acc_z) * 360 / 2.0 / PI;
  //acc_angle_z = atan2(acc_x, acc_z) * 360 / 2.0 / PI;

  //相補フィルタ
  angle_data_y = (0.8 * gyro_angle_y + 0.2* acc_angle_y);
  //Serial.println(angle_data_y);
  gyro_angle_y = angle_data_y;
  //pid
  e_speed = desired_speed - velocity;
  now_time_pid = millis();
  time_data_pid = (now_time_pid - old_time_pid);
  velocity = k1 * angle_data_y+k2 * gyro_angle_y;
  //velocity += accel;
  old_time_pid = now_time_pid;
  old_e_speed = e_speed;
  old_e_angle = e_angle;
}

void setup() {
  //  serial for debugging
  //Serial.begin(115200);
  //Serial.println("*** started");
  //  i2c as a master
  Wire.begin();
  start_up_mpu6050();
  inital_gyro();
  pinMode(step1_1,OUTPUT);
  pinMode(step1_2,OUTPUT);
  pinMode(step2_1,OUTPUT);
  pinMode(step2_2,OUTPUT);
}

void loop(){
  //ステッピングモータのコントロール
  timer_pid();
  //Serial.println(velocity);
  control_stepper_motor(velocity);
  delay(10);
}
