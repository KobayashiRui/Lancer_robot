# Arduinoにて使用するライブラリー及びライブラリーの変更について
## ライブラリーのダウンロード
+ https://github.com/PaulStoffregen/TimerOne : TimerOneライブラリー
+ https://github.com/PaulStoffregen/TimerThree : TimerThreeライブラリー
+ https://github.com/jrowberg/i2cdevlib : I2Cdev及びMPU6050ライブラリー (Arduinoフォルダ内にあるもの)

## ライブラリーの変更について
MPU6050のDMPのサンプリング周波数を高速(通常100Hzから200Hz)にするためには下記の変更が必要
1. ```Arduino/libraries/MPU6050```へ移動し**MPU6050_6Axis_MotionApps20.h**を変更をする
2. 下記の場所を探し(305行目あたり) **0x01となっている部分を0x00に変更する(矢印の部分)**
```
0x07,   0x6C,   0x04,   0xF1, 0x28, 0x30, 0x38,   // CFG_12 inv_send_accel -> inv_construct3_fifo
0x02,   0x16,   0x02,   0x00, 0x01 → 0x00        // D_0_22 inv_set_fifo_rate
```
3. 上記の作業を終えたらWireライブラリーの設定を変更する
4. Wireライブラリーは ```arduino-<ダウンロードしたバージョン>/hardware/arduion/avr/libraries```内にある (各自の環境による)
5. 上記のディレクトリに移動したらさらに ```Wire/src/utility```に移動し**twi.h**を変更する
6. 下記の場所を探し **100000Lの部分を400000Lに変更する(矢印の部分)**
```
#ifndef twi_h
#define twi_h

  #include <inttypes.h>

  //#define ATMEGA8

  #ifndef TWI_FREQ
  #define TWI_FREQ 100000L →400000L
  #endif

  #ifndef TWI_BUFFER_LENGTH
  #define TWI_BUFFER_LENGTH 32
  #endif
```

