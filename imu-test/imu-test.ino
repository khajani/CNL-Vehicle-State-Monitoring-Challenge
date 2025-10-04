/*!
 * @file readAllData.ino
 * @brief Through the example, you can get the sensor data by using getSensorData:
 * @n     get all data of magnetometer, gyroscope, accelerometer.
 * @n     With the rotation of the sensor, data changes are visible.
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [luoyufeng] (yufeng.luo@dfrobot.com)
 * @maintainer [Fary](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2021-10-20
 * @url https://github.com/DFRobot/DFRobot_BMX160
 */
#include <DFRobot_BMX160.h>

float magx = 0;
float magy = 0;
float magz = 0;
float magn = 0;

float home_magx = 160;
float home_magy = -30;
float home_magz = -620;
float home_magn = 640;
float mag_threshold = 20;

float d_magx = 0;
float d_magy = 0;
float d_magz = 0;
float d_magn = 0;

DFRobot_BMX160 bmx160;
void setup(){
  Serial.begin(115200);
  delay(20);
  
  Serial.println("MagX MagY MagZ MagN");



  //init the hardware bmx160  
  if (bmx160.begin() != true){
    Serial.println("init false");
    while(1);
  }
  //bmx160.setLowPower();   //disable the gyroscope and accelerometer sensor
  //bmx160.wakeUp();        //enable the gyroscope and accelerometer sensor
  //bmx160.softReset();     //reset the sensor
  
  /** 
   * enum{eGyroRange_2000DPS,
   *       eGyroRange_1000DPS,
   *       eGyroRange_500DPS,
   *       eGyroRange_250DPS,
   *       eGyroRange_125DPS
   *       }eGyroRange_t;
   **/
  //bmx160.setGyroRange(eGyroRange_500DPS);
  
  /**
   *  enum{eAccelRange_2G,
   *       eAccelRange_4G,
   *       eAccelRange_8G,
   *       eAccelRange_16G
   *       }eAccelRange_t;
   */
  //bmx160.setAccelRange(eAccelRange_4G);
  delay(20);
}

void loop(){
  sBmx160SensorData_t Omagn, Ogyro, Oaccel;

  /* Get a new sensor event */
  bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);

  //get magnetometer results
  magx= Omagn.x;
  magy = Omagn.y;
  magz = Omagn.z;
  magn = sqrt(sq(magx) + sq(magy) + sq(magz));

  d_magx = magx - home_magx;
  d_magy = magy - home_magy;
  d_magz = magz - home_magz;
  d_magn = magn - home_magn;

  /* Display the magnetometer results (magn is magnetometer in uTesla) */
 // Serial.print("M ");
  // Serial.print("X: "); 
  Serial.print(d_magx); Serial.print(" ");
  // Serial.print("Y: "); 
  Serial.print(d_magy); Serial.print(" ");
  // Serial.print("Z: "); 
  Serial.print(d_magz); Serial.print(" ");

  Serial.print(d_magn); Serial.print(" ");
 // Serial.println("uT");



  /* Display the gyroscope results (gyroscope data is in g) */
  // Serial.print("G ");
  // Serial.print("X: "); Serial.print(Ogyro.x); Serial.print("  ");
  // Serial.print("Y: "); Serial.print(Ogyro.y); Serial.print("  ");
  // Serial.print("Z: "); Serial.print(Ogyro.z); Serial.print("  ");
  // Serial.println("g");
  
  // // /* Display the accelerometer results (accelerometer data is in m/s^2) */
  // // Serial.print("A ");
  // // Serial.print("X: "); 
  // Serial.print(Oaccel.x); Serial.print(" ");
  // // Serial.print("Y: "); 
  // Serial.print(Oaccel.y); Serial.print(" ");
  // // Serial.print("Z: "); 
  // Serial.print(Oaccel.z); Serial.print(" ");
  // // Serial.println("m/s^2");

  Serial.println("");

  delay(20);
}

