
#include <DFRobot_BMX160.h>

//init xyz + norm unprocessed
float magx = 0;
float magy = 0;
float magz = 0;
float magn = 0;

//home xyz + norm; set properly once mounting is figured out
float home_magx = 160;
float home_magy = -30;
float home_magz = -620;
float home_magn = 640;
float mag_threshold = 20;

//init difference in xyz + norm home vs real magnitude
float d_magx = 0;
float d_magy = 0;
float d_magz = 0;
float d_magn = 0;

DFRobot_BMX160 bmx160;
void setup(){
  Serial.begin(115200);
  delay(20);
  
  //this should label the vars in serial but it doesnt lmao
  Serial.println("MagX MagY MagZ MagN");

  //init the hardware bmx160  
  if (bmx160.begin() != true){
    Serial.println("init false");
    while(1);
  }
  //bmx160.setLowPower();   //disable the gyroscope and accelerometer sensor
  //bmx160.wakeUp();        //enable the gyroscope and accelerometer sensor
  //bmx160.softReset();     //reset the sensor
  
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

  //calculate difference in xyz + norm home vs real magnitude
  d_magx = magx - home_magx;
  d_magy = magy - home_magy;
  d_magz = magz - home_magz;
  d_magn = magn - home_magn;

  /* Display the magnetometer difference results (magn is magnetometer in uTesla) */
  Serial.print(d_magx); Serial.print(" "); 
  Serial.print(d_magy); Serial.print(" ");
  Serial.print(d_magz); Serial.print(" ");
  Serial.print(d_magn); Serial.print(" ");

  Serial.println("");

  delay(20);
}

