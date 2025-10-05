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
#include <Wire.h>
#include <arduinoFFT.h>

//FFT configuration
#define SAMPLES 256
#define SAMPLE_FREQ 400
#define LED_PIN 13
#define FREQ_THRESHOLD 50000
#define BANDS 8

#define ACCEL_THRESHOLD 1

int IR_PIN = A0;
int BUTTON_PIN = 6;

int buttonState = 0;
int lastButtonState = 0;

//gyro variables
const float warningAngle=50;
const float criticalAngle=80;


//object inits
DFRobot_BMX160 bmx160;

//create buffers to store sensor data in memory

float vReal[SAMPLES];   // Real part of signal (accelerometer samples)
float vImag[SAMPLES];   // Imaginary part (all zeros for real input)
/*fft uses real and imaginary component of inputted data; we don't have an imaginary
component so thats just all zeroes*/

//create fft object
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLE_FREQ);

//baseline good condition spectrum
float baselineSpectrum[BANDS];
bool baselineSet = false; //indicates if theres a basline set stored

void setup(){
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  //init the hardware bmx160  
  if (bmx160.begin() != true){
    Serial.println(F("imu init false"));
    while(1);
  }
  Serial.println(F("imu initialized"));

  recordBaseline(); //method to obtain baseline data set + put in buffer

  //bunch of config stuff, keeping for potential later use

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

  //IR CODE
  //Serial.println(analogRead(IR_PIN));

  //IMU CODE
  sBmx160SensorData_t Omagn, Ogyro, Oaccel;

  /* Get a new sensor event */
  bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);

  //MAGNETOMETER!!!
    float home_magx = 160; 
    float home_magy = -30; 
    float home_magz = -620; 
    float home_magn = 640;
    float mag_threshold = 20;

    //get magnetometer results
    float magx= Omagn.x;
    float magy = Omagn.y;
    float magz = Omagn.z;
    float magn = sqrt(sq(magx) + sq(magy) + sq(magz));

    float d_magx = magx - home_magx;
    float d_magy = magy - home_magy;
    float d_magz = magz - home_magz;
    float d_magn = magn - home_magn;

    /* Display the magnetometer results (magn is magnetometer in uTesla) */
    // Serial.print(d_magx); Serial.print(" ");
    // Serial.print(d_magy); Serial.print(" ");
    // Serial.print(d_magz); Serial.print(" ");
    // Serial.print(d_magn); Serial.print(" ");

  //GYROSCOPE!!

    // Calculate accelerations (convert to g)
    float gyrox = Ogyro.x;// / 16384.0; // scale for ±2g
    float gyroy = Ogyro.y;// / 16384.0;
    float gyroz = Ogyro.z;// / 16384.0;

    // Convert to roll & pitch angles (degrees)
    float roll  = atan2(gyroy, gyroz) * 180 / PI; //tilt around x-axis
    float pitch = atan2(-gyrox, sqrt(gyroy * gyroy + gyroz * gyroz)) * 180 / PI; //tilt round y-axis

    float angleTilt= max(abs(roll), abs(pitch));

    //Serial.println(angleTilt);

    // if(angleTilt>=criticalAngle){
    //   Serial.println(F("CRITICAL TILT WARNING"));
    // } else if(angleTilt>=warningAngle){
    //   Serial.println(F("WARNING: TILT DETECTED"));  
    // } else{
    //   Serial.println(F("Normal operation"));
    // }

    /* Display the gyroscope results (gyroscope data is in g) */
    // Serial.print(gyrox); Serial.print("  ");
    // Serial.print(gyroy); Serial.print("  ");
    // Serial.print(gyroz); Serial.print("  ");
  
    
  //ACCELEROMETER!!

    //BASIC ACCEL (FOR COLLISONS)
      float accelx= Oaccel.x;
      float accely = Oaccel.y;
      float accelz = Oaccel.z;
      float acceln = sqrt(sq(accelx) + sq(accely) + sq(accelz));

      if (acceln > ACCEL_THRESHOLD) {
        digitalWrite(LED_PIN, HIGH); //light up LED if threshold exceeded
      } else {
        digitalWrite(LED_PIN, LOW);
      }

      //Serial.println(acceln);

    //OVERCOMPLICATED ACCEL (FOR VIBRATIONS)
      collectSamples(); //get samples
      runFFT(); //translate samples

      double freq_diff //VAR FOR ACTUAL USAGE
      = compareToBaseline(); //compare sample sets

      const float alpha = 0.2;  // smoothing factor, 0 < alpha <= 1
      static double smooth_diff = 0;  // remembers previous value
      smooth_diff = alpha * freq_diff + (1 - alpha) * smooth_diff;

      // Serial.print(" ");
      // Serial.println(freq_diff);

      //basic LED test; will be replaced with LCD code later
      if (smooth_diff > FREQ_THRESHOLD) {
        digitalWrite(LED_PIN, HIGH); //light up LED if threshold exceeded
      } else {
        digitalWrite(LED_PIN, LOW);
      }

      buttonState = digitalRead(BUTTON_PIN);

      if (buttonState == LOW && lastButtonState == HIGH) {
        delay(4000);
        recordBaseline();
      }

      lastButtonState = buttonState;
      //Serial.print(digitalRead(BUTTON_PIN));

    /* Display the accelerometer results (accelerometer data is in m/s^2) */
    // Serial.print(Oaccel.x); Serial.print(" ");
    // Serial.print(Oaccel.y); Serial.print(" ");
    // Serial.print(Oaccel.z); Serial.print(" ");

    delay(100); //change loop rate here
}

//method to 
void collectSamples() {
  //get time spacing for sample readings. unsigned long = large pos int
  unsigned long microsPerSample = 1000000UL / SAMPLE_FREQ;

  //loop to take amount of samples
  for (int i = 0; i < SAMPLES; i++) {
    //get time since arduino started
    unsigned long tStart = micros();

    //get sensor data in this method (diff from loop method)
    sBmx160SensorData_t Omagn, Ogyro, Oaccel;
    bmx160.getAllData(&Omagn, &Ogyro, &Oaccel); //don't need mag and gyro here

    //set index in sample buffer to the norm accel reading
    vReal[i] = sqrt(
      (double)Oaccel.x*Oaccel.x +
      (double)Oaccel.y*Oaccel.y +
      (double)Oaccel.z*Oaccel.z);

    vImag[i] = 0.0; //set index in imaginary buffer to 0, will fill with all 0s


    //wait until next sample
    while (micros() - tStart < microsPerSample);
  }

  //remove mean so that frequency oscillates around zero
  double mean = 0.0;
  for (int i = 0; i < SAMPLES; i++) mean += vReal[i];
  mean /= SAMPLES;
  for (int i = 0; i < SAMPLES; i++) vReal[i] -= mean;
}

void runFFT() {
  //hamming fixes the issue that the data won't 'loop' perfectly in given sample time
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); // apply Hamming window
  FFT.compute(FFTDirection::Forward);                       // compute FFT
  FFT.complexToMagnitude();                                  // convert to magnitudes
}


void recordBaseline() {
  //get and convert samples

  Serial.println(F("recording baseline"));

  collectSamples();
  runFFT();

  int binsPerBand = (SAMPLES/2) / BANDS;

  //assign samples to the baseline spectrum matrix
  for (int b = 0; b < BANDS; b++) {
    float sum = 0;
    for (int i = b*binsPerBand; i < (b+1)*binsPerBand; i++) {
      sum += vReal[i];
      Serial.println(vReal[i]);
    }
    baselineSpectrum[b] = sum / binsPerBand; // average magnitude for this band
  }

  //say that baseline is now filled 
  baselineSet = true;
  Serial.println(F("baseline recorded"));
}

//compare live set of data to baseline
double compareToBaseline() {
  int binsPerBand = (SAMPLES/2) / BANDS;
  double diff = 0.0;
 
  //iterate through indices of live and baseline sets
  for (int b = 0; b < BANDS; b++) {
    float sum = 0;
    for (int i = b*binsPerBand; i < (b+1)*binsPerBand; i++) {
      sum += vReal[i];
    }
    float liveAvg = sum / binsPerBand;
    //Serial.println(liveAvg);
    //what is the difference between mag in each set?
    float delta = liveAvg - baselineSpectrum[b];
     //add difference to overall difference. square penalizes larger differences more
    diff += delta;// * delta;
  }

  return diff;
}



