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
#define SAMPLES 512
#define SAMPLE_FREQ 400
#define LED_PIN 13
#define FREQ_THRESHOLD 50000

int IR_PIN = A0;

//magnetometer variables
float magx = 0; float magy = 0; float magz = 0; float magn = 0;

float home_magx = 160; float home_magy = -30; float home_magz = -620; float home_magn = 640;
float mag_threshold = 20;

float d_magx = 0; float d_magy = 0; float d_magz = 0; float d_magn = 0;


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
float baselineSpectrum[SAMPLES/2];
//uses samples/2 because half the data is a mirror or smth idk why vReal doesnt do this tbh lol
bool baselineSet = false; //indicates if theres a basline set stored

void setup(){
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  //init the hardware bmx160  
  if (bmx160.begin() != true){
    Serial.println("imu init false");
    while(1);
  }
  Serial.println("imu initialized");

  Serial.println("recording baseline");
  recordBaseline(); //method to obtain baseline data set + put in buffer
  Serial.println("baseline recorded");

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
    // Serial.print(d_magx); Serial.print(" ");
    // Serial.print(d_magy); Serial.print(" ");
    // Serial.print(d_magz); Serial.print(" ");
    // Serial.print(d_magn); Serial.print(" ");

  //GYROSCOPE!!

    /* Display the gyroscope results (gyroscope data is in g) */
    // Serial.print("G ");
    // Serial.print("X: "); Serial.print(Ogyro.x); Serial.print("  ");
    // Serial.print("Y: "); Serial.print(Ogyro.y); Serial.print("  ");
    // Serial.print("Z: "); Serial.print(Ogyro.z); Serial.print("  ");
    // Serial.println("g");
  
  //ACCELEROMETER!!

    /* Display the accelerometer results (accelerometer data is in m/s^2) */
    // Serial.print(Oaccel.x); Serial.print(" ");
    // Serial.print(Oaccel.y); Serial.print(" ");
    // Serial.print(Oaccel.z); Serial.print(" ");

    //actual FFT code
    collectSamples();
    runFFT(); //method to analyze samples

    double freq_diff //VAR FOR ACTUAL USAGE
    = compareToBaseline(); //method to compare sample sets

    const float alpha = 0.2;  // smoothing factor, 0 < alpha <= 1
    static double smooth_diff = 0;  // remembers previous value
    smooth_diff = alpha * freq_diff + (1 - alpha) * smooth_diff;

    Serial.print(" ");
    Serial.println(freq_diff);

    //basic LED test; will be replaced with LCD code later
    if (smooth_diff > FREQ_THRESHOLD) {
      digitalWrite(LED_PIN, HIGH); //light up LED if threshold exceeded
    } else {
      digitalWrite(LED_PIN, LOW);
    }

    delay(20); //change loop rate here
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
      //(double)Oaccel.x*Oaccel.x +
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
  collectSamples();
  runFFT();

  //assign samples to the baseline spectrum matrix
  for (int i = 0; i < SAMPLES/2; i++) {
    baselineSpectrum[i] = vReal[i];
  }
  //say that baseline is now filled 
  baselineSet = true;
}

//compare live set of data to baseline
double compareToBaseline() {
  double diff = 0.0;

  const int LOW_BIN = 0; //ignore low frequency bins
  //iterate through indices of live and baseline sets
  for (int i = LOW_BIN; i < SAMPLES/2; i++) {
    //what is the difference between mag in each set?
    double delta = vReal[i] - baselineSpectrum[i];
    //add difference to overall difference. square penalizes larger differences more
    diff += delta * delta;
  }
  return diff;
}



