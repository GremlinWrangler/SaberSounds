/*
  Demo of the audio sweep function.
  The user specifies the amplitude,
  start and end frequencies (which can sweep up or down)
  and the length of time of the sweep.
   
FMI:
The audio board uses the following pins.
 6 - MEMCS
 7 - MOSI
 9 - BCLK
10 - SDCS
11 - MCLK
12 - MISO
13 - RX
14 - SCLK
15 - VOL
18 - SDA
19 - SCL
22 - TX
23 - LRCLK

*/

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <Bounce.h>
#include <NXPMotionSense.h>
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <EEPROM.h>


// GUItool: begin automatically generated code
AudioSynthWaveformSine   sine3;          //xy=83,281
AudioSynthWaveformSineModulated sine_fm1;       //xy=215,317
AudioSynthWaveformSine   sine1;          //xy=244,191
AudioSynthWaveformSine   sine2;          //xy=244,241
AudioSynthNoiseWhite     noise1;         //xy=258,443
AudioSynthNoisePink      pink1;          //xy=276,491
AudioMixer4              mixer1;         //xy=491,420
AudioOutputAnalog        dac1;           //xy=920,344
AudioConnection          patchCord1(sine3, sine_fm1);
AudioConnection          patchCord2(sine_fm1, 0, mixer1, 1);
AudioConnection          patchCord3(sine1, 0, mixer1, 0);
AudioConnection          patchCord4(sine2, 0, mixer1, 2);
AudioConnection          patchCord5(pink1, 0, mixer1, 3);
AudioConnection          patchCord6(mixer1, dac1);
// GUItool: end automatically generated code


NXPMotionSense imu;
Madgwick filter;


uint32_t lastmillisCount = 0; 
uint32_t lastCollisionmillisCount = 0; 
uint32_t counter=0;  //progress vlue for updating perlin noise function
float lastHeadingValue=0; 
float lastRollValue=0;    
float headingRate=0; //holding values for generation of audio
float rollRate=0;    //  "

#define CHORUS_DELAY_LENGTH (16*AUDIO_BLOCK_SAMPLES)
#define FLANGE_DELAY_LENGTH (6*AUDIO_BLOCK_SAMPLES)
float t_ampx = 0.8;
int t_lox = 10;
int t_hix = 22000;
// Length of time for the sweep in seconds
float t_timex = 10;

int s_idx = FLANGE_DELAY_LENGTH/4;
int s_depth = FLANGE_DELAY_LENGTH/4;
double s_freq = .5;


// Allocate the delay lines for left and right channels
short l_delayline[CHORUS_DELAY_LENGTH];
short r_delayline[CHORUS_DELAY_LENGTH];

// <<<<<<<<<<<<<<>>>>>>>>>>>>>>>>
void setup(void)
{
  pinMode(5,OUTPUT);
   pinMode(13,OUTPUT);
  digitalWrite(5,HIGH);
   AudioMemory(120);
   sine1.amplitude(0.0);
  sine1.frequency(400.00);
  
   sine2.amplitude(0);
   sine2.phase(1);
   sine2.frequency(200.000);


   sine3.amplitude(1);
   sine3.frequency(50.000);

   sine_fm1.amplitude(0.3);
    sine_fm1.frequency(200.000);
    Serial.begin(9600);
    imu.begin();
}

void loop(void)
{
  if (lastCollisionmillisCount+500>millis()) sparksAudio(); else pnoiseaudio();
  updateIMU();
 // delay(1);
 // sine1.amplitude(0.0);
 /*
 if (millis()-lastmillisCount>500){
    Serial.println("ding"); 
    lastmillisCount=millis();
 }
 // pink1.amplitude(0.5);

 counter=millis()%500;
 float tempsin=counter;
 tempsin=sin(tempsin/500*PI*2)*100;
 sine3.amplitude(tempsin/3);
 tempsin = tempsin+600;
 sine1.frequency(floor(tempsin));
 
  counter=millis()%300;
  tempsin=counter;
 tempsin=sin(tempsin/300*PI*2)*50;
 tempsin = tempsin+550;
 sine2.frequency(floor(tempsin)); */
 // pink1.amplitude(0.0);
 //  sine1.amplitude(0.5);
}

void updateIMU()
{
 float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, heading;

  if (imu.available()) {
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

    // Scale the gyroscope to the range Madgwick expects
    float gyroScale = 0.097656f; // TODO: is this really correct?
    gx = gx * gyroScale;
    gy = gy * gyroScale;
    gz = gz * gyroScale;
    headingRate= (filter.getYaw()-lastHeadingValue)*10;
    rollRate= (filter.getRoll()-lastRollValue)*10;
    
    lastHeadingValue=filter.getYaw();
    lastRollValue=filter.getRoll();
    if (headingRate<0) headingRate=0-headingRate;
    if (rollRate<0) rollRate=0-rollRate;
    if ((headingRate>1)||(rollRate>1))   lastCollisionmillisCount=millis();
    // Update the Madgwick filter
    filter.updateIMU(gx, gy, gz, ax, ay, az);
    //filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  }

  if (readyToPrint()) {
    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print(heading);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.println(roll);
  }

  
}

bool readyToPrint() {
  static unsigned long nowMillis;
  static unsigned long thenMillis;

  // If the Processing visualization sketch is sending "s"
  // then send new data each time it wants to redraw
  while (Serial.available()) {
    int val = Serial.read();
    if (val == 's') {
      thenMillis = millis();
      return true;
    }
  }
  // Otherwise, print 8 times per second, for viewing as
  // scrolling numbers in the Arduino Serial Monitor
  nowMillis = millis();
  if (nowMillis - thenMillis > 125) {
    thenMillis = nowMillis;
    return true;
  }
  return false;
}
void sparksAudio()
{
     sine_fm1.amplitude(0.0);
     pink1.amplitude(0.6);
}

void pnoiseaudio()
{  pink1.amplitude(0.0);  
   sine_fm1.amplitude(0.3);
    sine3.frequency(50.000+rollRate*10); 
    sine_fm1.frequency(200.000+ headingRate*50);
   counter=millis();
      float yflot = counter;
      yflot = yflot/100;
     
       float rise = 0;
       rise = rise/500;
       float amp = (pnoise(0,yflot-cos(yflot),rise))/2+0.5;
       amp=amp*amp;
       sine1.amplitude(amp);
       sine1.frequency(300-amp*50);
       if ((amp>1)||(amp<0)){digitalWrite(13,HIGH);}else{digitalWrite(13,LOW);}
 if (millis()-lastmillisCount>500){
    Serial.println(amp); 
    lastmillisCount=millis();
 }   
}



