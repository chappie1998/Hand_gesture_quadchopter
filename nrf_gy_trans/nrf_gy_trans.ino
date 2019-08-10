//including necessary
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//opening the pipline for nRF24L01 communication
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
bool sw;
float t_now =0;

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

bool arm = false;

/*Create a unique pipe out. The receiver has to
  wear the same unique code*/

const uint64_t pipeOut = 0xE8E8F0F0E1LL;

RF24 radio(7, 8); // select  CSN  pin

// The sizeof this struct should not exceed 32 bytes
// This gives us up to 32 8 bits channals
struct MyData {
  int throttle;
  float  yaw;
  float  pitch;
  float  roll;
  bool AUX1;
  byte AUX2;
};

MyData data;

void resetData()
{
  //This are the start values of each channal
  // Throttle is 0 in order to stop the motors
  //127 is the middle value of the 10ADC.

  data.throttle = 0;
  data.yaw = 0;
  data.pitch = 0;
  data.roll = 0;
  data.AUX1 = false;
  data.AUX2 = 0;
}

void setup()
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    mpu.initialize();
    Serial.begin(115200);
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);
    if (devStatus == 0) {

        mpu.setDMPEnabled(true); // enable Arduino interrupt
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
   pinMode(LED_PIN, OUTPUT);
   pinMode(3, INPUT);
   pinMode(A2, INPUT);
   pinMode(9, OUTPUT);
   pinMode(10, OUTPUT);
  //attachInterrupt(digitalPinToInterrupt(3), my_interrupt_handler, FALLING);
  radio.begin();
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MIN);
  radio.openWritingPipe(pipeOut);
  resetData();


}




void loop()
{
      if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {

        mpu.resetFIFO();

    }
    else if (mpuIntStatus & 0x02) {
     while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        Serial.print((ypr[2] * 18/M_PI)/5);
        Serial.print(" ");
        Serial.print((ypr[1] * 180/M_PI)*10);
        Serial.print(" ");
        Serial.println(ypr[0] * 180/M_PI);


        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

      }

  //preparing the data as our requirment
  sw = digitalRead(3);
  if(sw == 0) {
    if(micros()-t_now > 1000000) {
      arm = !arm;
      t_now = micros();
    }
  }
  else {
    t_now = micros();
  }

  if( data.yaw > -70 && data.yaw < 70) {
    digitalWrite(10, HIGH);
  }
  else {
    digitalWrite(10, LOW);
  }

  if(arm) {
    digitalWrite(9, HIGH);
  }
  else {
    digitalWrite(9, LOW);
  }
  data.yaw=ypr[0]* 180/M_PI;
  data.pitch=ypr[1]* 180/M_PI;
  data.roll=ypr[2]* 180/M_PI;

  data.throttle = analogRead(A2);
  data.pitch = constrain(data.pitch, -45.0, 45.0);
  data.roll = constrain(data.roll, -45.0, 45.0);
  data.AUX1 = arm;

  data.yaw = 0;

  radio.write(&data, sizeof(MyData));
}
