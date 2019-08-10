//including used libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

//Define widths
 //We define each byte of data input, in this case just 6 channels
int pwm_width_2 = 0;
int pwm_width_3 = 0;
int pwm_width_4 = 0;
int pwm_width_5 = 0;
int pwm_width_6 = 0;
int pwm_width_7 = 0;

Servo PWM2;
Servo PWM3;
Servo PWM4;
Servo PWM5;
Servo PWM6;
Servo PWM7;


struct MyData {
int throttle;
float yaw;
float pitch;
float roll;
bool AUX1;
byte AUX2;
};
MyData data;


const uint64_t pipeIn = 0xE8E8F0F0E1LL;     //opening the pipe line
RF24 radio(9, 10);                          //definin nrf reciever connections



void resetData(){
//We define the inicial value of each data input
//3 potenciometers will be in the middle position so 127 is the middle from 254
data.throttle = 0;
data.yaw = 0;
data.pitch = 0;
data.roll = 0;
data.AUX1 = false;
data.AUX2 = 0;
}

/**************************************************/

void setup()
{
  //Set the pins for each PWM signal
  PWM2.attach(2);
  PWM3.attach(3);
  PWM4.attach(4);
  PWM5.attach(5);
  PWM6.attach(6);
  PWM7.attach(7);

  //Configure the NRF24 module
  resetData();
  radio.begin();
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MIN);
  radio.openReadingPipe(1,pipeIn);

  //we start the radio comunication
  radio.startListening();
  Serial.begin(115200);
}

/**************************************************/

unsigned long lastRecvTime = 0;

void recvData()
{
while ( radio.available() ) {
radio.read(&data, sizeof(MyData));
lastRecvTime = millis(); //here we receive the data
}
}

/**************************************************/

void loop()
{
recvData();
unsigned long now = millis();
//Here we check if we've lost signal, if we did we reset the values
if ( now - lastRecvTime > 1000 ) {
// Signal lost?
resetData();
Serial.println("lost");
}

pwm_width_4 = map(data.throttle, 0, 1023, 1000, 2000);     //PWM value on digital pin D2
pwm_width_5 = map(data.yaw,    -45,   45, 1000, 2000);     //PWM value on digital pin D3
pwm_width_3 = map(data.pitch,   45,  -45, 1000, 2000);     //PWM value on digital pin D4
pwm_width_2 = map(data.roll,   -45,   45, 1000, 2000);     //PWM value on digital pin D5
pwm_width_6 = map(data.AUX1,     0, 255, 1000, 2000);     //PWM value on digital pin D6
pwm_width_7 = map(data.AUX2,     0, 255, 1000, 2000);     //PWM value on digital pin D7
if(data.AUX1) {
  pwm_width_6 = 2000;
}
else {
  pwm_width_6 = 1000;
}
pwm_width_7 = 1000;

/*Serial.print(pwm_width_2);
Serial.print("\t");
Serial.print(pwm_width_3);
Serial.print("\t");
Serial.print(pwm_width_4);
Serial.print("\t");
Serial.print(pwm_width_5);
Serial.print("\t");
Serial.print(pwm_width_6);
Serial.print("\t");
Serial.println(data.roll);*/


//Now we write the PWM signal using the servo function
PWM2.writeMicroseconds(pwm_width_2);
PWM3.writeMicroseconds(pwm_width_3);
PWM4.writeMicroseconds(pwm_width_4);
PWM5.writeMicroseconds(pwm_width_5);
PWM6.writeMicroseconds(pwm_width_6);
PWM7.writeMicroseconds(pwm_width_7);


}//Loop end
/**************************************************/
