#include <PS4BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the PS4BT class in two ways */
// This will start an inquiry and then pair with the PS4 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS4 controller will then start to blink rapidly indicating that it is in pairing mode
PS4BT PS4(&Btd);

// After that you can simply create the instance like so and then press the PS button on the device
//PS4BT PS4(&Btd);

bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;

#define RPWM_2 7
#define RPWM_1 44
#define RPWM_3 12
#define LPWM_2 6
#define LPWM_1 46
#define LPWM_3 11

int x, y, w, v, v_x, v_y, v_w;
int v1, v2, v3;
int targated_angle = 0, curr_angle=0, error=0, prev_error = 0, correction = 0, diff = 0;
float angle;
float KP = 0.2, KD = 0;
int z;
byte l, m;



void BNO()
{
  digitalWrite(21, HIGH);
  if (Serial1.available() >= 2)
  {
    m = Serial1.read();
    l = Serial1.read();
    digitalWrite(21, LOW);
    curr_angle = (m<<8) | l;
//    Serial.println("Angle");
//    Serial.println(z);
   if (curr_angle > 180)
      curr_angle = curr_angle - 360;
    Serial.println(String ("Angle--- ") + curr_angle);
   

  }
}


void movement(int v, int v_w, float angle , float KP , float KD)
{
  //    KP = 0; KD = 0; v = 0; v_w = 0;
  if (v > 100)
    v = 100;
  //Serial.println(String ("TAN ANGLE ") + angle);
  BNO();

  v_x = v * cos(angle);
  v_y = v * sin(angle);

  error = curr_angle - targated_angle;
  if (error < -180)
    error = error + 360;
  else if (error > 180)
    error = error - 360;
  diff = error - prev_error;
  correction = (error * KP) + (diff * KD);
  prev_error = error;

  v_x = v * cos(angle);
  v_y = v * sin(angle);


//  v1 = ( -v_x + v_w) + correction ;
//  v2 = ((v_x * 0.5) - (v_y * 0.5377) + v_w)+ correction ;
//  v3 = -((v_y * 0.5377) + (v_x * 0.5) + v_w)+ correction  ;

  v1 = ((-0.667 * v_x)+ (v_w)) + correction ;
  v2 = ((v_x * 0.333) - (v_y * 0.5773) + v_w)+ correction ;
  v3 = ((v_x * 0.333) + (v_y * 0.5773) +  v_w)+ correction  ;

//  v1 = ( 0.6667*v_x - v_w) + correction ;
//  v2 = ((v_x * 0.3333) - (v_y * 0.57735) + v_w)+ correction ;
//  v3 = -((v_y * 0.3333) + (v_x * 0.57735) + v_w)+ correction  ;

  Serial.println(v1+String("       ")+v2+String("        ")+v3+String("     "));

  if (v1 >= 0)
  {
    analogWrite(RPWM_1, v1);
    analogWrite(LPWM_1, 0);
  }
  else
  {
    analogWrite(RPWM_1, 0);
    analogWrite(LPWM_1, -v1);
  }

  if (v2 >= 0)
  {
    analogWrite(RPWM_2, v2);
    analogWrite(LPWM_2, 0);
  }
  else
  {
    analogWrite(RPWM_2, 0);
    analogWrite(LPWM_2, -v2);
  }

  if (v3 >= 0)
  {
    analogWrite(RPWM_3, v3);
    analogWrite(LPWM_3, 0);
  }
  else
  {
    analogWrite(RPWM_3, 0);
    analogWrite(LPWM_3, -v3);
 
  }

}



void setup() {
  // put your setup code here, to run once:

   // put your setup code here, to run once:
  Serial1.begin(115200);
  Serial.begin(115200);
//  while(1){// for BNO data
//  BNO();
//}
  int z;
  byte l, m;
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));
 
 
}


void loop() {
  // put your main code here, to run repeatedly:
  Usb.Task();

  if (PS4.connected()) {

      KP = 0.2;//2.5
      KD = 0;//1

      if (PS4.getAnalogHat(LeftHatX) < 138 && PS4.getAnalogHat(LeftHatX) > 116)
        x = 0;
      else
        x  = map(PS4.getAnalogHat(LeftHatX), 0, 255, -127, 128);

      if (PS4.getAnalogHat(LeftHatY) < 138 && PS4.getAnalogHat(LeftHatY) > 116)
        y = 0;
      else
        y = map(PS4.getAnalogHat(LeftHatY), 0, 255, 128, -127);

      w = PS4.getAnalogButton(L2) - PS4.getAnalogButton(R2);

      angle = atan2(y, x);

      v = sqrt( (x * x) + (y * y) );
      v = map(v, 0, 150 , 0, 100);   //100

      v_w = map(w, 16, 255, 0, 43);

      if (PS4.getAnalogButton(L2) > 4 || PS4.getAnalogButton(R2) > 4)
        targated_angle = curr_angle;

      movement(v, v_w, angle, KP, KD);
  }

   

    else
    {
      v = 0;
      v_w = 0;
      KP = 0;
      KD = 0;
      movement(v, v_w, angle, KP, KD);
    }


    }

  
