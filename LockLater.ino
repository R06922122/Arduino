
#include "I2Cdev.h"
#include "MPU6050.h"
#include <MFRC522.h>
#include <require_cpp11.h>

// debug log flag
#define SHOW_ACC 0
#define RFID_CHECK 1

/* RFID related var */
#define RESET 9
#define SS 10
MFRC522 rc522(SS, RESET);
byte Legal_id[4] = {0xF2, 0x86, 0xFC, 0x16};

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define LED_PIN_BIKE_RUN 4
#define LED_PIN_UNLOCK 3 // Unlock LED

//MPU6050 accelgyro(0x69); // <-- use for AD0 high

// 0 for BIKE STOP
// 1 for BIKE RUNNING
int State = 1;

int globalDect = 2;

/* steper PIN */
int Pin1 = 8; //<--STEPPER PIN
int Pin2 = 7; //<--STEPPER PIN
int Pin3 = 6; //<--STEPPER PIN
int Pin4 = 5; //<--STEPPER PIN

/* system/debug used var */
int count = 0;
int Wait = 10000;
int countI = 0;
int countJ = 0;
int lockAngle = 1500;
/* TODO: Using RFID-key */

/* steper_rotation */
// TODO: set the right angle for steper_rotationer
void steper_rotation(bool dir)
{
     digitalWrite(LED_PIN_UNLOCK, 1);
     int _step = 0;
     Serial.print("LockLater Start!!! \n");
     for (size_t i = 0; i < lockAngle; i++)
     {
          switch (_step)
          {
          case 0:
               digitalWrite(Pin1, 0);
               digitalWrite(Pin2, 0);
               digitalWrite(Pin3, 0);
               digitalWrite(Pin4, 1);
               //1
               break;
          case 1:
               digitalWrite(Pin1, 0);
               digitalWrite(Pin2, 0);
               digitalWrite(Pin3, 1);
               digitalWrite(Pin4, 1);
               //2
               break;
          case 2:
               digitalWrite(Pin1, 0);
               digitalWrite(Pin2, 0);
               digitalWrite(Pin3, 1);
               digitalWrite(Pin4, 0);
               //3
               break;

          case 3:
               digitalWrite(Pin1, 0);
               digitalWrite(Pin2, 1);
               digitalWrite(Pin3, 1);
               digitalWrite(Pin4, 0);
               //4
               break;
          case 4:
               digitalWrite(Pin1, 0);
               digitalWrite(Pin2, 1);
               digitalWrite(Pin3, 0);
               digitalWrite(Pin4, 0);
               //7
               break;
          case 5:
               digitalWrite(Pin1, 1);
               digitalWrite(Pin2, 1);
               digitalWrite(Pin3, 0);
               digitalWrite(Pin4, 0);
               //8
               break;
          case 6:
               digitalWrite(Pin1, 1);
               digitalWrite(Pin2, 0);
               digitalWrite(Pin3, 0);
               digitalWrite(Pin4, 0);
               //15
               break;
          case 7:
               digitalWrite(Pin1, 1);
               digitalWrite(Pin2, 0);
               digitalWrite(Pin3, 0);
               digitalWrite(Pin4, 1);
               //14
               break;
          default:
               digitalWrite(Pin1, 0);
               digitalWrite(Pin2, 0);
               digitalWrite(Pin3, 0);
               digitalWrite(Pin4, 0);
               //0
               break;
          }
          if (dir)
          {
               _step++;
          }
          else
          {
               _step--;
          }
          if (_step > 7)
          {
               _step = 0;
          }
          if (_step < 0)
          {
               _step = 7;
          }
          delay(1);
     } // for-loop end
     Serial.print("LockLater done!!! \n");
     digitalWrite(LED_PIN_UNLOCK, 0);
}

/*   Keep sensoring
     false for unstable 
     true for stable*/
bool Sensoring()
{
     for (int i = 0; i < 100; i++)
     {
          int localDect = digitalRead(A0);
          if (localDect != globalDect)
          {
               return false;
          }
     }
     return true;
}

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

// #define LED_PIN 13
// bool blinkState = false;

void setup()
{

     // set up the stepper pin
     pinMode(Pin1, OUTPUT);
     pinMode(Pin2, OUTPUT);
     pinMode(Pin3, OUTPUT);
     pinMode(Pin4, OUTPUT);
     // pinMode(LED_PIN, OUTPUT);
     pinMode(LED_PIN_BIKE_RUN, OUTPUT);
     pinMode(LED_PIN_UNLOCK, OUTPUT);

     pinMode(A0, INPUT);
     pinMode(A1, OUTPUT);
     pinMode(A2, OUTPUT);
     pinMode(11, OUTPUT);
     digitalWrite(A2, HIGH);
     digitalWrite(A1, LOW);

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
     Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
     Fastwire::setup(400, true);
#endif

     // initialize serial communication
     // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
     // it's really up to you depending on your project)
     Serial.begin(38400);

     // RFID related code
     SPI.begin();
     rc522.PCD_Init();

     Serial.println("Lock later setup done...");
}

void loop()
{
     countI++;
     if (State)
          digitalWrite(LED_PIN_BIKE_RUN, 1);
     // do sensor every 1000 count
     if (countI % 250 == 0 && State)
     {
          if (State)
          {
               digitalWrite(LED_PIN_BIKE_RUN, 1);
          }
          else
          {
               digitalWrite(LED_PIN_BIKE_RUN, 0);
          }
          Serial.print("[LockLater]: BIKE MOVING\n");
          globalDect = digitalRead(A0);
          bool bStable = Sensoring();

          if (!bStable) // BIKE_RUNNING
          {
               // digitalWrite(LED_PIN_BIKE_RUN, 1);
               countI = 0;
               count = 0;
               State = 1;
               digitalWrite(LED_PIN_BIKE_RUN, 1);
          }
          else if (bStable && State) // BIKE is STOP and may need LockLater
          {
               Serial.print("[LockLater]: BIKE might stop?\n");
               while (count < Wait)
               {
                    globalDect = digitalRead(A0);
                    if (Sensoring())
                    {
                         count++;
                         if (!(count % 500)) // Don't print too much
                         {
                              Serial.print(count);
                              Serial.print("\n");
                         }
                    }
                    else
                    {
                         Serial.print("[LockLater]: Oh, you just chating. MOVE!!! MOVE!!! MOVE!!!\n");
                         State = 1;
                         digitalWrite(LED_PIN_BIKE_RUN, 1);
                         break;
                    }
               }
               if (count == Wait && State) // Stable last til counts done means "RUN->STOP",so BIKE STOP and needs LockLater
               {
                    Serial.print("[LockLater]: Oh, you certainly STOP!!!\n");
                    digitalWrite(LED_PIN_BIKE_RUN, 0);
                    steper_rotation(true);
                    count = 0;
                    countI = 0;
                    State = 0;
                    digitalWrite(LED_PIN_BIKE_RUN, 0);
               }
               else
               {
                    // digitalWrite(LED_PIN_BIKE_RUN, 1);
                    count = 0;
                    countI = 0;
                    State = 1;
                    digitalWrite(LED_PIN_BIKE_RUN, 1);
               }
          }
     }

     // Simulate Unlock with RFID
     if (rc522.PICC_IsNewCardPresent() && !State)
     {
          bool bAcc_aproval = true;
          if (rc522.PICC_ReadCardSerial())
          {
               byte *id = rc522.uid.uidByte;
               int len = rc522.uid.size;
#if RFID_CHECK
               Serial.print("Here comes the challenger!!!\n");
               Serial.print("challenger ID:\n");
               for (int i = 0; i < len; i++)
               {
                    Serial.print("0x");
                    Serial.print(id[i], HEX);
                    Serial.print(" ");
               }
               Serial.print("\n");
               Serial.print("king ID:\n");
               for (int i = 0; i < len; i++)
               {
                    Serial.print("0x");
                    Serial.print(Legal_id[i], HEX);
                    Serial.print(" ");
               }
#endif
               for (int i = 0; i < 4; i++)
               {
                    if (id[i] != Legal_id[i])
                    {
                         bAcc_aproval = false;
                    }
               }
               if (bAcc_aproval)
               {
                    steper_rotation(false);
                    State = 1;
                    digitalWrite(LED_PIN_BIKE_RUN, 1);
               }
               else
               {
                    Serial.print("[LockLater]: HOW DARE U STEAL MY BIKE!!!!!!\n");
               }
          }
     }
}
