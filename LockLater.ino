#include "I2Cdev.h"
#include "MPU6050.h"
#include <MFRC522.h>
#include <require_cpp11.h>

/*
     0 for ACC system
     1 for FC-51 system
*/
#define SYSTEM 1

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

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
#if !SYSTEM
MPU6050 accelgyro;
#endif
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

// 0 for BIKE STOP
// 1 for BIKE RUNNING
int State = 1;

#if SYSTEM // use FC-51
int globalDect;
#else
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t Pre_xyz[3];  // Previous x, y, z acceleration
int16_t Preg_xyz[3]; // Previous gx, gy, gz acceleration
#endif // SYSTEM

/* steper PIN */
int Pin1 = 8; //<--STEPPER PIN
int Pin2 = 7; //<--STEPPER PIN
int Pin3 = 6; //<--STEPPER PIN
int Pin4 = 5; //<--STEPPER PIN

/* system/debug used var */
int count = 0;
#if SYSTEM
int Wait = 30000;
#else
int Wait = 6000;
#endif
int countI = 0;
int countJ = 0;

/* TODO: Using RFID-key */
void Unlock()
{
     Serial.print("[LockLater]: U shall pass!!!\n");
     for (size_t i = 0; i < 100; i++)
     {
          digitalWrite(Pin1, HIGH);
          digitalWrite(Pin2, LOW);
          digitalWrite(Pin3, LOW);
          digitalWrite(Pin4, LOW);
          delay(1);
          digitalWrite(Pin1, HIGH);
          digitalWrite(Pin2, HIGH);
          digitalWrite(Pin3, LOW);
          digitalWrite(Pin4, LOW);
          delay(1);
          digitalWrite(Pin1, LOW);
          digitalWrite(Pin2, HIGH);
          digitalWrite(Pin3, LOW);
          digitalWrite(Pin4, LOW);
          delay(1);
          digitalWrite(Pin1, LOW);
          digitalWrite(Pin2, HIGH);
          digitalWrite(Pin3, HIGH);
          digitalWrite(Pin4, LOW);
          delay(1);
          digitalWrite(Pin1, LOW);
          digitalWrite(Pin2, LOW);
          digitalWrite(Pin3, HIGH);
          digitalWrite(Pin4, LOW);
          delay(1);
          digitalWrite(Pin1, LOW);
          digitalWrite(Pin2, LOW);
          digitalWrite(Pin3, HIGH);
          digitalWrite(Pin4, HIGH);
          delay(1);
          digitalWrite(Pin1, LOW);
          digitalWrite(Pin2, LOW);
          digitalWrite(Pin3, LOW);
          digitalWrite(Pin4, HIGH);
          delay(1);
          digitalWrite(Pin1, LOW);
          digitalWrite(Pin2, LOW);
          digitalWrite(Pin3, HIGH);
          digitalWrite(Pin4, HIGH);
          delay(1);
          digitalWrite(Pin1, LOW);
          digitalWrite(Pin2, LOW);
          digitalWrite(Pin3, LOW);
          digitalWrite(Pin4, LOW);
     }
     State = 1;
}

/* steper_rotation */
// TODO: set the right angle for steper_rotationer
void steper_rotation()
{
     Serial.print("LockLater Start!!! \n");
     for (size_t i = 0; i < 100; i++)
     {
          digitalWrite(Pin1, LOW);
          digitalWrite(Pin2, LOW);
          digitalWrite(Pin3, LOW);
          digitalWrite(Pin4, HIGH);
          delay(1);
          digitalWrite(Pin1, LOW);
          digitalWrite(Pin2, LOW);
          digitalWrite(Pin3, HIGH);
          digitalWrite(Pin4, HIGH);
          delay(1);
          digitalWrite(Pin1, LOW);
          digitalWrite(Pin2, LOW);
          digitalWrite(Pin3, HIGH);
          digitalWrite(Pin4, LOW);
          delay(1);
          digitalWrite(Pin1, LOW);
          digitalWrite(Pin2, HIGH);
          digitalWrite(Pin3, HIGH);
          digitalWrite(Pin4, LOW);
          delay(1);
          digitalWrite(Pin1, LOW);
          digitalWrite(Pin2, HIGH);
          digitalWrite(Pin3, LOW);
          digitalWrite(Pin4, LOW);
          delay(1);
          digitalWrite(Pin1, HIGH);
          digitalWrite(Pin2, HIGH);
          digitalWrite(Pin3, LOW);
          digitalWrite(Pin4, LOW);
          delay(1);
          digitalWrite(Pin1, HIGH);
          digitalWrite(Pin2, LOW);
          digitalWrite(Pin3, LOW);
          digitalWrite(Pin4, LOW);
          delay(1);
          digitalWrite(Pin1, HIGH);
          digitalWrite(Pin2, LOW);
          digitalWrite(Pin3, LOW);
          digitalWrite(Pin4, HIGH);
          delay(1);
          digitalWrite(Pin1, LOW);
          digitalWrite(Pin2, LOW);
          digitalWrite(Pin3, LOW);
          digitalWrite(Pin4, LOW);
     }
     Serial.print("LockLater done!!! \n");
}

/*   Keep sensoring
     false for unstable 
     true for stable*/
bool Sensoring()
{
#if SYSTEM // using FC-51
     for (int i = 0; i < 100; i++)
     {
          int localDect = digitalRead(A0);
          if (localDect != globalDect)
          {
               return false;
          }
     }
     return true;
#else
     accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
     int16_t ax_dif = abs(Pre_xyz[0] - ax);
     int16_t ay_dif = abs(Pre_xyz[1] - ay);
     int16_t az_dif = abs(Pre_xyz[2] - az);

#if SHOW_ACC
     Serial.print("ax=");
     Serial.print(ax);
     Serial.print("ay=");
     Serial.print(ay);
     Serial.print("az=");
     Serial.print(az);
     Serial.println("\n");
#endif
     Pre_xyz[0] = ax;
     Pre_xyz[1] = ay;
     Pre_xyz[2] = az;
     // int16_t gx_dif = abs(Preg_xyz[0] - gx);
     // int16_t gy_dif = abs(Preg_xyz[1] - gy);
     // int16_t gz_dif = abs(Preg_xyz[2] - gz);
     if (ax_dif > 1000 || ay_dif > 1000 || az_dif > 1000)
     {
          return false;
     }
     else // stable
     {
          return true;
     }
#endif
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
#if SYSTEM
     pinMode(A0, INPUT);
     pinMode(A1, OUTPUT);
     pinMode(A2, OUTPUT);
     pinMode(11, OUTPUT);
     digitalWrite(A2, HIGH);
     digitalWrite(A1, LOW);
#else
     // Initial Pre_xyz
     Pre_xyz[0] = 0.f;
     Pre_xyz[1] = 0.f;
     Pre_xyz[2] = 0.f;
#endif
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
#if SYSTEM
     Serial.println("Lock later setup done...");
#else
     // initialize device
     Serial.println("Initializing I2C devices...");
     accelgyro.initialize();

     // verify connection
     Serial.println("Testing device connections...");
     Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
#endif
}

void loop()
{
     countI++;

     digitalWrite(LED_PIN_UNLOCK, 0); // Unlock done

     // do sensor every 1000 count
     if (countI % 500 == 0 && State)
     {
          Serial.print("[LockLater]: BIKE MOVING\n");
#if SYSTEM
          globalDect = digitalRead(A0);
          bool bStable = Sensoring();
          
#else
          bool bStable = Sensoring();
#endif

          if (!bStable) // BIKE_RUNNING
          {
               digitalWrite(LED_PIN_BIKE_RUN, 1);
               count = 0;
               State = 1;
          }
          else if (bStable && State) // BIKE is STOP and may need LockLater
          {
               Serial.print("[LockLater]: BIKE might stop?\n");
               while (count < Wait)
               {
#if SYSTEM
                    globalDect = digitalRead(A0);
#endif
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
                         break;
                    }
               }
               if (count == Wait && State) // Stable last til counts done means "RUN-STOP",so BIKE STOP and needs LockLater
               {
                    Serial.print("[LockLater]: Oh, you certainly STOP!!!\n");
                    digitalWrite(LED_PIN_BIKE_RUN, 0);
                    digitalWrite(LED_PIN_UNLOCK, 1);
                    steper_rotation();
                    count = 0;
                    State = 0;
                    countI = 0;
               }
               else
               {
                    count = 0;
                    State = 1;
                    countI = 0;
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
                    Unlock();
               }
               else
               {
                    Serial.print("[LockLater]: HOW DARE U STEAL MY BIKE!!!!!!\n");
               }
          }
     }
}