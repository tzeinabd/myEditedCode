
//MSE 2202 
//Western Engineering base code
//2020 05 13 E J Porter


/*
  esp32                                           MSE-DuinoV2
  pins         description                        Brd Jumpers /Labels                                                                  User (Fill in chart with user PIN usage) 
  1             3v3                               PWR 3V3                                                                              3V3
  2             gnd                               GND                                                                                  GND
  3             GPIO15/AD2_3/T3/SD_CMD/           D15 (has connections in both 5V and 3V areas)                    
  4             GPIO2/AD2_2/T2/SD_D0              D2(has connections in both 5V and 3V areas)  /INDICATORLED ( On ESP32 board )        Heartbeat LED
  5             GPIO4/AD2_0/T0/SD_D1              D4(has connections in both 5V and 3V areas)                                          Left Motor, Channel A
  6             GPIO16/RX2                        Slide Switch S1b                                                                     IR Receiver
  7             GPIO17/TX2                        Slide Switch S2b                                                                     Left Encoder, Channel A
  8             GPIO5                             D5 (has connections in both 5V and 3V areas)                                         Left Encoder, Channel B
  9             GPIO18                            D18 (has connections in both 5V and 3V areas)                                        Left Motor, Channel B
  10            GPIO19/CTS0                       D19 (has connections in both 5V and 3V areas)                                        Right Motor, Channel A
  11            GPIO21                            D21/I2C_DA  
  12            GPIO3/RX0                         RX0
  13            GPIO1//TX0                        TX0
  14            GPIO22/RTS1                       D22/I2C_CLK
  15            GPIO23                            D23 (has connections in both 5V and 3V areas)  
  16            EN                                JP4 (Labeled - RST) for reseting ESP32
  17            GPI36/VP/AD1_0                    AD0                   
  18            GPI39/VN/AD1_3/                   AD3
  19            GPI34/AD1_6/                      AD6
  20            GPI35/AD1_7                       Potentiometer R2 / AD7
  21            GPIO32/AD1_4/T9                   Potentiometer R1 / AD4                                                               Pot 1 (R1)
  22            GPIO33/AD1_5/T8                   IMon/D33  monitor board current
  23            GPIO25/AD2_8/DAC1                 SK6812 Smart LEDs / D25                                                              Smart LEDs
  24            GPIO26/A2_9/DAC2                  Push Button PB2                                                                      Limit switch
  25            GPIO27/AD2_7/T7                   Push Button PB1                                                                      PB1
  26            GPOP14/AD2_6/T6/SD_CLK            Slide Switch S2a                                                                     Right Encoder, Channel A
  27            GPIO12/AD2_5/T5/SD_D2/            D12(has connections in both 5V and 3V areas)                                         Right Motor, Channel B
  28            GPIO13/AD2_4/T4/SD_D3/            Slide Switch S1a                                                                     Right Encoder, Channel B
  29            GND                               GND                                                                                  GND
  30            VIN                               PWR 5V t 7V                                                                          PWR 5V to 7V
*/


//Pin assignments
const int ciHeartbeatLED = 2;
const int ciPB1 = 27;     
const int ciPB2 = 26;      
const int ciPot1 = A4;    //GPIO 32  - when JP2 has jumper installed Analog pin AD4 is connected to Poteniometer R1
const int ciLimitSwitch = 26;
const int ciIRDetector = 16;
const int ciMotorLeftA = 4;
const int ciMotorLeftB = 18;
const int ciMotorRightA = 19;
const int ciMotorRightB = 12;
const int ciEncoderLeftA = 17;
const int ciEncoderLeftB = 5;
const int ciEncoderRightA = 14;
const int ciEncoderRightB = 13;
const int ciSmartLED = 25;
const int ciStepperMotorDir = 22;
const int ciStepperMotorStep = 21;
//const int servoPin = 15;

volatile uint32_t vui32test1;
volatile uint32_t vui32test2;

#include "0_Core_Zero.h"

#include <esp_task_wdt.h>

#include <Adafruit_NeoPixel.h>
#include <Math.h>
#include "Motion.h";
#include "MyWEBserver.h"
#include "BreakPoint.h"
#include "WDT.h";
//#include <Servo.h>

//for servo
/*long degreesToDutyCycle(int deg) {
  const long minDutyCycle = 1675;            // duty cycle for 0 degrees
  const long maxDutyCycle = 8050;            // duty cycle for 180 degrees

  long dutyCycle = map(deg, 0, 180, minDutyCycle, maxDutyCycle);  // convert to duty cycle

  return dutyCycle;
}
*/


//################################################################################

void loopWEBServerButtonresponce(void);

const int CR1_ciMainTimer =  1000;
const int CR1_ciHeartbeatInterval = 500;
 int CR1_ciMotorRunTime = 1000;
const long CR1_clDebounceDelay = 50;
const long CR1_clReadTimeout = 220;

const uint8_t ci8RightTurn = 18;
const uint8_t ci8LeftTurn = 17;

unsigned char CR1_ucMainTimerCaseCore1;
uint8_t CR1_ui8LimitSwitch;

uint8_t CR1_ui8IRDatum;
uint8_t CR1_ui8WheelSpeed;
uint8_t CR1_ui8LeftWheelSpeed;
uint8_t CR1_ui8RightWheelSpeed;

uint32_t CR1_u32Now;
uint32_t CR1_u32Last;
uint32_t CR1_u32Temp;
uint32_t CR1_u32Avg;

unsigned long CR1_ulLastDebounceTime;
unsigned long CR1_ulLastByteTime;

unsigned long CR1_ulMainTimerPrevious;
unsigned long CR1_ulMainTimerNow;

unsigned long CR1_ulMotorTimerPrevious;
unsigned long CR1_ulMotorTimerNow;
unsigned char ucMotorStateIndex = 0;

unsigned long CR1_ulHeartbeatTimerPrevious;
unsigned long CR1_ulHeartbeatTimerNow;

boolean btHeartbeat = true;
boolean btRun = false;
boolean btToggle = true;
int iButtonState;
int iLastButtonState = HIGH;

// Declare our SK6812 SMART LED object:
Adafruit_NeoPixel SmartLEDs(2, 25, NEO_GRB + NEO_KHZ400);
// Argument 1 = Number of LEDs (pixels) in use
// Argument 2 = ESP32 pin number 
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

//Servo servo1;
void setup() {
   Serial.begin(115200); 
   Serial2.begin(2400, SERIAL_8N1, ciIRDetector);  // IRDetector on RX2 receiving 8-bit words at 2400 baud


   
   Core_ZEROInit();

   WDT_EnableFastWatchDogCore1();
   WDT_ResetCore1();
   WDT_vfFastWDTWarningCore1[0] = 0;
   WDT_vfFastWDTWarningCore1[1] = 0;
   WDT_vfFastWDTWarningCore1[2] = 0;
   WDT_vfFastWDTWarningCore1[3] = 0;
   WDT_ResetCore1();
   WDT_vfFastWDTWarningCore1[4] = 0;
   WDT_vfFastWDTWarningCore1[5] = 0;
   WDT_vfFastWDTWarningCore1[6] = 0;
   WDT_vfFastWDTWarningCore1[7] = 0;
   WDT_ResetCore1();
   WDT_vfFastWDTWarningCore1[8] = 0;
   WDT_vfFastWDTWarningCore1[9] = 0;
   WDT_ResetCore1(); 

   setupMotion();
   pinMode(ciHeartbeatLED, OUTPUT);
   pinMode(ciPB1, INPUT_PULLUP);
   pinMode(ciLimitSwitch, INPUT_PULLUP);

   SmartLEDs.begin();                          // Initialize Smart LEDs object (required)
   SmartLEDs.clear();                          // Set all pixel colours to off
   SmartLEDs.show();                           // Send the updated pixel colours to the hardware

//   servo1.attach(servoPin);
}

void loop()
{
  //WSVR_BreakPoint(1);

   //average the encoder tick times
   ENC_Averaging();

  int iButtonValue = digitalRead(ciPB1);       // read value of push button 1
  if (iButtonValue != iLastButtonState) {      // if value has changed
     CR1_ulLastDebounceTime = millis();        // reset the debouncing timer
  }

  if ((millis() - CR1_ulLastDebounceTime) > CR1_clDebounceDelay) {
    if (iButtonValue != iButtonState) {        // if the button state has changed
    iButtonState = iButtonValue;               // update current button state

     // only toggle the run condition if the new button state is LOW
     if (iButtonState == LOW)
     {
       ENC_ClearLeftOdometer();
       ENC_ClearRightOdometer();
       btRun = !btRun;
        Serial.println(btRun);
       // if stopping, reset motor states and stop motors
       if(!btRun)
       {
          ucMotorStateIndex = 0; 
          ucMotorState = 0;
          move(0);
       }
      
     }
   }
 }
 iLastButtonState = iButtonValue;             // store button state

 if(!digitalRead(ciLimitSwitch))
 {
  btRun = 0; //if limit switch is pressed stop bot
  ucMotorStateIndex = 0;
  ucMotorState = 0;
  move(0);
 }
 
 if (Serial2.available() > 0) {               // check for incoming data
    CR1_ui8IRDatum = Serial2.read();          // read the incoming byte
// Serial.println(iIncomingByte, HEX);        // uncomment to output received character
    CR1_ulLastByteTime = millis();            // capture time last byte was received
 }
 else
 {
    // check to see if elapsed time exceeds allowable timeout
    if (millis() - CR1_ulLastByteTime > CR1_clReadTimeout) {
      CR1_ui8IRDatum = 0;                     // if so, clear incoming byte
    }
 }
 CR1_ulMainTimerNow = micros();
 if(CR1_ulMainTimerNow - CR1_ulMainTimerPrevious >= CR1_ciMainTimer)
 {
   WDT_ResetCore1(); 
   WDT_ucCaseIndexCore0 = CR0_ucMainTimerCaseCore0;
   
   CR1_ulMainTimerPrevious = CR1_ulMainTimerNow;
    
   switch(CR1_ucMainTimerCaseCore1)  //full switch run through is 1mS
   {
    //###############################################################################
    case 0: 
    {
      
      if(btRun)
      {
       CR1_ulMotorTimerNow = millis();
       if(CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime)   
       {   
         CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
         switch(ucMotorStateIndex)
         {
          case 0:
          {
            ucMotorStateIndex = 1;
            ucMotorState = 0;
            move(0);
            break;
          }
           case 1:
          {
            ENC_SetDistance(50,50);
            ucMotorState = 1;   //forward
            CR1_ui8LeftWheelSpeed = 200;
            CR1_ui8RightWheelSpeed = 250; 
            ucMotorStateIndex = 2;            
            break;
          }
           case 2:
          {
           ucMotorStateIndex = 4 ;
           ucMotorState = 0;
           move(0);
           break;
          }
         case 3:
        {
            ENC_SetDistance(20,20);
            ucMotorState = 1;   //forward
            CR1_ui8LeftWheelSpeed = 200;
            CR1_ui8RightWheelSpeed = 250; 
            ucMotorStateIndex = 4;            
            break;
          }
           case 4:
          {
            ucMotorStateIndex = 5;
            ucMotorState = 0;
            move(0);
            break;
          }
         case 5:
          {
           ENC_SetDistance(50,50);
            ucMotorState = 1;   //forward
            CR1_ui8LeftWheelSpeed = 200;
            CR1_ui8RightWheelSpeed = 250; 
            ucMotorStateIndex = 6;            
            break;
          }
     
         case 6:
          {
            ucMotorStateIndex = 7;
            ucMotorState = 0;
            move(0);
            break;
          }
           case 7:
          {
       
            ENC_SetDistance(-(ci8LeftTurn), ci8LeftTurn);
            CR1_ui8LeftWheelSpeed = 140;
            CR1_ui8RightWheelSpeed = 250;
            ucMotorStateIndex = 8;
            ucMotorState = 2;  //left
            break;
           
          }
          case 8:
          {
            ucMotorStateIndex = 9;
            ucMotorState = 0;
            move(0);
            break;
          }
       case 9:
          {
           ENC_SetDistance(50,50);
            ucMotorState = 1;   //forward
            CR1_ui8LeftWheelSpeed = 200;
            CR1_ui8RightWheelSpeed = 200; 
            ucMotorStateIndex =10;            
            break;  
           
          }
          case 10:
          {
            ucMotorStateIndex = 11;
            ucMotorState = 0;
            move(0);
            break;
          }
       case 11:
          {
           
            
            ENC_SetDistance(ci8RightTurn,-(ci8RightTurn));
            CR1_ui8LeftWheelSpeed = 140;
            CR1_ui8RightWheelSpeed = 250;
            ucMotorStateIndex = 12;
            ucMotorState = 3;  //right 
            break;  
          }
          case 12:
          {
           ucMotorStateIndex = 13;
           ucMotorState = 0;
           move(0);
           break;
          }
         case 13:
          {
           ENC_SetDistance(50,50);
            ucMotorState = 1;   //forward
            CR1_ui8LeftWheelSpeed = 200;
            CR1_ui8RightWheelSpeed = 200; 
            ucMotorStateIndex =14;            
            break;
       
          }
          case 14:
          {
            ucMotorStateIndex = 15;
            ucMotorState = 0;
            move(0);
            break;
          }
      case 15:
          {  
              ENC_SetDistance(70,70);
            ucMotorState = 1;   //forward
            CR1_ui8LeftWheelSpeed = 200;
            CR1_ui8RightWheelSpeed = 250; 
            ucMotorStateIndex =16;            
            break;  
          }
          
           case 16:
          {
            ucMotorStateIndex = 17;
            ucMotorState = 0;
            move(0);
            break;
          }
        case 17:
          {  
           
             ENC_SetDistance(ci8RightTurn,-(ci8RightTurn));
            CR1_ui8LeftWheelSpeed = 140;
            CR1_ui8RightWheelSpeed = 250;
            ucMotorStateIndex = 18;
            ucMotorState = 3;  //right 
            break;
            
            break;
          }
          
           case 18:
          {
            ucMotorStateIndex = 19;
            ucMotorState = 0;
            move(0);
            break;
          }
           case 19:
          {
           
          ENC_SetDistance(50, 50);
            ucMotorState = 1;   //forward
            CR1_ui8LeftWheelSpeed = 250;
            CR1_ui8RightWheelSpeed = 140; 
            ucMotorStateIndex =  20;
            
            break;
            
          }
           case 20:
          {
            ucMotorStateIndex = 21;
            ucMotorState = 0;
            move(0);
            break;
          }
         case 21:
           {
                      
                     ENC_SetDistance(50, 50);
            ucMotorState = 1;   //forward
            CR1_ui8LeftWheelSpeed = 200;
            CR1_ui8RightWheelSpeed = 250; 
            ucMotorStateIndex =  22;
            break;
         }
         case 22:
          {
            ucMotorStateIndex = 23;
            ucMotorState = 0;
            move(0);     
            break;
          }
     /*    case 23:
          {
    ENC_SetDistance(50, 50);
            ucMotorState = 1;   //forward
            CR1_ui8LeftWheelSpeed = 250;
            CR1_ui8RightWheelSpeed = 140; 
            ucMotorStateIndex =  24;    
             
           break;
          }
          case 24:
          {
            ucMotorStateIndex = 25;
            ucMotorState = 0;
            move(0);
            break;
          }
          case 25:
          {

             ENC_SetDistance(50, 50);
            ucMotorState = 1;   //forward
            CR1_ui8LeftWheelSpeed = 250;
            CR1_ui8RightWheelSpeed = 140; 
            ucMotorStateIndex =  26;
             
           break;
          }
          case 26:
          {
            ucMotorStateIndex = 27;
            ucMotorState = 0;      
            break;
          }
      /*     case 27:
          {

    
            ENC_SetDistance(30, 30);
            ucMotorState = 1;   //reverse
            CR1_ui8LeftWheelSpeed = 250;
            CR1_ui8RightWheelSpeed = 140; 
            ucMotorStateIndex =  28;
            break;
          }
          case 28:
          {
            ucMotorStateIndex = 29;
            ucMotorState = 0;
            move(0);
            break;
        }
        case 29:
        {
            ENC_SetDistance(70, 70);
            ucMotorState = 1;   //reverse
            CR1_ui8LeftWheelSpeed = 250;
            CR1_ui8RightWheelSpeed = 140; 
            ucMotorStateIndex =  30;
            break;
        }
        case 30:
        {
          ucMotorStateIndex = 31;
          ucMotorState = 0;
          move(0);
                      break;

        }
case 31:        
{

            ENC_SetDistance(50, 50);
            ucMotorState = 1;   //reverse
            CR1_ui8LeftWheelSpeed = 250;
            CR1_ui8RightWheelSpeed = 140; 
            ucMotorStateIndex =  32;
            break;
        }
        case 32:
        {
          ucMotorStateIndex = 33;
          ucMotorState = 0;
          move(0);
          break;
        }
       case 33:
        {
         
           ENC_SetDistance(ci8RightTurn,-(ci8RightTurn));
            CR1_ui8LeftWheelSpeed = 250;
            CR1_ui8RightWheelSpeed = 140;
            ucMotorStateIndex = 34;
            ucMotorState = 3;  //right 
            break;
       
      }
       case 34:
        {
          ucMotorStateIndex = 35;
          ucMotorState = 0;
          move(0);
                      break;

        }
   /*     case 35:
        {
         
           ENC_SetDistance(20, 20);
            ucMotorState = 1;   //forward
            CR1_ui8LeftWheelSpeed = 250;
            CR1_ui8RightWheelSpeed = 140; 
            ucMotorStateIndex =  36;
            break;
       
      }
      case 36:
        {
          ucMotorStateIndex = 37;
          ucMotorState = 0;
          move(0);
                      break;
         }
         case 37:
         {
          ENC_SetDistance(-(ci8LeftTurn), ci8LeftTurn);
            CR1_ui8LeftWheelSpeed = 250;
            CR1_ui8RightWheelSpeed = 140;
            ucMotorStateIndex = 38;
            ucMotorState = 2;  //left
            break;
         }
         case 38:
        {
          ucMotorStateIndex = 39;
          ucMotorState = 0;
          move(0);
                      break;
         }
          case 39:
        {
         
            ENC_SetDistance(20, 20);
            ucMotorState = 1;   //forward
            CR1_ui8LeftWheelSpeed = 250;
            CR1_ui8RightWheelSpeed = 140; 
            ucMotorStateIndex =  42;
            break;
       
      }
       case 40:
        {
          ucMotorStateIndex = 41;
          ucMotorState = 0;
          move(0);
                      break;
         }
          case 41:
         {
          ENC_SetDistance(20, 20);
            ucMotorState = 1;   //forward
            CR1_ui8LeftWheelSpeed = 250;
            CR1_ui8RightWheelSpeed = 140; 
            ucMotorStateIndex =  42;
            break;
         }
         case 42:
        {
          ucMotorStateIndex = 43;
          ucMotorState = 0;
          move(0);
                      break;
         }
          case 43:
        {
         
           ENC_SetDistance(ci8RightTurn,-(ci8RightTurn));
            CR1_ui8LeftWheelSpeed = 250;
            CR1_ui8RightWheelSpeed = 140;
            ucMotorStateIndex = 44;
            ucMotorState = 3;  //right 
            break;
       
      }
       case 44:
        {
          ucMotorStateIndex = 45;
          ucMotorState = 0;
          move(0);
                      break;
         }
           case 45:
         {
          ENC_SetDistance(70, 70);
            ucMotorState = 1;   //forward
            CR1_ui8LeftWheelSpeed = 250;
            CR1_ui8RightWheelSpeed = 140; 
            ucMotorStateIndex =  46;
            break;
         }
          case 46:
        {
          ucMotorStateIndex = 51;
          ucMotorState = 0;
          move(0);
                      break;
         } 
         case 51:
         {
          ENC_SetDistance(20, 20);
            ucMotorState = 1;   //forward
            CR1_ui8LeftWheelSpeed = 250;
            CR1_ui8RightWheelSpeed = 140; 
            ucMotorStateIndex =  52;
            break;
         }
          case 52:
        {
          ucMotorStateIndex = 53;
          ucMotorState = 0;
          move(0);
                      break;
        }
        case 53:
        {
            ENC_SetDistance(ci8RightTurn,-(ci8RightTurn));
            CR1_ui8LeftWheelSpeed = 250;
            CR1_ui8RightWheelSpeed = 140;
            ucMotorStateIndex = 54;
            ucMotorState = 3;  //right 
            break;
        }
        case 54:
        {
          ucMotorStateIndex = 55;
          ucMotorState = 0;
          move(0);
                      break;
        }
        case 55:
        {
          ENC_SetDistance(20, 20);
            ucMotorState = 1;   //forward
            CR1_ui8LeftWheelSpeed = 250;
            CR1_ui8RightWheelSpeed = 140; 
            ucMotorStateIndex =  56;
            break;
        }
        
          case 56:
         {
          ucMotorStateIndex = 48;
          ucMotorState = 0;
          move(0);
                      break;
         }
          case 48:
        {
          ucMotorStateIndex = 49;
          ucMotorState = 0;
          move(0);
                      break;
         } 
       /*   case 49:
        {
         
            ENC_SetDistance(70, 70);
            ucMotorState = 1;   //forward
            CR1_ui8LeftWheelSpeed = 250;
            CR1_ui8RightWheelSpeed = 140; 
            ucMotorStateIndex =  50;
            break;
       
      }
       case 50:
        {
          ucMotorStateIndex = 50;
          ucMotorState = 0;
          move(0);
                      break;
         }*/
         
       }
         }
      }
       
      CR1_ucMainTimerCaseCore1 = 1;
      
      break;
    }
    
    //###############################################################################
    case 1: 
    {
      //read pot 1 for motor speeds 
      CR1_ui8WheelSpeed = map(analogRead(ciPot1), 0, 4096, 130, 255);  // adjust to range that will produce motion
      
      CR1_ucMainTimerCaseCore1 = 2;
      break;
    }
    //###############################################################################
    case 2: 
    {
     // asm volatile("esync; rsr %0,ccount":"=a" (vui32test1)); // @ 240mHz clock each tick is ~4nS 
     
   //   asm volatile("esync; rsr %0,ccount":"=a" (vui32test2)); // @ 240mHz clock each tick is ~4nS 
     
      CR1_ucMainTimerCaseCore1 = 3;
      break;
    }
    //###############################################################################
    case 3: 
    {
      //move bot X number of odometer ticks
      if(ENC_ISMotorRunning())
      {
        MoveTo(ucMotorState, CR1_ui8LeftWheelSpeed,CR1_ui8LeftWheelSpeed);
      }
   
      CR1_ucMainTimerCaseCore1 = 4;
      break;
    }
    //###############################################################################
    case 4:   
    {
    
      CR1_ucMainTimerCaseCore1 = 5;
      break;
    }
    //###############################################################################
    case 5: 
    {
      
     
      CR1_ucMainTimerCaseCore1 = 6;
      break;
    }
    //###############################################################################
    case 6:
    {
  
    
      CR1_ucMainTimerCaseCore1 = 7;
      break;
    }
    //###############################################################################
    case 7: 
    {
       if (CR1_ui8IRDatum == 0x55) {                // if proper character is seen
         SmartLEDs.setPixelColor(0,0,25,0);         // make LED1 green with 10% intensity
       }
       else if (CR1_ui8IRDatum == 0x41) {           // if "hit" character is seen
         SmartLEDs.setPixelColor(0,25,0,25);        // make LED1 purple with 10% intensity
       }
       else {                                       // otherwise
         SmartLEDs.setPixelColor(0,25,0,0);         // make LED1 red with 10% intensity
       }
       SmartLEDs.show();                            // send updated colour to LEDs
          
      CR1_ucMainTimerCaseCore1 = 8;
      break;
    }
    //###############################################################################
    case 8: 
    {
    
      CR1_ucMainTimerCaseCore1 = 9;
      break;
    }
    //###############################################################################
    case 9: 
    {
  
      CR1_ucMainTimerCaseCore1 = 0;
      break;
    }

  }
 }

 // Heartbeat LED
 CR1_ulHeartbeatTimerNow = millis();
 if(CR1_ulHeartbeatTimerNow - CR1_ulHeartbeatTimerPrevious >= CR1_ciHeartbeatInterval)
 {
    CR1_ulHeartbeatTimerPrevious = CR1_ulHeartbeatTimerNow;
    btHeartbeat = !btHeartbeat;
    digitalWrite(ciHeartbeatLED, btHeartbeat);
   // Serial.println((vui32test2 - vui32test1)* 3 );
 }

}
