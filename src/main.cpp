#include <Arduino.h>
#include <U8g2lib.h>
#include<string.h>
#include <STM32FreeRTOS.h>
#include "Knob.h"

enum notes {
  C  = 0,
  Cs = 1,
  D  = 2,
  Ds = 3,
  E  = 4,
  F  = 5,
  Fs = 6,
  G  = 7,
  Gs = 8,
  A  = 9,
  As = 10,
  B  = 11,
  None = 12
};

//Constants
const uint32_t interval = 100; //Display update interval

//Pin definitions
//Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

//Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

//Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

//Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

//Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

const int32_t stepSizes [] = {51076057,54113197,57330935,60740010, 64351799, 68178356, 72232452, 76527617, 81078186, 85899346,91007187,96418756};
const int32_t averages [] = {1072095723,	1055207342,	1060528483,	1062773477,	1061711081,	1056367844,	1047042225,	1071042264,	1053813723,	1030792152,	1046317902,	1060317060};
const String noteNames [] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
volatile uint8_t keyArray[7];

// volatile int32_t currentStepSize;
SemaphoreHandle_t keyArrayMutex;
// volatile int32_t currentStepSize_1;
// volatile int32_t currentStepSize_2;
// volatile int32_t currentAverage_1;
// volatile int32_t currentAverage_2;

const int32_t int32_max = 2147483647;
const uint8_t n = 4;
volatile int32_t currentStepSize[n];
volatile int32_t currentAverage[n];

// Knob
Knob knob0(3, 0, 0);
Knob knob1(16, 0, 1);
Knob knob2(16, 0, 2);
Knob knob3(16, 0, 3);

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

uint8_t readCols() {
      uint8_t cols = 0;
      cols |= digitalRead(C0_PIN) << 0;
      cols |= digitalRead(C1_PIN) << 1;
      cols |= digitalRead(C2_PIN) << 2;
      cols |= digitalRead(C3_PIN) << 3;
      return cols;
}

void setRow(uint8_t rowIdx){
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, rowIdx & 0x01);
      digitalWrite(RA1_PIN, rowIdx & 0x02);
      digitalWrite(RA2_PIN, rowIdx & 0x04);
      digitalWrite(REN_PIN,HIGH);
}

void findKeywithFunc(void (*func)(notes*)) {
      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      uint8_t CDs = keyArray[0];
      uint8_t EG = keyArray[1];
      uint8_t GsB =keyArray[2];
      xSemaphoreGive(keyArrayMutex);
      bool C  = (~CDs >> 0) & B1;
      bool Cs = (~CDs >> 1) & B1;
      bool D  = (~CDs >> 2) & B1;
      bool Ds = (~CDs >> 3) & B1;
      bool E  = (~EG  >> 0) & B1;
      bool F  = (~EG  >> 1) & B1;
      bool Fs = (~EG  >> 2) & B1;
      bool G  = (~EG  >> 3) & B1;
      bool Gs = (~GsB >> 0) & B1;
      bool A  = (~GsB >> 1) & B1;
      bool As = (~GsB >> 2) & B1;
      bool B  = (~GsB >> 3) & B1;
      

      bool bool_array [] = {C, Cs, D, Ds, E, F, Fs, G, Gs, A, As, B};
      notes localNotesPressed [] = {None,None,None,None};

      // Return an array of 2 notes
      bool current_key = false;
      notes current_note = None;
      
      for (int i=0; i<12; i++) {
        current_key = bool_array[i];
        if (current_key) {current_note = ((notes)i);}
        if (current_note != None) {
          if (localNotesPressed[0] == None) {
            localNotesPressed[0] = current_note;
          } else if (localNotesPressed[1] == None){
            localNotesPressed[1] = current_note;
          } else if (localNotesPressed[2] == None){
            localNotesPressed[2] = current_note;
          } else if (localNotesPressed[3] == None){
            localNotesPressed[3] = current_note;
          } /* else if (localNotesPressed[4] == None){
            localNotesPressed[4] = current_note;
          } else if (localNotesPressed[5] == None){
            localNotesPressed[5] = current_note;
          } else if (localNotesPressed[6] == None){
            localNotesPressed[6] = current_note;
          } else if (localNotesPressed[7] == None){
            localNotesPressed[7] = current_note;
          } else if (localNotesPressed[8] == None){
            localNotesPressed[8] = current_note;
          } else if (localNotesPressed[9] == None){
            localNotesPressed[9] = current_note;
          } */ else {
            //ignore for now
          }
        }
        current_note = None;
      }

      func(localNotesPressed);

      //func(localNotesPressed[0], localNotesPressed[1]);
      // __atomic_store_n(&localNotesPressed,notesPressed,__ATOMIC_RELAXED);
      /*
      if(C)  {func((notes)0);}
      if(Cs) {func((notes)1);}
      if(D)  {func((notes)2);}
      if(Ds) {func((notes)3);}
      if(E)  {func((notes)4);}
      if(F)  {func((notes)5);}
      if(Fs) {func((notes)6);}
      if(G)  {func((notes)7);}
      if(Gs) {func((notes)8);}
      if(A)  {func((notes)9);}
      if(As) {func((notes)10);}
      if(B)  {func((notes)11);}
      if(!C && !Cs && !D && !Ds && !E && !F && !Fs && !G && !Gs && !A && !As && !B) {
              func((notes)12);
      }
      */
}

void setStepSize(notes* note_list) {

      for (int i=0; i<n; i++) {
        if (note_list[i] != None) {
          currentStepSize[i] = stepSizes[note_list[i]];
          currentAverage[i] = averages[note_list[i]];
        } else {
          currentStepSize[i] = 0; //TODO: fix this. this is time consuming
          currentAverage[i] = 0;
        }
      }
}

void setNoteName(notes* note_list) {

      // Serial.print("Set note name with" + String(note_1) + " and " + String(note_2));
      String keyString;
      for (int i=0; i<n; i++) {
        if (note_list[i] != None) {
          keyString += noteNames[note_list[i]];
        }
      }

      u8g2.clearBuffer();         // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font

      // Dumb line
      u8g2.drawStr(2,10,"Not a real piano!");  // write something to the internal memory

      // Key array matrix
      u8g2.setCursor(2,20);
      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      u8g2.print(keyArray[0], HEX);
      u8g2.print(keyArray[1], HEX);
      u8g2.print(keyArray[2], HEX);
      u8g2.print(keyArray[3], HEX);
      xSemaphoreGive(keyArrayMutex);

      // Piano note
      u8g2.drawStr(2,30, keyString.c_str());

      // Right hand knob
      u8g2.setCursor(50,20);
      u8g2.print(knob0.get_rotation(), DEC);
      u8g2.setCursor(65,20);
      u8g2.print(knob1.get_rotation(), DEC);
      u8g2.setCursor(80,20);
      u8g2.print(knob2.get_rotation(), DEC);
      u8g2.setCursor(95,20);
      u8g2.print(knob3.get_rotation(), DEC);

      // transfer internal memory to the display
      u8g2.sendBuffer();          
}

void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS; // Q6a: atttempt to improve knob accuracy by increasing sample rate
  TickType_t xLastWakeTime= xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    int32_t localCurrentStepSize[n] = {0,0,0,0}; //TODO: is this critical?

    for (int i = 0; i < 7; i++) { //expanded to read row 3, which is for the right hand knob
        setRow(i);
        delayMicroseconds(2);
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        keyArray[i] = readCols();
        xSemaphoreGive(keyArrayMutex);
    }
    // Call function for setting stepsize
    findKeywithFunc(&setStepSize);

    // Find rotation of knob
    // TODO: protected with a key array mutex?
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    knob3.read(keyArray[3]);
    knob2.read(keyArray[3]);
    knob1.read(keyArray[4]);
    knob0.read(keyArray[4]);
    xSemaphoreGive(keyArrayMutex);
    knob3.update();
    knob2.update();
    knob1.update();
    knob0.update();
  }
}

void displayUpdateTask(void * pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime= xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    // Call function for setting notename
    findKeywithFunc(&setNoteName);

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }

}

void sampleISR(){
  static int32_t phaseAcc[n] = {0,0,0,0};
  static int32_t phaseAcc_DC[n] = {0,0,0,0};

  static int32_t phaseAcc_0 = 0; phaseAcc_0 += currentStepSize[0];
  static int32_t phaseAcc_1 = 0; phaseAcc_1 += currentStepSize[1];
  static int32_t phaseAcc_2 = 0; phaseAcc_2 += currentStepSize[2];
  static int32_t phaseAcc_3 = 0; phaseAcc_3 += currentStepSize[3];
  // static int32_t phaseAcc_4 = 0; phaseAcc_4 += currentStepSize[4];
  // static int32_t phaseAcc_5 = 0; phaseAcc_5 += currentStepSize[5];
  // static int32_t phaseAcc_6 = 0; phaseAcc_6 += currentStepSize[6];
  // static int32_t phaseAcc_7 = 0; phaseAcc_7 += currentStepSize[7];
  // static int32_t phaseAcc_8 = 0; phaseAcc_8 += currentStepSize[8];
  // static int32_t phaseAcc_9 = 0; phaseAcc_9 += currentStepSize[9];

  static int32_t phaseAcc_0_sel = 0;
  static int32_t phaseAcc_1_sel = 0;
  static int32_t phaseAcc_2_sel = 0;
  static int32_t phaseAcc_3_sel = 0;

  if (knob0.get_rotation() == 0) {
    // Sawtooth
    phaseAcc_0_sel = phaseAcc_0;
    phaseAcc_1_sel = phaseAcc_1;
    phaseAcc_2_sel = phaseAcc_2;
    phaseAcc_3_sel = phaseAcc_3;
  } else if (knob0.get_rotation() == 1) {
    // Square
    if (phaseAcc_0 > int32_max/2) {
      phaseAcc_0_sel = 1;
    } else {
      phaseAcc_0_sel = 0;
    }
    if (phaseAcc_1 > int32_max/2) {
      phaseAcc_1_sel = 1;
    } else {
      phaseAcc_1_sel = 0;
    }
    if (phaseAcc_2 > int32_max/2) {
      phaseAcc_2_sel = 1;
    } else {
      phaseAcc_2_sel = 0;
    }
    if (phaseAcc_3 > int32_max/2) {
      phaseAcc_3_sel = 1;
    } else {
      phaseAcc_3_sel = 0;
    }
  } else if (knob0.get_rotation() == 2) {
    // Triangle
    if (phaseAcc_0 > int32_max/2) {
      phaseAcc_0_sel = -1 + (phaseAcc_0/int32_max - 0.5)*4;
    } else {
      phaseAcc_0_sel = 1 - (phaseAcc_0/int32_max)*4;;
    }
    if (phaseAcc_1 > int32_max/2) {
      phaseAcc_1_sel = -1 + (phaseAcc_1/int32_max - 0.5)*4;
    } else {
      phaseAcc_1_sel = 1 - (phaseAcc_1/int32_max)*4;;
    }
    if (phaseAcc_2 > int32_max/2) {
      phaseAcc_2_sel = -1 + (phaseAcc_2/int32_max - 0.5)*4;
    } else {
      phaseAcc_2_sel = 1 - (phaseAcc_2/int32_max)*4;;
    }
    if (phaseAcc_3 > int32_max/2) {
      phaseAcc_3_sel = -1 + (phaseAcc_3/int32_max - 0.5)*4;
    } else {
      phaseAcc_3_sel = 1 - (phaseAcc_3/int32_max)*4;;
    }
  } else if (knob0.get_rotation() == 3) {
    // Sine
  }

  static int32_t phaseAcc_DC_0 = 0; phaseAcc_DC_0 = phaseAcc_0_sel - currentAverage[0];
  static int32_t phaseAcc_DC_1 = 0; phaseAcc_DC_1 = phaseAcc_1_sel - currentAverage[1];
  static int32_t phaseAcc_DC_2 = 0; phaseAcc_DC_2 = phaseAcc_2_sel - currentAverage[2];
  static int32_t phaseAcc_DC_3 = 0; phaseAcc_DC_2 = phaseAcc_3_sel - currentAverage[3];
  static int32_t phaseAcc_DC_4 = 0; phaseAcc_DC_2 = phaseAcc_4 - currentAverage[4];
  static int32_t phaseAcc_DC_5 = 0; phaseAcc_DC_2 = phaseAcc_5 - currentAverage[5];
  static int32_t phaseAcc_DC_6 = 0; phaseAcc_DC_2 = phaseAcc_6 - currentAverage[6];
  static int32_t phaseAcc_DC_7 = 0; phaseAcc_DC_2 = phaseAcc_7 - currentAverage[7];
  static int32_t phaseAcc_DC_8 = 0; phaseAcc_DC_2 = phaseAcc_8 - currentAverage[8];
  static int32_t phaseAcc_DC_9 = 0; phaseAcc_DC_2 = phaseAcc_9 - currentAverage[9];

  static int32_t phaseAcc_final = 0;

  if (phaseAcc_0 > phaseAcc_1 && phaseAcc_0 > phaseAcc_2 && phaseAcc_0 > phaseAcc_3) {
    phaseAcc_final = phaseAcc_0;
  } else if (phaseAcc_1 > phaseAcc_0 && phaseAcc_1 > phaseAcc_2 && phaseAcc_1 > phaseAcc_3) {
    phaseAcc_final = phaseAcc_1;
  } else if (phaseAcc_2 > phaseAcc_1 && phaseAcc_2 > phaseAcc_0 && phaseAcc_2 > phaseAcc_3) {
    phaseAcc_final = phaseAcc_2;
  } else {
    phaseAcc_final = phaseAcc_3;
  } 

  /*
  for (int i=0; i<n; i++) {
    phaseAcc[i] += currentStepSize[i];
    phaseAcc_DC[i] = phaseAcc[i] - currentAverage[i];
  }

  static int32_t phaseAcc_final = 0;
  for (int i=0; i<n; i++) {
    if (phaseAcc_DC[i] > phaseAcc_final) {
      phaseAcc_final = phaseAcc_DC[i];
    }
  }
  */

  int32_t Vout = phaseAcc_final >> 24;
  Vout = Vout >> (8 - knob3.get_rotation()/2); // Volume Control

  analogWrite(OUTR_PIN, Vout+128);
}

void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Profiling Execution Time of Task
  //1. Disable all tasks except the one being profiled by commenting out the xTaskCreate calls
  //2. Disable the sampleTimer ISR by commenting out the attachInterrupt call
  //3. Change the while loop of your Task into a for loop of 32 iterations
  //4. Make sure your loop runs the WORST CASE scenario of the given task
  //5. Run the program and observe the execution time of the task as an average of 32 iterations

  //Replace your outer while loop with the following:
  //uint32_t startTime = micros();
  //for (int iter = 0; iter < 32; iter++) {
  //   givenTask()
  //}
  //Serial.println(micros()-startTime);


  //Initialise timer
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer= new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  //Initialise Semaphore
  keyArrayMutex = xSemaphoreCreateMutex();

  //Initialise Keyscanning Loop
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,/* Function that implements the task */
    "scanKeys",/* Text name for the task */
    256,
    // 64,      /* Stack size in words, not bytes*/
    NULL,/* Parameter passed into the task */
    1,/* Task priority*/
    &scanKeysHandle
  );  /* Pointer to store the task handle*/

  //Initialise Display Loop
  TaskHandle_t displayHandle = NULL;
  xTaskCreate(
    displayUpdateTask,/* Function that implements the task */
    "displayUpdate",/* Text name for the task */
    256,      /* Stack size in words, not bytes*/
    NULL,/* Parameter passed into the task */
    1,/* Task priority*/
    &displayHandle
  );  /* Pointer to store the task handle*/

  vTaskStartScheduler();

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
}

void loop() {
}