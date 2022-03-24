#include <Arduino.h>
#include <U8g2lib.h>
#include<string.h>
#include <STM32FreeRTOS.h>
#include "Knob.h"
#include <ES_CAN.h>

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

//const int32_t stepSizes [] = {51076057,54113197,57330935,60740010, 64351799, 68178356, 72232452, 76527617, 81078186, 85899346,91007187,96418756}; // octave 4
//const int32_t stepSizes [] = {817217093, 865810744, 917293736, 971839820, 1029628605, 1090853364, 1155719084, 1224442465, 1297251922, 1374389535, 1456114953, 1542699542}; // octave 8
const int32_t stepSizes [] = {102151893,108227319,114661961,121479245,128702600,136357403,144465130,153055064,162156490,171798692,182014857,192838175}; // octave 5
const int32_t averages [] = {1072095723,	1055207342,	1060528483,	1062773477,	1061711081,	1056367844,	1047042225,	1071042264,	1053813723,	1030792152,	1046317902,	1060317060};
const double sine_factor[] = {0.000002303951606, 0.000002052587542, 0.000001828647598 ,0.000001629139798 ,0.000001451398562 ,0.000001293049119 ,0.000001151975806 ,0.000001026293767 ,0.000000914323797 ,0.000000814569898 ,0.000000725699277
};
const String noteNames [] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
volatile uint8_t keyArray[7];

SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t stepSizeMutex;

const int32_t int32_max = 2147483647;
const uint8_t n = 4;
volatile int32_t currentStepSize[n];
volatile int32_t currentAverage[n];


// global variable determining mode
bool receiver = true;
bool only_module = false;
bool position_set = false; // indicates whether position has been set
uint8_t position = 6; // undefined
uint8_t octave = 7; // some initial value
uint8_t lowest_octave = 3; // position 0 will have this octave
//size_t id_hash;
volatile bool externalKeyPressed = false;

// CAN communication variables
uint8_t TX_Message[8] = {0};
uint8_t RX_Message[8] = {0};

// queue handler
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;

// global handle for a FreeRTOS mutex
SemaphoreHandle_t RX_Message_Mutex;
SemaphoreHandle_t CAN_TX_Semaphore;

// Knob
Knob knob0(7, 0, 0, 0);
Knob knob1(16, 0, 1, 0);
Knob knob2(16, 0, 2, 0);
Knob knob3(16, 0, 3, 16);

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

void findKeyWithFunc(void (*func)(notes*)) {
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
}

void setStepSize(notes* note_list) {

      int32_t localCurrentStepSize;
      int32_t localCurrentAverage;
      for (int i=0; i<n; i++) {
        if (note_list[i] != None) {
          localCurrentStepSize = stepSizes[note_list[i]] >> (5 - octave);
          localCurrentAverage = averages[note_list[i]];
          xSemaphoreTake(stepSizeMutex, portMAX_DELAY);
          currentStepSize[i] = localCurrentStepSize;
          currentAverage[i] = localCurrentAverage;
          xSemaphoreGive(stepSizeMutex);
        } else {
          xSemaphoreTake(stepSizeMutex, portMAX_DELAY);
          currentStepSize[i] = 0;
          currentAverage[i] = 0;
          xSemaphoreGive(stepSizeMutex);
        }
      }
}

void setNoteName(notes* note_list) {

      String keyString;
      for (int i=0; i<n; i++) {
        if (note_list[i] != None) {
          keyString += noteNames[note_list[i]];
        }
      }

      u8g2.clearBuffer();         // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font

      // Key array matrix
      u8g2.setCursor(2,10);
      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      u8g2.print(keyArray[0], HEX);
      u8g2.print(keyArray[1], HEX);
      u8g2.print(keyArray[2], HEX);
      u8g2.print(keyArray[3], HEX);
      xSemaphoreGive(keyArrayMutex);

      // Piano note
      u8g2.setCursor(37,10);
      u8g2.print(position, HEX);
      u8g2.print(octave, HEX);
      u8g2.drawStr(47,10, keyString.c_str());

      //receiver/Sender
      receiver ? u8g2.drawStr(77,10,"R") : u8g2.drawStr(77,10,"S");

      // TX/RX Messages
      u8g2.setCursor(87,10);
      u8g2.print((char) TX_Message[0]);
      xSemaphoreTake(RX_Message_Mutex, portMAX_DELAY);
      u8g2.print(RX_Message[1]);
      u8g2.print(RX_Message[2]);
      xSemaphoreGive(RX_Message_Mutex);

      // Right hand knob
      u8g2.setCursor(2,20);
      u8g2.print(knob0.get_rotation(), DEC);
      u8g2.setCursor(37,20);
      u8g2.print(knob1.get_rotation(), DEC);
      u8g2.setCursor(72,20);
      u8g2.print(knob2.get_rotation(), DEC);
      u8g2.setCursor(107,20);
      u8g2.print(knob3.get_rotation(), DEC);

      if (knob0.get_rotation() == 0 || knob0.get_rotation() == 1) {
        u8g2.drawStr(2,30,"Saw");
      } else if (knob0.get_rotation() == 2 || knob0.get_rotation() == 3) {
        u8g2.drawStr(2,30,"Sqa");
      } else if (knob0.get_rotation() == 4 || knob0.get_rotation() == 5) {
        u8g2.drawStr(2,30,"Tri");
      } else if (knob0.get_rotation() == 6 || knob0.get_rotation() == 7) {
        u8g2.drawStr(2,30,"Sine");
      }

      u8g2.drawStr(37, 30,"-");
      u8g2.drawStr(72, 30,"-");
      u8g2.drawStr(107, 30,"Vol");
      // transfer internal memory to the display
      u8g2.sendBuffer();
}

void setCommMessage(notes* note_list){

  TX_Message[0] = keyArray[0];
  TX_Message[1] = keyArray[1];
  TX_Message[2] = keyArray[2];
  TX_Message[3] = octave;
  TX_Message[4] = position;
  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);


  // if(note_list[0] == None){
  //     TX_Message[0] = 'R';
  //     xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  // } else {
  //     TX_Message[0] = 'P';
  //     TX_Message[1] = octave;
  //     TX_Message[2] = note_list[0];
  //     xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  // }
}


void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS; // Q6a: atttempt to improve knob accuracy by increasing sample rate
  TickType_t xLastWakeTime= xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    // int32_t localCurrentStepSize[n] = {0,0,0,0}; //TODO: is this critical?

    for (int i = 0; i < 7; i++) { //expanded to read row 3, which is for the right hand knob
        setRow(i);
        delayMicroseconds(2);
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        keyArray[i] = readCols();
        xSemaphoreGive(keyArrayMutex);
    }

    if(!receiver) {
      findKeyWithFunc(&setCommMessage); // send keyArray to receiver
    }
    else if(receiver && only_module) { // receiver and it's the only module
      findKeyWithFunc(&setStepSize);
    }

    // Find rotation of knob, protected with a key array mutex?
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
    findKeyWithFunc(&setNoteName);

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }

}

void turnoffEast() {
  digitalWrite(OUT_PIN, LOW);
  setRow(6);
  digitalWrite(REN_PIN,1);
  delayMicroseconds(2);
  readCols();
  digitalWrite(REN_PIN,0);
}

void broadcastPosition() {
  TX_Message[0] = 'H';
  TX_Message[1] = 0;
  TX_Message[2] = position;
  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
}

void broadcastEndOfHandshake() {
  if(position != 0) {
    receiver = false;
  }
  octave = position + lowest_octave;
  TX_Message[0] = 'E';
  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
}

void handshakeDecodeTask(void * pvParameters) {
  uint8_t local_RX_Message[8] = {0};
  xQueueReceive(msgInQ, local_RX_Message, portMAX_DELAY);
  //xSemaphoreTake(RX_Message_Mutex, portMAX_DELAY);
  std::copy(std::begin(local_RX_Message), std::end(local_RX_Message), std::begin(RX_Message));
  //xSemaphoreGive(RX_Message_Mutex);

  switch(RX_Message[0]) {
    case 'H':
      {
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        setRow(5);
        delayMicroseconds(2);
        keyArray[5] = readCols();
        if(keyArray[5] >> 3 & B1) { // west turned off
          if(!position_set) {
            position = RX_Message[2] + 1;
            position_set = true;
          }
          if(keyArray[6] >> 3 & B1) { // check if east turned off
            broadcastEndOfHandshake();
          }
          else {
            turnoffEast();
            broadcastPosition();
          }
        }
        xSemaphoreGive(keyArrayMutex);
        break;
      }
    case 'E':
      {
        if(position == 0) {
          receiver = true;
        }
        else {
          receiver = false;
        }
        octave =  position + lowest_octave;
        break;
      }
  }
}

std::array<bool, 12> getKeys(uint8_t* keys){
  std::array<bool, 12> rtn;
  
  uint8_t CDs = keys[0];
  uint8_t EG = keys[1];
  uint8_t GsB =keys[2];
  rtn[0] = (~CDs >> 0) & B1;
  rtn[1] = (~CDs >> 1) & B1;
  rtn[2] = (~CDs >> 2) & B1;
  rtn[3] = (~CDs >> 3) & B1;
  rtn[4] = (~EG  >> 0) & B1;
  rtn[5] = (~EG  >> 1) & B1;
  rtn[6] = (~EG  >> 2) & B1;
  rtn[7] = (~EG  >> 3) & B1;
  rtn[8] = (~GsB >> 0) & B1;
  rtn[9] = (~GsB >> 1) & B1;
  rtn[10] = (~GsB >> 2) & B1;
  rtn[11] = (~GsB >> 3) & B1;

  return rtn;
}

void decodeTask(void * pvParameters) {
  uint8_t local_RX_Message[8] = {0};
  uint8_t keyArray_1[8] = {15,15,15,15,15,15,15,15}; // keyArray from sender 1 will be stored here
  uint8_t keyArray_2[8] = {15,15,15,15,15,15,15,15}; // keyArray from sender 2 will be stored here
  xQueueReceive(msgInQ, local_RX_Message, portMAX_DELAY); 
  xSemaphoreTake(RX_Message_Mutex, portMAX_DELAY);
  std::copy(std::begin(local_RX_Message), std::end(local_RX_Message), std::begin(RX_Message));
  xSemaphoreGive(RX_Message_Mutex);

  switch(RX_Message[0]) {
    case 'H':
      {
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        setRow(5);
        delayMicroseconds(2);
        keyArray[5] = readCols();
        if(keyArray[5] >> 3 & B1) { // west turned off
          if(!position_set) {
            position = RX_Message[2] + 1;
            position_set = true;
          }
          if(keyArray[6] >> 3 & B1) { // check if east turned off
            broadcastEndOfHandshake();
          }
          else {
            turnoffEast();
            broadcastPosition();
          }
        }
        xSemaphoreGive(keyArrayMutex);
        break;
      }
    case 'E':
      {
        if(position == 0) {
          receiver = true;
        }
        else {
          receiver = false;
        }
        octave =  position + lowest_octave;
        break;
      }
  }
  
  // storing received message in correct position
  switch(RX_Message[4]) {
    case 1:
      {
        xSemaphoreTake(RX_Message_Mutex, portMAX_DELAY);
        std::copy(std::begin(RX_Message), std::end(RX_Message), std::begin(keyArray_1));
        xSemaphoreGive(RX_Message_Mutex);
        break;
      }
    case 2:
      {
        xSemaphoreTake(RX_Message_Mutex, portMAX_DELAY);
        std::copy(std::begin(RX_Message), std::end(RX_Message), std::begin(keyArray_2));
        xSemaphoreGive(RX_Message_Mutex);
        break;
      }
  }

  // now we have keyArray, keyArray_1, keyArray_2
  // set currentStepSize, currentAverage
  uint8_t keyArray_0[8] = {0};
  xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
  std::copy(std::begin(keyArray), std::end(keyArray), std::begin(keyArray_0));
  xSemaphoreGive(keyArrayMutex);
  std::array<bool,36> keysPressed;
  std::array<bool,12> keysPressed_0 = getKeys(keyArray_0);
  std::array<bool,12> keysPressed_1 = getKeys(keyArray_1);
  std::array<bool,12> keysPressed_2 = getKeys(keyArray_2);
  for(int i = 0; i < 12; i++) {
    keysPressed[i] = keysPressed_0[i];
    keysPressed[i+12] = keysPressed_1[i];
    keysPressed[i+24] = keysPressed_2[i];
  }
  uint8_t octaves[3] = {octave, keyArray_1[3], keyArray_2[3]};
  int32_t localCurrentStepSize[n] = {0};
  int32_t localCurrentAverage[n] = {0};
  int j = 0;
  for(int i = 0; i < n; i++) {
    while(j < 36) {
      if(keysPressed[j]) {
        localCurrentStepSize[i] = stepSizes[j % 12] << (5 - octaves[j / 12]);
        //localCurrentAverage[i] = averages[j % 12]; // TODO: what is average
        j++;
        break;
      }
      j++;
    }
  }
  xSemaphoreTake(stepSizeMutex, portMAX_DELAY);
  std::copy(std::begin(localCurrentStepSize), std::end(localCurrentStepSize), std::begin(currentStepSize));
  std::copy(std::begin(localCurrentAverage), std::end(localCurrentAverage), std::begin(currentAverage));
  xSemaphoreGive(stepSizeMutex);
}

// void decodeTask(void * pvParameters) {
//   uint8_t local_RX_Message[8] = {0};
//   bool local_externalKeyPressed;
//   while (true) {
//     for (int i = 0; i < 3; i++) {
//       setRow(i);
//       delayMicroseconds(2);
//       keyArray[i] = readCols();
//     }
//     // Call function for setting stepsize
//     if(receiver && !externalKeyPressed)
//       findKeyWithFunc(&setStepSize);
//     if(!receiver)
//       findKeyWithFunc(&setCommMessage);

//     // wait until a message is available in the queue
//     xQueueReceive(msgInQ, local_RX_Message, portMAX_DELAY);

//     xSemaphoreTake(RX_Message_Mutex, portMAX_DELAY);
//     std::copy(std::begin(local_RX_Message), std::end(local_RX_Message), std::begin(RX_Message));
//     xSemaphoreGive(RX_Message_Mutex);

//     switch(RX_Message[0]) {
//       case 'R':
//         {
//           local_externalKeyPressed = false;
//           __atomic_store_n(&externalKeyPressed,local_externalKeyPressed,__ATOMIC_RELAXED);
//           notes empty [4]= {None, None, None, None};
//           setStepSize(empty);
//           break;
//         }
//       case 'P':
//         {
//           local_externalKeyPressed = true;
//           __atomic_store_n(&externalKeyPressed,local_externalKeyPressed,__ATOMIC_RELAXED);
//           // TODO: Fixing this
//           int32_t localCurrentStepSize = stepSizes[RX_Message[2]] >> (5 - RX_Message[1]);
//           xSemaphoreTake(stepSizeMutex, portMAX_DELAY);
//           for (int i=0; i<n; i++) {
//             if(currentStepSize[i] == 0) {
//               currentStepSize[i] = localCurrentStepSize;
//               break;
//             };
//           }
//           xSemaphoreGive(stepSizeMutex);
//           break;
//         }
//       case 'H':
//         {
//           setRow(5);
//           delayMicroseconds(2);
//           keyArray[5] = readCols();
//           if(keyArray[5] >> 3 & B1) { // west turned off
//             if(!position_set) {
//               position = RX_Message[2] + 1;
//               position_set = true;
//             }
//             if(keyArray[6] >> 3 & B1) { // check if east turned off
//               broadcastEndOfHandshake();
//             }
//             else {
//               turnoffEast();
//               broadcastPosition();
//             }
//           }
//           break;
//         }
//       case 'E':
//         {
//           if(position == 0) {
//             receiver = true;
//           }
//           else {
//             receiver = false;
//           }
//           octave =  position + lowest_octave;
//           break;
//         }
//     }
//   }
// }

void CAN_TX_Task(void * pvParameters) {
  uint8_t msgOut[8];
  while (true) {
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    CAN_TX(0x123, msgOut);
  }
}

//TODO: Update
void sampleISR(){
  //static int32_t phaseAcc[n] = {0,0,0,0};
  //static int32_t phaseAcc_DC[n] = {0,0,0,0};

  int32_t localCurrentStepSize[n];
  int32_t localCurrentAverage[n];

  for (int i=0; i<n; i++) {
    localCurrentStepSize[i] = currentStepSize[i];
    localCurrentAverage[i] = currentAverage[i];
  }

  static int32_t phaseAcc_0 = 0; phaseAcc_0 += localCurrentStepSize[0];
  static int32_t phaseAcc_1 = 0; phaseAcc_1 += localCurrentStepSize[1];
  static int32_t phaseAcc_2 = 0; phaseAcc_2 += localCurrentStepSize[2];
  static int32_t phaseAcc_3 = 0; phaseAcc_3 += localCurrentStepSize[3];

  static int32_t phaseAcc_0_sel = 0;
  static int32_t phaseAcc_1_sel = 0;
  static int32_t phaseAcc_2_sel = 0;
  static int32_t phaseAcc_3_sel = 0;

  static double phaseAcc_0_sine = 0;
  static double phaseAcc_1_sine = 0;
  static double phaseAcc_2_sine = 0;
  static double phaseAcc_3_sine = 0;

  if (receiver){
    if (knob0.get_rotation() == 0 || knob0.get_rotation() == 1) {
      // Sawtooth: Deduct by DC for polyphony
      phaseAcc_0_sel = phaseAcc_0 - currentAverage[0];
      phaseAcc_1_sel = phaseAcc_1 - currentAverage[1];
      phaseAcc_2_sel = phaseAcc_2 - currentAverage[2];
      phaseAcc_3_sel = phaseAcc_3 - currentAverage[3];
    } else if (knob0.get_rotation() == 2 || knob0.get_rotation() == 3) {
      // Square
      if (phaseAcc_0 > int32_max/2) {
        phaseAcc_0_sel = int32_max;
      } else {
        phaseAcc_0_sel = 0;
      }
      if (phaseAcc_1 > int32_max/2) {
        phaseAcc_1_sel = int32_max;
      } else {
        phaseAcc_1_sel = 0;
      }
      if (phaseAcc_2 > int32_max/2) {
        phaseAcc_2_sel = int32_max;
      } else {
        phaseAcc_2_sel = 0;
      }
      if (phaseAcc_3 > int32_max/2) {
        phaseAcc_3_sel = int32_max;
      } else {
        phaseAcc_3_sel = 0;
      }
    } else if (knob0.get_rotation() == 4 || knob0.get_rotation() == 5) {
      // Triangle
      if (phaseAcc_0 > int32_max/2) {
        phaseAcc_0_sel = -3*int32_max + phaseAcc_0*4;
      } else {
        phaseAcc_0_sel = int32_max - phaseAcc_0*4;
      }
      if (phaseAcc_1 > int32_max/2) {
        phaseAcc_1_sel = -3*int32_max + phaseAcc_1*4;
      } else {
        phaseAcc_1_sel = int32_max - phaseAcc_1*4;
      }
      if (phaseAcc_2 > int32_max/2) {
        phaseAcc_2_sel = -3*int32_max + phaseAcc_2*4;
      } else {
        phaseAcc_2_sel = int32_max - phaseAcc_2*4;
      }
      if (phaseAcc_3 > int32_max/2) {
        phaseAcc_3_sel = -3*int32_max + phaseAcc_3*4;
      } else {
        phaseAcc_3_sel = int32_max - phaseAcc_3*4;
      }
    } else if (knob0.get_rotation() == 6 || knob0.get_rotation() == 7) {
      /* Sine
      phaseAcc_0_sine = (double)int32_max *sin(phaseAcc_0*sine_factor[0]);
      phaseAcc_1_sine = (double)int32_max *sin(phaseAcc_1*sine_factor[1]);
      phaseAcc_2_sine = (double)int32_max *sin(phaseAcc_2*sine_factor[2]);
      phaseAcc_3_sine = (double)int32_max *sin(phaseAcc_3*sine_factor[3]);

      static double phaseAcc_final = 0;
      if (phaseAcc_0_sine > phaseAcc_1_sine && phaseAcc_0_sine > phaseAcc_2_sine && phaseAcc_0_sine > phaseAcc_3_sine) {
        phaseAcc_final = phaseAcc_0_sine;
      } else if (phaseAcc_1_sine > phaseAcc_0_sine && phaseAcc_1_sine > phaseAcc_2_sine && phaseAcc_1_sine > phaseAcc_3_sine) {
        phaseAcc_final = phaseAcc_1_sine;
      } else if (phaseAcc_2_sine > phaseAcc_1_sine && phaseAcc_2_sine > phaseAcc_0_sine && phaseAcc_2_sine > phaseAcc_3_sine) {
        phaseAcc_final = phaseAcc_2_sine;
      } else {
        phaseAcc_final = phaseAcc_3_sine;
      }

      static int32_t phaseAcc_casted = (int32_t) phaseAcc_final;
      int32_t Vout = phaseAcc_casted >> 24;
      Vout = Vout >> (8 - knob3.get_rotation()/2); // Volume Control
      analogWrite(OUTR_PIN, Vout+128);
      */
    }

    if (knob0.get_rotation() == 6 || knob0.get_rotation() == 7) {
      //pass
    } else {
      static int32_t phaseAcc_final = 0;

      if (phaseAcc_0_sel > phaseAcc_1_sel && phaseAcc_0_sel > phaseAcc_2_sel && phaseAcc_0_sel > phaseAcc_3_sel) {
        phaseAcc_final = phaseAcc_0_sel;
      } else if (phaseAcc_1_sel > phaseAcc_0_sel && phaseAcc_1_sel > phaseAcc_2_sel && phaseAcc_1_sel > phaseAcc_3_sel) {
        phaseAcc_final = phaseAcc_1_sel;
      } else if (phaseAcc_2_sel > phaseAcc_1_sel && phaseAcc_2_sel > phaseAcc_0_sel && phaseAcc_2_sel > phaseAcc_3_sel) {
        phaseAcc_final = phaseAcc_2_sel;
      } else {
        phaseAcc_final = phaseAcc_3_sel;
      }

      int32_t Vout = phaseAcc_final >> 24;
      Vout = Vout >> (8 - knob3.get_rotation()/2); // Volume Control
      analogWrite(OUTR_PIN, Vout+128);
    }
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
}

void CAN_RX_ISR(void) {
  uint8_t RX_Message_ISR[8];
  uint32_t ID;
  // receive message data
  CAN_RX(ID, RX_Message_ISR);
  // place data in the queue
  xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void CAN_TX_ISR(void) {
  xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void initialCheck() {
  digitalWrite(OUT_PIN,1);
  for(int i = 5; i <= 6; i++) {
    setRow(i);
    digitalWrite(REN_PIN,1);
    delayMicroseconds(2);
    keyArray[i] = readCols();
    digitalWrite(REN_PIN,0);
  }
  if((keyArray[5] >> 3) & B1) { // west not connected
    position = 0;
    position_set = true;
  }
}

void initialHandshake() {
  // uint32_t id = HAL_GetUIDw0(); // unique id
  // std::hash<uint32_t> myHash;
  // id_hash = myHash(id);
  if(position_set) { // left most
    if((keyArray[6] >> 3) & B1) { // the only module
      octave = position + lowest_octave;
      only_module = true;
      // end of handshake
    }
    else {
      turnoffEast();
      broadcastPosition();
    }
  }
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

  //Initialise CAN
  //CAN_Init(true);
  CAN_Init(false);
  setCANFilter(0x123,0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  //CAN_Start();

  //Initialise queue handler
  msgInQ = xQueueCreate(36, 8);
  msgOutQ = xQueueCreate(36, 8);

  //Initialise Semaphore
  RX_Message_Mutex = xSemaphoreCreateMutex();
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);
  keyArrayMutex = xSemaphoreCreateMutex();
  stepSizeMutex = xSemaphoreCreateMutex();

  //Initialise Keyscanning Loop
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,/* Function that implements the task */
    "scanKeys",/* Text name for the task */
    512,
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
    512,      /* Stack size in words, not bytes*/
    NULL,/* Parameter passed into the task */
    1,/* Task priority*/
    &displayHandle
  );  /* Pointer to store the task handle*/
  TaskHandle_t decodeHandle = NULL;
  //Initialise Decode loop
  xTaskCreate(
    decodeTask,   /* Function that implements the task */
    "decode",     /* Text name for the task */
    256,          /* Stack size in words, not bytes */
    NULL,         /* Parameter passed into the task */
    1,            /* Task priority */
    &decodeHandle /* Pointer to store the task handle */
  );

  // //Initialise handshakeDecode loop
  // TaskHandle_t handshakeDecodeHandle = NULL;
  // xTaskCreate(
  //   handshakeDecodeTask,   /* Function that implements the task */
  //   "handshakeDecode",     /* Text name for the task */
  //   256,          /* Stack size in words, not bytes */
  //   NULL,         /* Parameter passed into the task */
  //   1,            /* Task priority */
  //   &handshakeDecodeHandle /* Pointer to store the task handle */
  // );

  //Initialise transmit thread
  TaskHandle_t transmitHandle = NULL;
  xTaskCreate(
    CAN_TX_Task,   /* Function that implements the task */
    "transmit",     /* Text name for the task */
    256,          /* Stack size in words, not bytes */
    NULL,         /* Parameter passed into the task */
    1,            /* Task priority */
    &transmitHandle /* Pointer to store the task handle */
  );

  //Initialise handshake: setting west and east out high
  digitalWrite(OUT_PIN, HIGH);
  for(int i = 5; i <= 6; i++) {
    setRow(i);
    digitalWrite(REN_PIN,1);
    delayMicroseconds(2);
    readCols();
    digitalWrite(REN_PIN,0);
  }

  delayMicroseconds(1000000); // wait 1 sec
  initialCheck();
  CAN_Start(); // start here as CAN_Init value can change inside initialCheck()
  delayMicroseconds(1000000); // wait 1 sec
  initialHandshake();

  //vTaskSuspend(handshakeDecodeHandle);

  // TaskHandle_t decodeHandle = NULL;
  // if(true) { // TODO: change this to receiver only
  //   //Initialise Decode loop
  //   xTaskCreate(
  //     decodeTask,   /* Function that implements the task */
  //     "decode",     /* Text name for the task */
  //     256,          /* Stack size in words, not bytes */
  //     NULL,         /* Parameter passed into the task */
  //     1,            /* Task priority */
  //     &decodeHandle /* Pointer to store the task handle */
  //   );
  // }

  vTaskStartScheduler();

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
}

void loop() {
}