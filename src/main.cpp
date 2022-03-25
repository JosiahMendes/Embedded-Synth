#include <Arduino.h>
#include <U8g2lib.h>
#include <string.h>
#include <ES_CAN.h>
#include <STM32FreeRTOS.h>
#include "Knob.h"
#include "global_vars.h"
#include <vector>
#include <algorithm>



// This is for 44kHz!!
const int32_t stepSizes [] = {51076057, 54113197, 57330935, 60740010, 64351799, 68178356, 72232452, 76527617, 81078186, 85899346, 91007187, 96418756, };
const int32_t sine_acc[] = {49, 52, 55, 58, 61, 65, 69, 73, 77, 82, 87, 92};

const int16_t lookup_size = 2048;

const String noteNames [] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
volatile uint8_t keyArray[7];

SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t stepSizeMutex;

const int32_t int32_max = 2147483647;
const int32_t half_max = 1073741824;
const uint8_t n = 4;

volatile int32_t currentStepSize[n];
volatile int32_t currentSineAcc[n];

// global variable that keeps track of which keys are currently pressed
std::vector<uint16_t> keysPressed;

// global variable determining mode for multi-module CAN communication
bool receiver = true;
bool only_module = false;
bool position_set = false; // indicates whether position has been set
uint8_t position = 6; // undefined
uint8_t octave = 7; // some initial value
uint8_t lowest_octave = 3; // position 0 will have this octave

// CAN communication variables
uint8_t TX_Message[8] = {0};
uint8_t RX_Message[8] = {0};

// global handle for a FreeRTOS mutex
SemaphoreHandle_t RX_Message_Mutex;
SemaphoreHandle_t CAN_TX_Semaphore;

// queue handler
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;

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

      int j = 0;
      for (int i=0; i<12; i++) {
        current_key = bool_array[i];
        current_note = ((notes)i);
        if (current_key && localNotesPressed[j] == None && j < n) {
          localNotesPressed[j] = current_note;
          j++;
        }
      }

      func(localNotesPressed);
}

void setStepSize(notes* note_list) {

      int32_t localCurrentStepSize;
      int32_t localCurrentSineAcc;
      for (int i = 0; i < n; i++) {
        if (note_list[i] != None) {
          localCurrentStepSize = stepSizes[note_list[i]];
          localCurrentSineAcc = sine_acc[note_list[i]];
          xSemaphoreTake(stepSizeMutex, portMAX_DELAY);
          currentStepSize[i] = localCurrentStepSize;
          currentSineAcc[i] = localCurrentSineAcc;
          xSemaphoreGive(stepSizeMutex);
        } else {
          xSemaphoreTake(stepSizeMutex, portMAX_DELAY);
          currentStepSize[i] = 0;
          currentSineAcc[i] = 0;
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
      u8g2.drawStr(72,10, keyString.c_str());
      u8g2.setCursor(82,10);
      xSemaphoreTake(RX_Message_Mutex, portMAX_DELAY);
      u8g2.print((char) RX_Message[0]);
      u8g2.print((char) RX_Message[2], HEX);
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

void compareKeyArray(uint8_t oldKeyArray[3], uint8_t newKeyArray[3]) {
  uint8_t oldCDs = oldKeyArray[0];
  uint8_t oldEG = oldKeyArray[1];
  uint8_t oldGsB = oldKeyArray[2];
  uint8_t newCDs = newKeyArray[0];
  uint8_t newEG = newKeyArray[1];
  uint8_t newGsB = newKeyArray[2];
  TX_Message[1] = octave;
  if ( (~oldCDs >> 0) & B1 ^ (~newCDs >> 0) & B1 ) { // C changed
    TX_Message[0] = ((~oldCDs >> 0) & B1) ? 'R' : 'P';
    TX_Message[2] = 0;
    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  }
  if((~oldCDs >> 1) & B1 ^ (~newCDs >> 1) & B1) { // Cs changed
    TX_Message[0] = ((~oldCDs >> 1) & B1) ? 'R' : 'P';
    TX_Message[2] = 1;
    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  }
  if((~oldCDs >> 2) & B1 ^ (~newCDs >> 2) & B1) { // D changed
    TX_Message[0] = ((~oldCDs >> 2) & B1) ? 'R' : 'P';
    TX_Message[2] = 2;
    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  }
  if((~oldCDs >> 3) & B1 ^ (~newCDs >> 3) & B1) { // Ds changed
    TX_Message[0] = ((~oldCDs >> 3) & B1) ? 'R' : 'P';
    TX_Message[2] = 3;
    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  }
  if((~oldEG  >> 0) & B1 ^ (~newEG >> 0) & B1) { // E changed
    TX_Message[0] = ((~oldEG >> 0) & B1) ? 'R' : 'P';
    TX_Message[2] = 4;
    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  }
  if((~oldEG  >> 1) & B1 ^ (~newEG >> 1) & B1) { // F changed
    TX_Message[0] = ((~oldEG >> 1) & B1) ? 'R' : 'P';
    TX_Message[2] = 5;
    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  }
  if((~oldEG  >> 2) & B1 ^ (~newEG >> 2) & B1) { // Fs changed
    TX_Message[0] = ((~oldEG >> 2) & B1) ? 'R' : 'P';
    TX_Message[2] = 6;
    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  }
  if((~oldEG  >> 3) & B1 ^ (~newEG >> 3) & B1) { // G changed
    TX_Message[0] = ((~oldEG >> 3) & B1) ? 'R' : 'P';
    TX_Message[2] = 7;
    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  }
  if((~oldGsB >> 0) & B1 ^ (~newGsB >> 0) & B1) { // Gs changed
    TX_Message[0] = ((~oldGsB >> 0) & B1) ? 'R' : 'P';
    TX_Message[2] = 8;
    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  }
  if((~oldGsB >> 1) & B1 ^ (~newGsB >> 1) & B1) { // A changed
    TX_Message[0] = (~oldGsB >> 1) & B1 ? 'R' : 'P';
    TX_Message[2] = 9;
    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  }
  if((~oldGsB >> 2) & B1 ^ (~newGsB >> 2) & B1) { // As changed
    TX_Message[0] = ((~oldGsB >> 2) & B1) ? 'R' : 'P';
    TX_Message[2] = 10;
    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  }
  if((~oldGsB >> 3) & B1 ^ (~newGsB >> 3) & B1) { // B changed
    TX_Message[0] = (~oldGsB >> 3) & B1 ? 'R' : 'P';
    TX_Message[2] = 11;
    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
  }
}

void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS; // Q6a: atttempt to improve knob accuracy by increasing sample rate
  TickType_t xLastWakeTime= xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    uint8_t local_keyArray[3];
    uint8_t old_keyArray [3] = {keyArray[0], keyArray[1], keyArray[2]};
    for (int i = 0; i < 3; i++) {
      setRow(i);
      delayMicroseconds(2);
      local_keyArray[i] = readCols();
      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      keyArray[i] = readCols();
      xSemaphoreGive(keyArrayMutex);
    }

    for (int i = 3; i < 7; i++) { //expanded to read row 3, which is for the right hand knob
        setRow(i);
        delayMicroseconds(2);
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        keyArray[i] = readCols();
        xSemaphoreGive(keyArrayMutex);
    }

    compareKeyArray(old_keyArray, local_keyArray);
    // // Call function for setting stepsize
    // findKeywithFunc(&setStepSize);

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

void decodeTask(void * pvParameters) {
  uint8_t local_RX_Message[8] = {0};

  while(true){
    xQueueReceive(msgInQ, local_RX_Message, portMAX_DELAY);
    xSemaphoreTake(RX_Message_Mutex, portMAX_DELAY);
    std::copy(std::begin(local_RX_Message), std::end(local_RX_Message), std::begin(RX_Message));
    xSemaphoreGive(RX_Message_Mutex);

    switch(local_RX_Message[0]) {
      case 'P': {
        uint16_t value  = local_RX_Message[1] << 8 | local_RX_Message[2];
        keysPressed.push_back(value);
        break;
      }
      case 'R': {
        uint16_t value  = local_RX_Message[1] << 8 | local_RX_Message[2];
        keysPressed.erase(std::remove(keysPressed.begin(), keysPressed.end(), value), keysPressed.end());
        break;
      }
    }
    int32_t localCurrentStepSize;
    int32_t localCurrentSineAcc;
    for (int j = 0; j < n; j++) {
        if(keysPressed.size() > j) {
          uint16_t current = keysPressed[j];
          uint8_t note = current & 0xff;
          uint8_t octave = current >> 8;
          localCurrentStepSize = stepSizes[note];
          localCurrentSineAcc = sine_acc[note];
          xSemaphoreTake(stepSizeMutex, portMAX_DELAY);
          currentStepSize[j] = localCurrentStepSize;
          currentSineAcc[j] = localCurrentSineAcc;
          xSemaphoreGive(stepSizeMutex);
        } else{
          xSemaphoreTake(stepSizeMutex, portMAX_DELAY);
            currentStepSize[j] = 0;
            currentSineAcc[j] = 0;
          xSemaphoreGive(stepSizeMutex);
        }
    }
  //   for (int i = 0; i < keysPressed.size(); i++) {
  //     std::string current = keysPressed[i];
  //     int octave = current[1] - '0';
  //     int note = current[0];
  //   }
  }
}

void displayUpdateTask(void * pvParameters) {
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
  static int32_t phaseAcc_sel[n] = {0,0,0,0};
  static int32_t acc[n] = {0,0,0,0};

  if (knob0.get_rotation() == 6 || knob0.get_rotation() == 7) {
    int32_t localCurrentSineAcc[n];
    for (int i=0; i<n; i++) {
      localCurrentSineAcc[i] = currentSineAcc[i];
    }
    for (int i=0; i<n; i++) {
      acc[i] += localCurrentSineAcc[i];
      if (acc[i] > lookup_size) {
        acc[i] = 0;
      }
      phaseAcc_sel[i]  = sine_lookup[acc[i]];
    }
  } else {
    int32_t localCurrentStepSize[n];
    for (int i=0; i<n; i++) {
      localCurrentStepSize[i] = currentStepSize[i];
      phaseAcc[i] += localCurrentStepSize[i];
    }
    if (knob0.get_rotation() == 0 || knob0.get_rotation() == 1) { // Sawtooth
      for (int i=0; i<n; i++) {
        phaseAcc_sel[i] =  phaseAcc[i];
      }
    } else if (knob0.get_rotation() == 2 || knob0.get_rotation() == 3) { // Square: particularly problematic
      for (int i=0; i<n; i++) {
        if (phaseAcc[i] > half_max) {
          phaseAcc_sel[i] = int32_max;
        } else {
          phaseAcc_sel[i] = 0;
        }
      }
    } else if (knob0.get_rotation() == 4 || knob0.get_rotation() == 5) { // Triangular
      for (int i=0; i<n; i++) {
        if (phaseAcc[i]  > half_max) {
          phaseAcc_sel[i]  = -3*int32_max + phaseAcc[i]*4;
        } else {
          phaseAcc_sel[i]  = int32_max - phaseAcc[i]*4;
        }
      }
    }
  }

  int32_t phaseAcc_final = 0;

  if (phaseAcc_sel[0] > phaseAcc_final) {
    phaseAcc_final = phaseAcc_sel[0];
  } else if (phaseAcc_sel[1] > phaseAcc_final) {
    phaseAcc_final = phaseAcc_sel[1];
  } else if (phaseAcc_sel[2] > phaseAcc_final) {
    phaseAcc_final = phaseAcc_sel[2];
  } else if (phaseAcc_sel[3] > phaseAcc_final) {
    phaseAcc_final = phaseAcc_sel[3];
  }

  int32_t Vout = phaseAcc_final >> 24;
  Vout = Vout >> (8 - knob3.get_rotation()/2); // Volume Control
  analogWrite(OUTR_PIN, Vout+128);
}

void CAN_TX_Task(void * pvParameters) {
  uint8_t msgOut[8];
  while (true) {
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    CAN_TX(0x123, msgOut);
  }
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
  sampleTimer->setOverflow(44000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  //Initialise Semaphore
  keyArrayMutex = xSemaphoreCreateMutex();
  stepSizeMutex = xSemaphoreCreateMutex();

  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_Start();
  //Initialise queue handler
  msgInQ = xQueueCreate(36, 8);
  msgOutQ = xQueueCreate(36, 8);
  RX_Message_Mutex = xSemaphoreCreateMutex();
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

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

  //Initialise Decode loop
  TaskHandle_t decodeHandle = NULL;
  xTaskCreate(
    decodeTask,   /* Function that implements the task */
    "decode",     /* Text name for the task */
    512,          /* Stack size in words, not bytes */
    NULL,         /* Parameter passed into the task */
    1,            /* Task priority */
    &decodeHandle /* Pointer to store the task handle */
  );

  //Initialise transmit thread
  TaskHandle_t transmitHandle = NULL;
  xTaskCreate(
    CAN_TX_Task,   /* Function that implements the task */
    "transmit",     /* Text name for the task */
    64,          /* Stack size in words, not bytes */
    NULL,         /* Parameter passed into the task */
    1,            /* Task priority */
    &transmitHandle /* Pointer to store the task handle */
  );

  vTaskStartScheduler();

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
}

void loop() {
}