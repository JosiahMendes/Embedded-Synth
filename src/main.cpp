#include <Arduino.h>
#include <U8g2lib.h>
#include<string.h>
#include <STM32FreeRTOS.h>

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

//TODO: Define step sizes for 12 notes
const int32_t stepSizes [] = {51076057,54113197,57330935,60740010, 64351799, 68178356, 72232452, 76527617, 81078186, 85899346,91007187,96418756};
const String noteNames [] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
volatile uint8_t keyArray[7];
volatile int32_t currentStepSize;

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

void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime= xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    int32_t localCurrentStepSize = 0;

    for (int i = 0; i < 3; i++) {
        setRow(i);
        delayMicroseconds(2);
        keyArray[i] = readCols();
      }
      if (keyArray[0] == 0x0E && keyArray[1] == 0x0F && keyArray[2] == 0x0F){
        localCurrentStepSize = stepSizes[0];
      } else if(keyArray[0] == 0x0D && keyArray[1] == 0x0F && keyArray[2] == 0x0F){
        localCurrentStepSize = stepSizes[1];
      } else if (keyArray[0] == 0x0B && keyArray[1] == 0x0F && keyArray[2] == 0x0F){
        localCurrentStepSize = stepSizes[2];
      } else if (keyArray[0] == 0x07 && keyArray[1] == 0x0F && keyArray[2] == 0x0F){
        localCurrentStepSize = stepSizes[3];
      } else if (keyArray[0] == 0x0F && keyArray[1] == 0x0E && keyArray[2] == 0x0F){
        localCurrentStepSize = stepSizes[4];
      } else if (keyArray[0] == 0x0F && keyArray[1] == 0x0D && keyArray[2] == 0x0F){
        localCurrentStepSize = stepSizes[5];
      } else if (keyArray[0] == 0x0F && keyArray[1] == 0x0B && keyArray[2] == 0x0F){
        localCurrentStepSize = stepSizes[6];
      } else if (keyArray[0] == 0x0F && keyArray[1] == 0x07 && keyArray[2] == 0x0F){
        localCurrentStepSize = stepSizes[7];
      } else if (keyArray[0] == 0x0F && keyArray[1] == 0x0F && keyArray[2] == 0x0E){
        localCurrentStepSize = stepSizes[8];
      } else if (keyArray[0] == 0x0F && keyArray[1] == 0x0F && keyArray[2] == 0x0D){
        localCurrentStepSize = stepSizes[9];
      } else if (keyArray[0] == 0x0F && keyArray[1] == 0x0F && keyArray[2] == 0x0B){
        localCurrentStepSize = stepSizes[10];
      } else if (keyArray[0] == 0x0F && keyArray[1] == 0x0F && keyArray[2] == 0x07){
        localCurrentStepSize = stepSizes[11];
      } else if (keyArray[0] == 0x0F && keyArray[1] == 0x0F && keyArray[2] == 0x0F){
        localCurrentStepSize = 0;
      }

      __atomic_store_n(&currentStepSize,localCurrentStepSize,__ATOMIC_RELAXED);
  }
}

void displayUpdateTask(void * pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime= xTaskGetTickCount();
  String keyString;
  while (true) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    if (keyArray[0] == 0x0E && keyArray[1] == 0x0F && keyArray[2] == 0x0F){
        keyString = noteNames[0];
      } else if(keyArray[0] == 0x0D && keyArray[1] == 0x0F && keyArray[2] == 0x0F){
        keyString = noteNames[1];
      } else if (keyArray[0] == 0x0B && keyArray[1] == 0x0F && keyArray[2] == 0x0F){
        keyString = noteNames[2];
      } else if (keyArray[0] == 0x07 && keyArray[1] == 0x0F && keyArray[2] == 0x0F){
        keyString = noteNames[3];
      } else if (keyArray[0] == 0x0F && keyArray[1] == 0x0E && keyArray[2] == 0x0F){
        keyString = noteNames[4];
      } else if (keyArray[0] == 0x0F && keyArray[1] == 0x0D && keyArray[2] == 0x0F){
        keyString = noteNames[5];
      } else if (keyArray[0] == 0x0F && keyArray[1] == 0x0B && keyArray[2] == 0x0F){
        keyString = noteNames[6];
      } else if (keyArray[0] == 0x0F && keyArray[1] == 0x07 && keyArray[2] == 0x0F){
        keyString = noteNames[7];
      } else if (keyArray[0] == 0x0F && keyArray[1] == 0x0F && keyArray[2] == 0x0E){
        keyString = noteNames[8];
      } else if (keyArray[0] == 0x0F && keyArray[1] == 0x0F && keyArray[2] == 0x0D){
        keyString = noteNames[9];
      } else if (keyArray[0] == 0x0F && keyArray[1] == 0x0F && keyArray[2] == 0x0B){
        keyString = noteNames[10];
      } else if (keyArray[0] == 0x0F && keyArray[1] == 0x0F && keyArray[2] == 0x07){
        keyString = noteNames[11];
      } else if (keyArray[0] == 0x0F && keyArray[1] == 0x0F && keyArray[2] == 0x0F){
        keyString = "";
      } else {
        keyString = "";
      }

    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Helllo World!");  // write something to the internal memory
    u8g2.setCursor(2,20);
    u8g2.print(keyArray[0], HEX);
    u8g2.print(keyArray[1], HEX);
    u8g2.print(keyArray[2], HEX);
    u8g2.drawStr(2,30, keyString.c_str());
    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }

}

void sampleISR(){
  static int32_t phaseAcc = 0;
  phaseAcc += currentStepSize;

  int32_t Vout = phaseAcc >> 24;

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

  //Initialise timer
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer= new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  //Initialise Keyscanning Loop
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,/* Function that implements the task */
    "scanKeys",/* Text name for the task */
    64,      /* Stack size in words, not bytes*/
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