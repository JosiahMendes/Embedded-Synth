#include <Arduino.h>
#include <U8g2lib.h>
#include<string.h>
#include <STM32FreeRTOS.h>

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
const String noteNames [] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
volatile uint8_t keyArray[7];
volatile int32_t currentStepSize;
SemaphoreHandle_t keyArrayMutex;

// Knob
volatile uint8_t prev_Knob = 0;
volatile uint8_t current_Knob = 0;
volatile int8_t rotation_increment = 0; // can be negative
volatile int8_t knob3_rotation_variable = 0; // from 0 to 16

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

void findKeywithFunc(void (*func)(notes)) {
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
}

void setStepSize(notes note) {
      int32_t localCurrentStepSize = 0;
      switch(note){
        case None:
          break;
        default:
          localCurrentStepSize = stepSizes[note];
          break;
      }
      __atomic_store_n(&currentStepSize,localCurrentStepSize,__ATOMIC_RELAXED);
}

void setNoteName(notes note) {
      String keyString = "";
      switch(note){
        case None:
          keyString = "Nothing";
          break;
        default:
          keyString = noteNames[note];
          break;
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
      u8g2.setCursor(52,20);
      u8g2.print(rotation_increment, DEC);
      u8g2.setCursor(67,20);
      u8g2.print(knob3_rotation_variable, DEC);

      // transfer internal memory to the display
      u8g2.sendBuffer();          
}

void readKnob(uint8_t prev, uint8_t current) {
  bool b0 = (prev >> 1) & B1;
  bool a0 = (prev >> 0) & B1;
  bool b1 = (current >> 1) & B1;
  bool a1 = (current >> 0) & B1;

  if (b0 == b1 && a1 == a0) {
    // no change 00->00, 01->01, 10->10, 11->11
    rotation_increment = 0;
  } else if (b0 != b1 &&  a1 != a0) {
    // impossible transition: 00->11, 01->10, 10->01, 11->00
    //  q6a: interpret impossible transitions by assuming impossible transition is same as last legal transition: so increase or decrease by 2
    if (rotation_increment == 1 || rotation_increment == 2) {
      rotation_increment = 2;
    } else if (rotation_increment == -1 || rotation_increment == -2) {
      rotation_increment = -2;
    }
  } else if (prev == 0 && current == 1 || prev == 1 && current == 3 || prev == 2 && current == 0 || prev == 3 && current == 2) {
    // +1: 00->01, 01->11, 10->00, 11->10
    rotation_increment = 1;
  } else if (prev == 0 && current == 2 || prev == 1 && current == 0 || prev == 2 && current == 3 || prev == 3 && current == 1) {
    // -1: 00->10, 01->00, 10->11, 11->01
    rotation_increment = -1;
  }
}

void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS; // Q6a: atttempt to improve knob accuracy by increasing sample rate
  TickType_t xLastWakeTime= xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    int32_t localCurrentStepSize = 0;

    for (int i = 0; i < 4; i++) { //expanded to read row 3, which is for the right hand knob
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
    current_Knob = keyArray[3];
    xSemaphoreGive(keyArrayMutex);
    readKnob(prev_Knob, current_Knob);

    int8_t local_knob3_rotation_variable ;
    if (knob3_rotation_variable + rotation_increment > 16) {
      local_knob3_rotation_variable = 16;
    } else if (knob3_rotation_variable + rotation_increment < 0) {
      local_knob3_rotation_variable = 0;
    } else {
      local_knob3_rotation_variable = knob3_rotation_variable + rotation_increment;
    }
    prev_Knob = current_Knob;
    __atomic_store_n(&knob3_rotation_variable,local_knob3_rotation_variable,__ATOMIC_RELAXED);
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
  static int32_t phaseAcc = 0;
  phaseAcc += currentStepSize;

  int32_t Vout = phaseAcc >> 24;
  Vout = Vout >> (8 - knob3_rotation_variable/2); // Volume Control

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