#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <cmath>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

#define OCTAVE 5
#define DISABLE_THREADS
#define DISABLE_SOUND_ISR
#define DISABLE_CAN_ISR
#define TEST_SCANKEYS


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

//Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

  bool runOnce = false;

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

std::bitset<4> readCols(){
  std::bitset<4> result;

  //set each row select address (RA0, RA1, RA2) low
  //digitalWrite(RA0_PIN, LOW);
  //digitalWrite(RA1_PIN, LOW);
  //digitalWrite(RA2_PIN, LOW);

  //set row select enable high
  //digitalWrite(REN_PIN, HIGH);

  //Read inputs of 4 columns of switch matrix
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);

  // set row select enable low
  //digitalWrite(REN_PIN, LOW);

  return result;
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN, LOW);

  digitalWrite(RA0_PIN, rowIdx & 0x01); 
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);

  digitalWrite(REN_PIN, HIGH);
}

// Global Variables
volatile uint32_t currentStepSize = 0;
const double baseFreq = 440.0;
const double TemperamentRatio = std::pow(2.0, 1.0/12.0);
const uint32_t sampleFreq = 22000;
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;
uint32_t ID = 0x123;  
SemaphoreHandle_t CAN_TX_Semaphore;

void CAN_RX_ISR (uint32_t ID, uint8_t* data) { // to do
	// uint8_t RX_Message_ISR[8];
	// uint32_t ID;
	CAN_RX(ID, data);
	xQueueSendFromISR(msgInQ, data, NULL);
}

void CAN_TX_ISR () { // to do
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

const uint32_t stepSizes[] = {
    static_cast<uint32_t>((std::pow(2.0, 32.0) * (baseFreq * std::pow(TemperamentRatio, -9.0))) / sampleFreq), // C
    static_cast<uint32_t>((std::pow(2.0, 32.0) * (baseFreq * std::pow(TemperamentRatio, -8.0))) / sampleFreq),  // C#
    static_cast<uint32_t>((std::pow(2.0, 32.0) * (baseFreq * std::pow(TemperamentRatio, -7.0))) / sampleFreq),  // D
    static_cast<uint32_t>((std::pow(2.0, 32.0) * (baseFreq * std::pow(TemperamentRatio, -6.0))) / sampleFreq),  // D#
    static_cast<uint32_t>((std::pow(2.0, 32.0) * (baseFreq * std::pow(TemperamentRatio, -5.0))) / sampleFreq),  // E
    static_cast<uint32_t>((std::pow(2.0, 32.0) * (baseFreq * std::pow(TemperamentRatio, -4.0))) / sampleFreq),  // F
    static_cast<uint32_t>((std::pow(2.0, 32.0) * (baseFreq * std::pow(TemperamentRatio, -3.0))) / sampleFreq),  // F#
    static_cast<uint32_t>((std::pow(2.0, 32.0) * (baseFreq * std::pow(TemperamentRatio, -2.0))) / sampleFreq),  // G
    static_cast<uint32_t>((std::pow(2.0, 32.0) * (baseFreq * std::pow(TemperamentRatio, -1.0))) / sampleFreq),  // G#
    static_cast<uint32_t>((std::pow(2.0, 32.0) * (baseFreq * std::pow(TemperamentRatio, 0.0))) / sampleFreq),   // A  
    static_cast<uint32_t>((std::pow(2.0, 32.0) * (baseFreq * std::pow(TemperamentRatio, 1.0))) / sampleFreq),  // A# 
    static_cast<uint32_t>((std::pow(2.0, 32.0) * (baseFreq * std::pow(TemperamentRatio, 2.0))) / sampleFreq)  // B
};

struct {
std::bitset<32> inputs;
int rotation;
const char* notePressed;
SemaphoreHandle_t mutex;
uint8_t RX_Message[8]={0};
} sysState;

void decodeTask(void * pvParameters){ // 736 microseconds when simulating a full message of 8 bytes. 736/32 frfr
  uint8_t RX_Message_Local[8] = {0};

  while(1){
  
    xQueueReceive(msgInQ, RX_Message_Local, portMAX_DELAY);

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    memcpy(sysState.RX_Message, RX_Message_Local,  sizeof(sysState.RX_Message));
    uint32_t localStepSize;
    uint32_t calcStepSize;

    if (sysState.RX_Message[0] == 'P'){ // Key press
      //Convert the note number to a step size
      localStepSize = stepSizes[sysState.RX_Message[2]];
      calcStepSize = localStepSize  * std::pow(2.0, (sysState.RX_Message[1]-4)); //maybe change this to use shift operations
      __atomic_store_n(&currentStepSize, calcStepSize, __ATOMIC_RELAXED);

    }
    else if (sysState.RX_Message[0] == 'R'){ // Key release
      __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED); //set current step size to 0 on release
    }

    xSemaphoreGive(sysState.mutex);

    if(runOnce){
      break;
    }

  }

}

void CAN_TX_Task (void * pvParameters) { // 28529+113/32 microseconds
	uint8_t msgOut[8];
	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
    if(runOnce){
      break;
    }
	}
}

void sampleISR(){ // 303/32 microseconds!
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  //xSemaphoreTake(sysState.mutex, portMAX_DELAY);
  __atomic_store_n(&Vout, (Vout >> (8 - sysState.rotation)), __ATOMIC_RELAXED);
  //Vout = Vout >> (8 - sysState.rotation);
  //xSemaphoreGive(sysState.mutex);
  analogWrite(OUTR_PIN, Vout + 128);
  
}

void scanKeysTask (void * pvParameters){ // 4274us or 4015us as worst case scenario (microseconds) 4015+128/32!!! 
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  std::bitset<2> prevKnob3State(0b00);
  std::bitset<32> prevInput(0b00);
  int lastKey = 0;
  uint8_t TX_Message[8] = {0};

  while(1){

    if(runOnce) {
      // Skip's delay for testing...
    } else {
      vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }

    for(uint8_t row=0; row < 4; row++){ //Now should be able to write to row 3 which includes knobs?

      setRow(row); //select the row select address
      delayMicroseconds(3);

      std::bitset<4> inputData = readCols();
      for (uint8_t col = 0; col < 4; col++) {
        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        sysState.inputs[row * 4 + col] = inputData[col];

        #ifdef TEST_SCANKEYS
          inputData[col] = 0;
        #endif
        xSemaphoreGive(sysState.mutex);
          }
    }

    // Decoding rotation variable from knob 3

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    std::bitset<2> currentKnob3State((sysState.inputs[13] ? 1 : 0) << 1 | (sysState.inputs[12] ? 1 : 0));
    
    if (currentKnob3State != prevKnob3State && (sysState.rotation >=0 && sysState.rotation <= 8)) {
      if (prevKnob3State == 0b00 && currentKnob3State == 0b01)
          sysState.rotation++;
      else if (prevKnob3State == 0b01 && currentKnob3State == 0b00)
          sysState.rotation--;
      else if (prevKnob3State == 0b10 && currentKnob3State == 0b11)
          sysState.rotation--;
      else if (prevKnob3State == 0b11 && currentKnob3State == 0b10)
          sysState.rotation++;    
      else if (prevKnob3State == 0b11 && currentKnob3State == 0b00)
          sysState.rotation++;      
    }
    else if (sysState.rotation < 0) sysState.rotation = 0;
    else if (sysState.rotation > 8) sysState.rotation = 8;
    

    // After decoding complete
    
    prevKnob3State = currentKnob3State;
    xSemaphoreGive(sysState.mutex);

    // Clifford version
    //  __atomic_store_n(&currentStepSize, currentStepSize, __ATOMIC_RELAXED);
    // __atomic_load(&currentStepSize, &currentStepSize, __ATOMIC_RELAXED);

    // if(currentStepSize == 0){
    //   TX_Message[0] = 'R';
    // }
    // else {
    //   TX_Message[0] = 'P';
    // }

    // Kevin's Code!!! WHY FOR LOOP!
    uint32_t currentStepSize_local = 0;
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    // Check pressed keys and update currentStepSize

    for (uint8_t i = 0; i < 12; i++) { 

      sysState.inputs[i] = false;   // This line is for testing worst case execution time only
        
      if (!sysState.inputs[i]) {
        if (not(sysState.inputs[i] & prevInput[i])){
          TX_Message[0] = 'P';
          TX_Message[1] = OCTAVE; // Global Variable
          TX_Message[2] = i;
          lastKey = i;
        }          
        currentStepSize_local = stepSizes[i];
        break;        
      }
      else{
          TX_Message[0] = 'R';
          TX_Message[1] = OCTAVE; //Global Variable
          TX_Message[2] = lastKey;
      }
    }
          
      xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
      prevInput = sysState.inputs;
      xSemaphoreGive(sysState.mutex);
      // u8g2.sendBuffer();      //This line of code breaks everything, need a mutex here for the display to update notes
      __atomic_store_n(&currentStepSize, currentStepSize_local, __ATOMIC_RELAXED);
      
      if(runOnce){
        break;
      }
  }
}


void displayUpdatesTask(void * pvParameters){// 480965/32 MICROSECONDS :o

  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1){

    if(runOnce){
      // skip delaytaskuntil...
    } else {
      vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }

    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Testing");  // write something to the internal memory  

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);

    u8g2.setCursor(2,20);    
    u8g2.print(sysState.inputs.to_ulong(), HEX);
    u8g2.setCursor(60,10);
    u8g2.print(sysState.rotation);
    u8g2.setCursor(66,30);
    u8g2.print((char) sysState.RX_Message[0]);
    u8g2.print(sysState.RX_Message[1]);
    switch (sysState.RX_Message[2]){
      case 0:
        u8g2.print("C");
        break;
      case 1:
        u8g2.print("C#");
        break;
      case 2:
        u8g2.print("D");
        break;
      case 3:
        u8g2.print("D#");
        break;
      case 4:
        u8g2.print("E");
        break;
      case 5:
        u8g2.print("F");
        break;
      case 6:
        u8g2.print("F#");
        break;
      case 7:
        u8g2.print("G");
        break;
      case 8:
        u8g2.print("G#");
        break;
      case 9:
        u8g2.print("A");
        break;
      case 10:
        u8g2.print("A#");
        break;
      case 11:
        u8g2.print("B");
        break;    
      }
        

    xSemaphoreGive(sysState.mutex);   

    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);

    if(runOnce) {
      break;
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

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  //Initialising CAN Bus
  CAN_Init(true);

  #ifndef DISABLE_CAN_ISR
    CAN_RegisterRX_ISR(CAN_RX_ISR);  
    CAN_RegisterTX_ISR(CAN_TX_ISR);
  #endif

  setCANFilter(0x123,0x7ff);  
  CAN_Start();  

  //Recieve Queue
  msgInQ = xQueueCreate(36,8);  

  //Send Queue
  msgOutQ = xQueueCreate(384,8);
  
  //Timer
  #ifndef DISABLE_SOUND_ISR
    TIM_TypeDef *Instance = TIM1;
    HardwareTimer *sampleTimer = new HardwareTimer(Instance);
    sampleTimer->setOverflow(22000, HERTZ_FORMAT);
    sampleTimer->attachInterrupt(sampleISR);
    sampleTimer->resume();  
  #endif

  #ifndef DISABLE_THREADS

    TaskHandle_t displayUpdatesHandle = NULL;
    xTaskCreate(
      displayUpdatesTask,		/* Function that implements the task */
      "displayUpdates",		/* Text name for the task */
      256,      		/* Stack size in words, not bytes */
      NULL,			/* Parameter passed into the task */
      1,			/* Task priority */
      &displayUpdatesHandle 
    );	/* Pointer to store the task handle */

    TaskHandle_t scanKeysHandle = NULL;
    xTaskCreate(
      scanKeysTask,		/* Function that implements the task */
      "scanKeys",		/* Text name for the task */
      64,      		/* Stack size in words, not bytes */
      NULL,			/* Parameter passed into the task */
      2,			/* Task priority */
      &scanKeysHandle 
    );	/* Pointer to store the task handle */ 
    
    TaskHandle_t decodeHandle = NULL;
    xTaskCreate(
      decodeTask,
      "Decode Task",
      64,
      NULL,
      3,
      &decodeHandle
    );

    TaskHandle_t CAN_TX_Handle = NULL;
    xTaskCreate(
      CAN_TX_Task,
      "CAN TX Task",
      64,
      NULL,
      4,
      &CAN_TX_Handle
    );

    vTaskStartScheduler();
    
  #endif

  sysState.mutex = xSemaphoreCreateMutex();
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);


  #ifdef TEST_SCANKEYS
    uint32_t startTime;
    runOnce = true;
    // uint32_t startTime = micros();
    uint8_t RX_Message_ISR[8] = {0};
    uint32_t dummyID = 0x123;
    uint32_t totalTime = 0;
    for (int iter = 0; iter < 1; iter++) {

      startTime = micros();
      // CAN_TX_ISR();
      // xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
      // CAN_RX_ISR();

      CAN_RX_ISR(dummyID, RX_Message_ISR);

      // decodeTask(NULL);
      // xQueueSend(msgOutQ, testData, portMAX_DELAY);
      // // CAN_TX_Task(NULL);
      // decodeTask(NULL);
      // sampleISR();

      // CAN_RX_ISR();
      // xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
      // CAN_RX_ISR();

      uint32_t execution_time = micros() - startTime;
      totalTime += execution_time;

    }
    Serial.print("Execution time (microseconds) for 32 iterations of scanKeysTask: ");
    Serial.println(totalTime);
    while(1);
  #endif
}

void loop() {  
}