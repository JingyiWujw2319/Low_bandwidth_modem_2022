/*
Note: 
Each task is using its own variable, so no memory resources protection needed 
Could use atomic access for uint8_t variables in the future to speed up process 
*/
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <STM32FreeRTOS.h>
#include <iostream>
#include <queue>
// Pin setup 
const int OUTA = A2;
// DAC 1/1 is A2(PA_4)    DAC 1/2 is D13
const int INA = A5;
// PC_0(A5) is (ADC1/10)
//ADC is now PC_5 (ADC1/15) 
// maybe internal bug in platformio 
const int userButton = PC13;
const int userLED = D13;
const int DEBUG_send_win = D2;
const int DEBUG_rece_win = D3;

HardwareSerial Serial1(PC_11 ,PC_10);
// D2 is UART1_RX, D8 is UART1_TX (located on the right)
// PC_11 is UART3_RX, PC_10 is UART3_TX (located on left) 

// Variable Initialization 

// DAC variables 
/// 1: idle
/// 2: load message
/// 3: sending 
/// 4: Interval 
uint8_t sendStep = 1;
int32_t Vout = 0;
volatile uint8_t sendBit = 0; 
volatile uint8_t sendByte = 0; 
volatile uint8_t sendBitIdx = 0; 
// FSK extra variable
uint8_t cycle_index = 0;

// ADC variables
/// 1: idle 
/// 2: receiving 
/// 3: ending (parity in the future)
volatile uint8_t receiveStep = 1;
volatile uint16_t adcValPre = 0;
volatile uint32_t adcDiff =0;
volatile uint8_t receiveByte = 0;
volatile uint8_t receiveBitIdx = 0;
volatile uint8_t receiveCount =0;
// FSK extra variable
volatile uint8_t prevStamp = 0; 

#define WAVEBUFFERLENGTH  50
#define DACRANGE  230
#define DACRANGEMID  115
uint8_t sine_index = 0;
int16_t WaveBuffer[WAVEBUFFERLENGTH];
std::queue<uint8_t> messageBuffer[64];
std::queue<uint8_t> receiveBuffer[64];
bool buttonPressedBefore = false;
/// mainMode = true is ASK 
/// mainMode = false is FSK 
bool mainMode = true;

SemaphoreHandle_t receiveBufferMutex;

void sampleISR(){
    // This function will be called 22000 Hz 
    if(sendStep==1){
        if(!messageBuffer->empty()){
            sendStep=2;
            digitalWrite(DEBUG_send_win,HIGH);
        }else{
            if(!receiveBuffer->empty()){
                // Used two queue to prevent interference 
                // receivebuffer only use push operation 
                std::swap(messageBuffer,receiveBuffer);
            }else{
                analogWrite(OUTA,DACRANGEMID);
                // analogWrite(OUTA,0);
            }
        }
    }else if(sendStep==2){
        sendByte=(messageBuffer->front()|0b10000000);
        messageBuffer->pop();
        sendStep = 3;
        cycle_index = 0;
        sine_index = 0;
        sendBit = bitRead(sendByte,7);
        sendBitIdx = 1;

    }else if(sendStep==3){
        if(mainMode){
            if(sendBit==1){
                analogWrite(OUTA,DACRANGEMID+WaveBuffer[cycle_index]);
            }else{
                analogWrite(OUTA,DACRANGEMID);
            }
        }else{
            analogWrite(OUTA,DACRANGEMID+WaveBuffer[sine_index]);
            if(sendBit==1){
                sine_index+=2;
            }else{
                sine_index+=1;
            }
            if(sine_index>=WAVEBUFFERLENGTH-1 ){
                sine_index=0;
            }
        }
        cycle_index+=1;

        if(cycle_index>=WAVEBUFFERLENGTH-1){
            sendBit = bitRead(sendByte,7-sendBitIdx);
            sine_index=0;
            cycle_index=0;
            sendBitIdx+=1;
            if(sendBitIdx == 9){
                sendStep= 4 ;
                sendBitIdx=0;
            }
        }
    }else if(sendStep==4){
        // analogWrite(OUTA,DACRANGE);
        analogWrite(OUTA,DACRANGEMID);
        sine_index+=1;
        if(sine_index>=WAVEBUFFERLENGTH-1){
            sendStep=1;
            digitalWrite(DEBUG_send_win,LOW);
        }
    }
}

void demodulation(){
    uint16_t currentVal = analogRead(INA);
    // Serial2.println(currentVal);
    if(receiveStep==1){
        if(currentVal>2400){
            digitalWrite(DEBUG_rece_win,HIGH);
            receiveStep=2;
            receiveCount=0;
            receiveBitIdx=0;
            adcValPre = currentVal;
            adcDiff=0;
            prevStamp = 0; 
            // This is the first sample of the sine 
            if(mainMode){
                receiveCount = 0;
            }else{
                receiveCount = 1;
            }
        }
    }else if(receiveStep==2){
        if(mainMode){
            adcDiff += abs(currentVal-adcValPre);
            adcValPre=currentVal;
        }else{
            if(currentVal<1800){
                if((receiveCount-prevStamp)<=4 &(receiveCount-prevStamp)>1){
                    receiveByte+=1;
                }
                prevStamp = receiveCount;
            }
        }
        receiveCount+=1;
        if(receiveCount>=8){
            if(mainMode){
                if(adcDiff>4000){
                    receiveByte+=1;
                }
            }
            receiveByte=receiveByte<<1;
            receiveBitIdx+=1;
            receiveCount=0;
            adcDiff = 0;
            if(receiveBitIdx>=8){
                // Received 8 bits 
                prevStamp = 0; 
                receiveByte=receiveByte>>1;
                char tempchar = receiveByte;
                Serial2.print(tempchar);
                receiveByte = 0;
                digitalWrite(DEBUG_rece_win,LOW);
                receiveBitIdx=0;
                receiveStep=3;
            }
        }
    }else if(receiveStep=3){
        receiveStep=1;
    }
}


void readFromPCTask(void * pvParameters){
    const TickType_t xFrequency = 12/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint16_t raw_value;
    while (1)
    {
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        if(Serial2.available()){
            uint8_t x = Serial2.read();
            xSemaphoreTake(receiveBufferMutex, portMAX_DELAY);
            receiveBuffer->push(x);
            xSemaphoreGive(receiveBufferMutex);
        }
    }
}

void switchModulationTask(void * pvParameters){
    const TickType_t xFrequency = 500/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint16_t raw_value;
    while (1)
    {
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        if(digitalRead(userButton)==LOW){
            if(!buttonPressedBefore){
                mainMode = !mainMode;
                digitalWrite(userLED,HIGH);
                buttonPressedBefore = true;
                Serial2.println();
                if(mainMode){
                    Serial2.println("Using Amplitude modulation");
                }else{
                    Serial2.println("Using Frequency modulation");
                }
            }
            // Switch modulation scheme
        }else{
            digitalWrite(userLED,LOW);
            buttonPressedBefore = false; 
        }
    }
}

void setup() {
    pinMode(OUTA, OUTPUT);
    pinMode(INA, INPUT_ANALOG);
    pinMode(DEBUG_rece_win,OUTPUT);
    pinMode(DEBUG_send_win,OUTPUT);
    pinMode(userButton,INPUT);
    pinMode(userLED,OUTPUT);
    digitalWrite(DEBUG_rece_win,LOW);
    digitalWrite(DEBUG_send_win,LOW);
    analogReadResolution(12);
    Serial2.begin(1200);
    Serial2.println("Hello!");
    // Serial1.begin(9600);

// Sine wave parameters generation
for (int n = 0; n < WAVEBUFFERLENGTH; n++){
    const float   amplitude = DACRANGEMID-1;
    int32_t val = ( sin( 2 * PI * n / WAVEBUFFERLENGTH  )) * amplitude;
    WaveBuffer[n] = val;
    }

// Interupt Setup 
    TIM_TypeDef *Instance = TIM1;
    HardwareTimer *sampleTimer = new HardwareTimer(Instance);
    sampleTimer->setOverflow(40000, HERTZ_FORMAT);
    sampleTimer->attachInterrupt(sampleISR);
    sampleTimer->resume();

    TIM_TypeDef *Instance1 = TIM2;
    HardwareTimer *sampleTimer1 = new HardwareTimer(Instance1);
    sampleTimer1->setOverflow(5150, HERTZ_FORMAT);
    sampleTimer1->attachInterrupt(demodulation);
    sampleTimer1->resume();

// RTOS implementation
    // Variable protection 
    receiveBufferMutex = xSemaphoreCreateMutex();
    // Creating tasks 
    TaskHandle_t readFromPCHandle = NULL; 
    xTaskCreate(
        readFromPCTask,    /* Function that implements the task */ 
        "readFromPC",      /* Text name for the task */
        64,                /* Stack size in words, not bytes */ 
        NULL,              /* Parameter passed into the task */
        1,                 /* Task priority */
        &readFromPCHandle  /* Pointer to store the task handle */
    );

    TaskHandle_t switchModulationHandle = NULL; 
    xTaskCreate(
        switchModulationTask,    /* Function that implements the task */ 
        "switchModulation",      /* Text name for the task */
        64,                /* Stack size in words, not bytes */ 
        NULL,              /* Parameter passed into the task */
        3,                 /* Task priority */
        &readFromPCHandle  /* Pointer to store the task handle */
    );
    vTaskStartScheduler();
}

void loop() {
}