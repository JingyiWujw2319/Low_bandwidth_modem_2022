#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <STM32FreeRTOS.h>
#include <iostream>
#include <queue>
// Pin setup 
const int OUTA = A4;
const int INA = A3;
HardwareSerial Serial1(PA_10 ,PA_9);

// Variable Initialization 
int32_t Vout = 0;
volatile int32_t currentStepSize=0;
static int32_t phaseAcc = 0;
const String wave_display[] = {"Sawtooth", "Triangular", "Square"};
volatile uint8_t waveform_mode = 0;
volatile uint8_t currentBit = 0; 
volatile uint8_t currentByte = 0; 
volatile uint8_t bitIndex = 0; 
uint8_t currentStep = 1;
/*
1: idle
2: load message
3: sending 
4: Interval 
*/

#define WAVEBUFFERLENGTH  128
uint8_t sine_index = 0;
int16_t WaveBuffer[WAVEBUFFERLENGTH];
std::queue<uint8_t> messageBuffer[32];
std::queue<uint8_t> receiveBuffer[32];
uint8_t testByte = 0b01011110;



void sampleISR(){
    // This function will be called 22000 Hz 
    if (waveform_mode==0){
        if (currentStepSize!=0){
            phaseAcc += currentStepSize * 195225;
            int32_t Vout_intermediate = 0;
            Vout_intermediate += (phaseAcc >> 24);
            if (Vout_intermediate > 0)
            {
                Vout = INT32_MAX >> 24;;
            }
            else
            {
                Vout = INT32_MIN >> 24;
            }
            analogWrite(OUTA, Vout+128);
            // Serial2.println(raw_value);
        }
    }else if(waveform_mode==1){
        if(currentStep==1){
            if(!messageBuffer->empty()){
                currentStep=2;
            }else{
                if(!receiveBuffer->empty()){
                    // messageBuffer=receiveBuffer;
                    // Used two queue to prevent interference 
                    // receivebuffer only use push operation 
                    std::swap(messageBuffer,receiveBuffer);
                }else{
                    analogWrite(OUTA,0);
                }
            }
        }else if(currentStep==2){
            currentByte=messageBuffer->front();
            messageBuffer->pop();
            currentStep = 3;
            sine_index = 0;
            currentBit = bitRead(currentByte,7);
            bitIndex = 1;

        }else if(currentStep==3){
            if(currentBit==1){
                analogWrite(OUTA,128+WaveBuffer[sine_index]);
            }else{
                analogWrite(OUTA,128);
            }
            sine_index+=1;
            if(sine_index>=WAVEBUFFERLENGTH-1){
                currentBit = bitRead(currentByte,7-bitIndex);
                sine_index=0;
                bitIndex+=1;
                if(bitIndex == 9){
                    currentStep= 4 ;
                    bitIndex=0;
                }
            }
        }else if(currentStep==4){
            analogWrite(OUTA,255);
            sine_index+=1;
            if(sine_index>=WAVEBUFFERLENGTH-1){
                currentStep=1;
                // analogWrite(OUTA,128);
            }
        }
    }

    // ADC Demodulating section 
    // int adc_raw_val = analogRead(INA); //Range from 0 to 1024 
}

void readFromPCTask(void * pvParameters){
    const TickType_t xFrequency = 30/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint16_t raw_value;
    while (1)
    {
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        if(Serial2.available()){
            String x = Serial2.readStringUntil('\n');
            if(x[0]=='S'){
                waveform_mode =0;
                currentStepSize = (x[1]-'0')*100;
            }
            if(x[0]=='C'){
                waveform_mode =1;
            }
            if(x[0]=='a'){
                Serial2.println("Pushing byte into buffer");
                receiveBuffer->push(testByte);
            }

            Serial2.print("Read from PC: ");
            Serial2.println(x);
        }
        //   Range from 0 to 1024 
        if(Serial1.available()){
            // String x = Serial1.readStringUntil('\n');
            // messageBuffer->push(x[0]);
            // for(int i=0;i<x.length();i++){
            //     messageBuffer->push(x[i]);
            // }
            // Serial2.print("   Received len ");
            // Serial2.println(x.length());
            uint8_t x = Serial1.read();
            receiveBuffer->push(x);
            // Serial2.print("Received: ");
            // Serial2.print(x);
        }
    }
    
}

void setup() {
    pinMode(OUTA, OUTPUT);
    pinMode(INA, INPUT);
    Serial2.begin(9600);
    Serial2.println("Hello!");
    Serial1.begin(9600);

// Sine wave parameters generation
for (int n = 0; n < WAVEBUFFERLENGTH; n++){
    const float   frequency = 440;
    const float   amplitude = 127;
    int32_t val = ( sin( 2 * PI * n / WAVEBUFFERLENGTH  )) * amplitude;
    WaveBuffer[n] = val;
    }

// Interupt Setup 
    TIM_TypeDef *Instance = TIM1;
    HardwareTimer *sampleTimer = new HardwareTimer(Instance);
    sampleTimer->setOverflow(38400, HERTZ_FORMAT);
    sampleTimer->attachInterrupt(sampleISR);
    sampleTimer->resume();

    // RTOS implementation
    TaskHandle_t readFromPCHandle = NULL; 
    xTaskCreate(
        readFromPCTask, /* Function that implements the task */ 
        "readFromPC", /* Text name for the task */
        64, /* Stack size in words, not bytes */ 
        NULL, /* Parameter passed into the task */
        1, /* Task priority */
        &readFromPCHandle  /* Pointer to store the task handle */
    );
    vTaskStartScheduler();
}

void loop() {
}