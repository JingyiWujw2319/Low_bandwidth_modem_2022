#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <STM32FreeRTOS.h>
#include <iostream>
#include <queue>
// Pin setup 
const int OUTA = D13;
const int INA = D11;//ADC is now PC_5 (ADC1/15) 
// maybe internal bug in platformio 
HardwareSerial Serial1(D2 ,D8);

// Variable Initialization 
int32_t Vout = 0;
volatile int32_t sendStepSize=0;
static int32_t phaseAcc = 0;
const String wave_display[] = {"Sawtooth", "Triangular", "Square"};
volatile uint8_t waveform_mode = 0;
volatile uint8_t sendBit = 0; 
volatile uint8_t sendByte = 0; 
volatile uint8_t sendBitIdx = 0; 
uint8_t sendStep = 1;
/*
1: idle
2: load message
3: sending 
4: Interval 
*/
/// ADC variables
uint16_t adcValPre = 0;
uint32_t adcDiff =0;
uint8_t receiveByte = 0;
uint8_t receiveBitIdx = 0;
uint8_t receiveStep = 1;
uint8_t receiveCount =0;
uint8_t intervalDetected = false; 
/*
1: idle 
2: receiving 
3: ending (parity in the future)
*/


#define WAVEBUFFERLENGTH  115
#define DACRANGE  230
#define DACRANGEMID  115
uint8_t sine_index = 0;
int16_t WaveBuffer[WAVEBUFFERLENGTH];
std::queue<uint8_t> messageBuffer[32];
std::queue<uint8_t> receiveBuffer[32];
uint8_t testByte = 0b00000000;



void sampleISR(){
    // This function will be called 22000 Hz 
    if (waveform_mode==0){
        if (sendStepSize!=0){
            phaseAcc += sendStepSize * 195225;
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
            analogWrite(OUTA, Vout+DACRANGEMID);
            // Serial2.println(raw_value);
        }
    }else if(waveform_mode==1){
        if(sendStep==1){
            if(!messageBuffer->empty()){
                sendStep=2;
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
        }else if(sendStep==2){
            sendByte=messageBuffer->front();
            messageBuffer->pop();
            sendStep = 3;
            sine_index = 0;
            sendBit = bitRead(sendByte,7);
            sendBitIdx = 1;

        }else if(sendStep==3){
            if(sendBit==1){
                analogWrite(OUTA,DACRANGEMID+WaveBuffer[sine_index]);
            }else{
                analogWrite(OUTA,DACRANGEMID);
            }
            sine_index+=1;
            if(sine_index>=WAVEBUFFERLENGTH-1){
                sendBit = bitRead(sendByte,7-sendBitIdx);
                sine_index=0;
                sendBitIdx+=1;
                if(sendBitIdx == 9){
                    sendStep= 4 ;
                    sendBitIdx=0;
                }
            }
        }else if(sendStep==4){
            analogWrite(OUTA,DACRANGE);
            sine_index+=1;
            if(sine_index>=WAVEBUFFERLENGTH-1){
                sendStep=1;
                // analogWrite(OUTA,128);
            }
        }
    }

    // ADC Demodulating section 
    // int adc_raw_val = analogRead(INA); //Range from 0 to 1024 
}

void demodulation(){
    uint16_t currentVal = analogRead(INA);
    if(receiveStep==1){
        if(currentVal>100){
            // Serial1.println("Byte detected");
            digitalWrite(D3,HIGH);
            receiveStep=2;
            receiveCount=0;
            receiveBitIdx=0;
            adcValPre = currentVal;
            adcDiff=0;
            // This is the first sample of the sine 
        }
    }else if(receiveStep==2){
        // receiving 9 samples 
        adcDiff += abs(currentVal-adcValPre);
        adcValPre=currentVal;
        receiveCount+=1;
        if(receiveCount>=11){
            // Serial1.print("idx ");
            // Serial1.print(receiveBitIdx);
            // Serial1.print(" : ");
            // Serial1.println(adcDiff);
            if(adcDiff>4000){
                receiveByte+=1;
            }
            receiveByte=receiveByte<<1;
            receiveBitIdx+=1;
            receiveCount=0;
            adcDiff = 0;
            if(receiveBitIdx>=8){
                // Received 8 bits 
                receiveByte=receiveByte>>1;
                char tempchar = receiveByte;
                if(receiveByte==13){
                    Serial1.println(" ");
                }else{
                    Serial1.print(tempchar);
                }
                // Serial1.println(receiveByte,BIN);
                receiveByte = 0;
                digitalWrite(D3,LOW);
                receiveBitIdx=0;
                receiveStep=3;
                // Serial1.println(receiveBitIdx);
            }
        }
    }else if(receiveStep=3){
        // Serial2.println(currentVal);
        if(currentVal>3000){
            intervalDetected = true;
            return;
        }else{
            if(intervalDetected){
                receiveStep=1;
                intervalDetected=false;
            }
        }
    }
    // Serial1.println(adcValPre);
}


void readFromPCTask(void * pvParameters){
    const TickType_t xFrequency = 25/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint16_t raw_value;
    while (1)
    {
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        if(Serial2.available()){
            String x = Serial2.readStringUntil('\n');
            if(x[0]=='S'){
                waveform_mode =0;
                sendStepSize = (x[1]-'0')*100;
            }
            if(x[0]=='C'){
                waveform_mode =1;
            }
            if(x[0]=='a'){
                Serial2.println("Pushing byte into buffer");
                receiveBuffer->push(testByte);
                receiveBuffer->push('h');
                receiveBuffer->push('o');
                receiveBuffer->push('l');
                receiveBuffer->push('a');
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
            Serial2.print("Read from UART: ");
            Serial2.println(x);
            // Serial2.print("Received: ");
            // Serial2.print(x);
        }
    }
    
}

void setup() {
    pinMode(OUTA, OUTPUT);
    pinMode(INA, INPUT_ANALOG);
    pinMode(D3,OUTPUT);
    digitalWrite(D3,LOW);
    analogReadResolution(12);
    Serial2.begin(9600);
    Serial2.println("Hello!");
    Serial1.begin(9600);

// Sine wave parameters generation
for (int n = 0; n < WAVEBUFFERLENGTH; n++){
    const float   amplitude = DACRANGEMID-1;
    int32_t val = ( sin( 2 * PI * n / WAVEBUFFERLENGTH  )) * amplitude;
    WaveBuffer[n] = val;
    }

// Interupt Setup 
    TIM_TypeDef *Instance = TIM1;
    HardwareTimer *sampleTimer = new HardwareTimer(Instance);
    sampleTimer->setOverflow(34500, HERTZ_FORMAT);
    sampleTimer->attachInterrupt(sampleISR);
    sampleTimer->resume();

    TIM_TypeDef *Instance1 = TIM2;
    HardwareTimer *sampleTimer1 = new HardwareTimer(Instance1);
    sampleTimer1->setOverflow(3000, HERTZ_FORMAT);
    sampleTimer1->attachInterrupt(demodulation);
    sampleTimer1->resume();

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