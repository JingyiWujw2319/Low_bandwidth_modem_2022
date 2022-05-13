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
uint8_t gap = 1;


#define WAVEBUFFERLENGTH  128
uint8_t sine_index = 0;
int16_t WaveBuffer[WAVEBUFFERLENGTH];
std::queue<uint8_t> messageBuffer[32];

void sampleISR(){
    // This function will be called 22000 Hz 
    if (waveform_mode==0){
        if (currentStepSize!=0){
            phaseAcc += currentStepSize * 195225;
            int32_t Vout_intermediate = 0;
            Vout_intermediate += (phaseAcc >> 24);
            // Vout_intermediate = Vout_intermediate >> (8 - int(volume_local) / 2);
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
        analogWrite(OUTA,WaveBuffer[sine_index]+128);
        if(sine_index>=WAVEBUFFERLENGTH-gap){
            sine_index=0;
        }else{
            sine_index+=gap;
        }
    }

    // ADC Demodulating section 
    // int adc_raw_val = analogRead(INA); //Range from 0 to 1024 

    

}

void readFromPCTask(void * pvParameters){
    const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
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
                gap = (x[1]-'0');
            }

            Serial2.print("Read from PC: ");
            Serial2.println(x);
        }
        //   Range from 0 to 1024 
        if(Serial1.available()){
            String x = Serial1.readStringUntil('\n');
            for(int i=1;i<=int(x[0]);i++){
                if(x[i]=='0'){
                    messageBuffer->push(0);
                }else{
                    messageBuffer->push(1);
                }
                // currentStepSize = (x[1]-'0')*100;
            } 
            Serial2.print("Sending: ");
            Serial2.println(x);
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
    //int32_t val = ( sin( 2 * PI * frequency / SAMPLINGFREQUENCY * n )) * amplitude;
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
    // Serial2.println("Executing task");
}