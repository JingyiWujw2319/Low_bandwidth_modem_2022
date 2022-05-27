#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <STM32FreeRTOS.h>
#include <iostream>
#include <queue>
#include <deque>
// Pin setup 
const int OUTA = D13;
const int INA = D11;//ADC is now PC_5 (ADC1/15) 
// maybe internal bug in platformio 
HardwareSerial Serial1(D2 ,D8);

// Variable Initialization 
int32_t Vout = 0;
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
volatile uint16_t adcValPre = 0;
volatile uint32_t adcDiff =0;
volatile uint8_t receiveByte = 0;
volatile uint8_t receiveBitIdx = 0;
volatile uint8_t receiveStep = 1;
volatile uint8_t receiveCount =0;
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

// Demodulation variables
std::deque<uint16_t> ADC_buffer {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048};
uint16_t filter_output_buffer = 2048;  // holds previous value of filter output for detecting falling filter
uint8_t threshold_reached = 0;         // has the first bit been detected
uint8_t is_recording = 0;              // has data collection begun
uint8_t sample_count = 1;              // counts up to 10 samples for timing in demodulation
uint16_t threshold = 20000;            // threashold for detecting of a bit from sqaure filter
uint8_t data_pointer = 1;              // keeps track of the current bit

uint16_t calc_square_filter(){ //adds first 5 values and subracts last 5 values of ADC_buffer
    uint16_t total = 0;

    for(uint8_t i = 0; i < 5; i++){ //add first five elements to total
        total = total + *(ADC_buffer.begin() + i);
    }

    for(uint8_t i = 5; i < 10; i++){ //subtract last five elements to total
        total = total - *(ADC_buffer.begin() + i);
    }

    return total;
}

void output_byte(uint8_t output){
    char tempchar = output;

    if(output==13){
        Serial1.println(" ");
    }else{
        Serial1.print(tempchar);
    }
}

void demodulation(){

    ADC_buffer.pop_front();                //updating buffer w/ new ADC value
    ADC_buffer.push_back(analogRead(INA)); //

    uint16_t filter_output = calc_square_filter();

    if(is_recording == 1){
        if(sample_count == 10){     //determines when to make a sample
            if(data_pointer == 9){      //outputs data on '9th' bit, this is a stop bit while filter dies down
                output_byte(receiveByte);         
                receiveByte = 0;                 
                is_recording = 0;                 
                threshold_reached = 0;      
            }else{
                if(filter_output > threshold){    
                    receiveByte+=1;                
                }                                 
                receiveByte=receiveByte<<1;       
            }
            data_pointer++;
            sample_count = 1;
        }else{
            sample_count++;
        }
    }else{
        if(filter_output > threshold){ //first bit detected
            threshold_reached = 1;
        }

        if(threshold_reached == 1){ //wait for the filter output to fall, so timing is correct for recording
            if(filter_output < filter_output_buffer){
                is_recording = 1;
                sample_count = 3;
                data_pointer = 1;      
            }
        }

        filter_output_buffer = filter_output;
    }

}

void sampleISR(){
    // This function will be called 22000 Hz 
    if(sendStep==1){
        if(!messageBuffer->empty()){
            sendStep=2;
        }else{
            if(!receiveBuffer->empty()){
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
        }
    }
}

/*
void demodulation(){
    uint16_t currentVal = analogRead(INA);
    if(receiveStep==1){
        if(currentVal>100){
            digitalWrite(D3,HIGH);
            receiveStep=2;
            receiveCount=0;
            receiveBitIdx=0;
            adcValPre = currentVal;
            adcDiff=0;
            // This is the first sample of the sine 
        }
    }else if(receiveStep==2){
        // receiving 10 samples 
        adcDiff += abs(currentVal-adcValPre);
        adcValPre=currentVal;
        receiveCount+=1;
        if(receiveCount>=11){
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
                receiveByte = 0;
                digitalWrite(D3,LOW);
                receiveBitIdx=0;
                receiveStep=3;
            }
        }
    }else if(receiveStep=3){
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
}
*/

void readFromPCTask(void * pvParameters){
    const TickType_t xFrequency = 25/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint16_t raw_value;
    while (1)
    {
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        if(Serial2.available()){
            String x = Serial2.readStringUntil('\n');
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
        if(Serial1.available()){
            uint8_t x = Serial1.read();
            receiveBuffer->push(x);
            Serial2.print("Read from UART: ");
            Serial2.println(x);

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
    Serial2.println("Serial works");
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
        readFromPCTask,    /* Function that implements the task */ 
        "readFromPC",      /* Text name for the task */
        64,                /* Stack size in words, not bytes */ 
        NULL,              /* Parameter passed into the task */
        1,                 /* Task priority */
        &readFromPCHandle  /* Pointer to store the task handle */
    );
    vTaskStartScheduler();
}

void loop() {
}