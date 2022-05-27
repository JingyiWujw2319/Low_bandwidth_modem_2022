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

const int DEBUG_send_win = D2;
const int DEBUG_rece_win = D3;

HardwareSerial Serial1(PC_11 ,PC_10);
// D2 is UART1_RX, D8 is UART1_TX (located on the right)
// PC_11 is UART3_RX, PC_10 is UART3_TX (located on left) 

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
uint16_t ADC_buffer[10] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048};
double filter_output_buffer = 2048;  // holds previous value of filter output for detecting falling filter
uint8_t threshold_reached = 0;         // has the first bit been detected
uint8_t is_recording = 0;              // has data collection begun
uint8_t sample_count = 1;              // counts up to 10 samples for timing in demodulation
uint16_t threshold = 20000;            // threashold for detecting of a bit from sqaure filter
uint8_t data_pointer = 1;              // keeps track of the current bit

double calc_square_filter(uint16_t new_ADC_val){

    for(uint8_t i = 0; i < 9; i++){     //shift ADC values
        ADC_buffer[i] = ADC_buffer[i+1];
    }

    ADC_buffer[9] = new_ADC_val;        //add new ADC value

    uint16_t total = 0;
    
    for(uint8_t i = 0; i < 5; i++){     //sum first 5 values
        total = total + ADC_buffer[i];
    }

    for(uint8_t i = 5; i < 10; i++){   //subtract last 5 values
        total = total - ADC_buffer[i];
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

    uint16_t new_ADC_value = analogRead(INA);
    double filter_output = calc_square_filter(new_ADC_value);

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
    }

    filter_output_buffer = filter_output;

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
                analogWrite(OUTA,DACRANGEMID);
            }
        }
    }else if(sendStep==2){
        sendByte=messageBuffer->front();
        messageBuffer->pop();
        sendStep = 3;
        sine_index = 0;
        sendBit = 1;
        sendBitIdx = 1;

    }else if(sendStep==3){
        if(sendBit==1){
            analogWrite(OUTA,DACRANGEMID+WaveBuffer[sine_index]);
        }else{
            analogWrite(OUTA,DACRANGEMID);
        }
        sine_index+=1;
        if(sine_index>=WAVEBUFFERLENGTH-1){
            sendBit = bitRead(sendByte,8-sendBitIdx);
            sine_index=0;
            sendBitIdx+=1;
            if(sendBitIdx == 10){
                sendStep= 4 ;
                sendBitIdx=0;
            }
        }
    }else if(sendStep==4){
        analogWrite(OUTA,DACRANGEMID);
        sine_index+=1;
        if(sine_index>=WAVEBUFFERLENGTH-1){
            sendStep=1;
        }
    }
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