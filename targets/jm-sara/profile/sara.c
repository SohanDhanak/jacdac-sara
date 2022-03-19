#include "jdprofile.h"
#include "interfaces/jd_pins.h"
#include "jdstm.h"
#include <stdlib.h>
#include <stdio.h>

// Edit the string below to match your company name, the device name, and hardware revision.
// Do not change the 0x3.... value, as that would break the firmware update process.
FIRMWARE_IDENTIFIER(0x367a3681, "Sohan Dhanak. Sara Rev.A");

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))
#define BUFFER_SIZE 256
#define SOCKET_TCP 6
#define SOCKET_UDP 17

//Not sure why but these are necessary definitions
const uint32_t APBPrescTable[8]; 
const uint32_t AHBPrescTable[16]; 

//Buffers for DMA transfer for USART
uint8_t rxBuffer[BUFFER_SIZE];
uint16_t bufferPointer;
uint16_t readPointer;
int socketNum;

//General commands
const char *at = "AT\r";
const char *ok = "OK\r\n";
const char *echo = "ATE0\r";
const char *powerOff = "AT+CPWROFF\r";
const char *ccid = "AT+CCID\r";
const char *gcap = "AT+GCAP\r";
const char *opSel = "AT+COPS?\r";
const char *cimi = "AT+CIMI\r";
const char *profile = "AT+UMNOPROF?\r";
const char *gpioSIM = "AT+UGPIOC=42,7\r";
const char *cfun = "AT+CFUN=16\r";
const char *gpioRead = "AT+UGPIOR=42\r";

//Socket commands
const char *createSocket = "AT+USOCR\r";

//Initialisation funcs
void setupUsart(void);
void setupPower(void);

//USART funcs
void printTX(const uint8_t *data, uint16_t numbytes);
bool byteAvailable(void);
uint8_t readRXByte(void);
int testAT(void);

//IP functions
int socket(int protocol, uint16_t port);
int closeSocket(int socket);
void sendTo(int socket, uint8_t *data, int dataSize, const char *addr, int port);

/*
 * Initial method to be run.
 * Setup happens once,
 * code in while runs after
 */
void app_init_services() {
    //Main Setup
    pin_setup_output(PIN_AN);
    pin_set(PIN_AN, 0);
    pin_setup_output(PIN_MISO);
    pin_set(PIN_MISO, 0);
    pin_setup_output(PIN_RST);
    pin_set(PIN_RST, 0);
    setupUsart();
    // setupPower();

    // int currentSocket = 2;
    // const char *testData = "Binary data";
    // const char *addr = "255.255.255.255";
    // int port = 65535;
    //int port = 8080;
    printTX((uint8_t *)echo, strlen(echo));
    target_wait_us(1000000);
    printTX((uint8_t *) gpioSIM, strlen(gpioSIM));
    target_wait_us(20000);
    printTX((uint8_t *) cfun, strlen(cfun));
    target_wait_us(20000);
    //socketNum = socket(SOCKET_UDP, 80);
    //printTX((uint8_t *) powerOff, strlen(powerOff));
    target_wait_us(20000);

    //Loop
    while(1){
        printTX((uint8_t *)gpioRead, strlen(gpioRead));
        //printTX((uint8_t *)testCom, dataLen);
        //sendTo(currentSocket, (uint8_t *) testData, strlen(testData), addr, port);
        // socketNum = socket(SOCKET_UDP,port);
        // if(socketNum > 0){
        //     pin_set(PIN_MISO, 1);
        //     target_wait_us(50000);
        //     pin_set(PIN_MISO, 0);
        // }
        target_wait_us(1000000);
    }
}

/*
 * Setup for USART2 on STM32G0
 * TX -> PA2
 * RX -> PA3
 * RX on DMA (rxBuffer)
 */
void setupUsart(void){
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_USART_InitTypeDef USART_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    /*
     * USART2 GPIO
     *
     * PA2 = USART2_TX
     * PA3 = USART2_RX
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_USART2_RX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);

    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_RECEIVE));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)rxBuffer);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ARRAY_LEN(rxBuffer));

    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

    NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /*
     * USART2 Config
     * Baud Rate = 9600
     * 8 bits of data
     * 1 stop bit    
     */
    USART_InitStruct.BaudRate = 9600;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART2, &USART_InitStruct);
    LL_USART_EnableDMAReq_RX(USART2);
    LL_USART_EnableIT_IDLE(USART2);

    CLEAR_BIT(USART2->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
    CLEAR_BIT(USART2->CR3, (USART_CR3_SCEN | USART_CR3_IREN | USART_CR3_HDSEL));

    NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART2_IRQn);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_USART_Enable(USART2);

    bufferPointer = 0;
    readPointer = 0;
}

/*
 * Send a 2.7s low signal to pin connecting to PWR_ON
 * Pullup on pin should set it back to high
 */
void setupPower(void){
    pin_setup_output(PIN_MISO);
    pin_set(PIN_MISO, 0);
    target_wait_us(2700000);
    pin_setup_input(PIN_MISO, 0);
}


void DMA1_Channel1_IRQHandler(void){
    if (LL_DMA_IsActiveFlag_HT1(DMA1) || LL_DMA_IsActiveFlag_TC1(DMA1)) {
        LL_DMA_ClearFlag_HT1(DMA1);            
        bufferPointer = ARRAY_LEN(rxBuffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1) - 1;           
    }
}


void USART2_IRQHandler(void){
    if(LL_USART_IsActiveFlag_IDLE(USART2)){
        LL_USART_ClearFlag_IDLE(USART2);
        bufferPointer = ARRAY_LEN(rxBuffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1) - 1;
    }
}


void printTX(const uint8_t *data, uint16_t numbytes){
    const uint8_t *d = data;
    //Send each byte of data numbyte times
    for(; numbytes > 0; --numbytes, ++d) {
        LL_USART_TransmitData8(USART2, *d);
        while(!LL_USART_IsActiveFlag_TXE(USART2)) {}
    }
    while(!LL_USART_IsActiveFlag_TC(USART2)) {}

}


int socket(int protocol, uint16_t port){
    int socketNumber = -1;

    //Create and send AT command
    char *command = (char *) calloc(strlen(createSocket) + 16, sizeof(char));
    sprintf(command, "%s=%d,%d", createSocket, protocol, port);
    printTX((uint8_t *) command, strlen(command));
    target_wait_us(50000);
    free(command);

    //Read the response
    char *response = (char *) calloc(32, sizeof(char));
    uint8_t index = 0;
    uint8_t currentByte;
    while(byteAvailable()){
        currentByte = readRXByte();
        response[index] = (char) currentByte;
        index++;
    }

    //Extract socket number from response
    sscanf(response, "\r\nOK\r\n+USOCR: %d\r\n", &socketNumber);
    free(response);

    return socketNumber;
}


int closeSocket(int socket){
    const char *closeSocket = "AT+USOCL";
    const char *ok = "OK";

    //Create and send AT command
    char *command = (char *) calloc(strlen(closeSocket) + 16, sizeof(char));
    sprintf(command, "%s=%d",closeSocket, socket);
    printTX((uint8_t *) command, strlen(command));
    target_wait_us(50000);
    free(command);

    //Read the response
    char *response = (char *) calloc(16, sizeof(char));
    uint8_t index = 0;
    uint8_t currentByte;
    while(byteAvailable()){
        currentByte = readRXByte();
        response[index] = (char) currentByte;
        index++;
    }

    //Check if response was 'OK'
    if(strcmp(response, ok) == 0){
        free(response);
        return 1;
    } else {
        free(response);
        return -1;
    }
}

void sendTo(int socket, uint8_t *data, int dataLen, const char *addr, int port){
    const char *sendToCommand = "AT+USOST";
    const char *binaryData = "@";

    char *command = (char *) calloc(strlen(sendToCommand) + 64, sizeof(char));
    sprintf(command, "%s=%d,\"%s\",%d,%d",sendToCommand,socket,addr,port,dataLen);
    printTX((uint8_t *) command, strlen(command));
    target_wait_us(50000);
    free(command);

    //Read the response
    char *response = (char *) calloc(16, sizeof(char));
    uint8_t index = 0;
    uint8_t currentByte;
    while(byteAvailable()){
        currentByte = readRXByte();
        response[index] = (char) currentByte;
        index++;
    }

    if(strcmp(response, binaryData) == 0){
        printTX(data, dataLen);
    }
}


bool byteAvailable(void){
    //Checks if the read pointer is past the buffer pointer
    if(readPointer != ((bufferPointer+1) % BUFFER_SIZE)){
        return true;
    }
    return false;
}


uint8_t readRXByte(void){
    //Returns the current avaiable byte
    if(byteAvailable()){
        uint8_t current = rxBuffer[readPointer];
        readPointer = (readPointer + 1) % BUFFER_SIZE;
        return current;
    }
    return 0;
}