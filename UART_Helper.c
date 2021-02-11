
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/ringbuf.h"
#include <stdlib.h>
#include "esp_log.h"
#include "UART_Helper.h"
#include <Message_Helper.h>
#include "esp_task_wdt.h"
#include <../../../Message_Protocol.h>
#include <Management.h>

static const char *TAG = "uart_events";
#define ECHO_TEST_TXD (GPIO_NUM_4)
#define ECHO_TEST_RXD (GPIO_NUM_5)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define EX_UART_NUM UART_NUM_1

#define BUF_SIZE (1024 * 1)
static TaskHandle_t handleIncomingData_handle;
static RingbufHandle_t uart_buf_handle;

static TaskHandle_t uart_event_task_handle;
static QueueHandle_t uart1_queue;
static void handleIncomingData_StateMachine(uint8_t *len, uint8_t *expectedLength, uint8_t *state, uint16_t *tempCheckSum, uint16_t *tempCalculatedCheckSum, size_t *item_size, char *item, uint8_t *rx_message_buffer);

static void handleIncomingData(void *parameter)
{
    uint8_t len = 0;
    uint8_t expectedLength = 1;
    uint8_t state = 0;
    uint16_t tempCheckSum;
    uint16_t tempCalculatedCheckSum;
    uint8_t rx_message_buffer[255] = {
        0,
    };
    // this loop runs forever to check and parse data that is availble on the uart ring buffer
    while (1)
    {
        esp_task_wdt_reset(); // feed the watchdog so it doesn't kick in.

        size_t item_size;
        char *item = (char *)xRingbufferReceiveUpTo(uart_buf_handle, &item_size, pdMS_TO_TICKS(1000), expectedLength); // read from ring buffer
        if (item == NULL)
        {
            // there is no data available, so nothing to parse
            continue;
        }

        handleIncomingData_StateMachine(&len, &expectedLength, &state, &tempCheckSum, &tempCalculatedCheckSum, &item_size, item, rx_message_buffer);
        vRingbufferReturnItem(uart_buf_handle, (void *)item); //delete read data from ring buffer
    }
    vTaskDelete(NULL);
}
/**
 * @brief read, verify and forward the message received using uart to the cloud
 * 
 * @param len pointer to var to keep track of recieved characters
 * @param expectedLength pointer to var to read sepecific length from ring buffer, should be init to 1 
 * @param tempCheckSum to store the recieved checksum
 * @param tempCalculatedCheckSum store the locally calculated checksum
 * @param item_size length of received data from the ring buffer
 * @param item pointer to received data from the ring buffer
 * 
 */
static void handleIncomingData_StateMachine(uint8_t *len, uint8_t *expectedLength, uint8_t *state, uint16_t *tempCheckSum, uint16_t *tempCalculatedCheckSum, size_t *item_size, char *item, uint8_t *rx_message_buffer)
{

    switch (*state)
    {
    case 0:

        // check if this first byte is message start 1
        if (item[0] == MESSAGE_START1)
        {
            rx_message_buffer[*len] = item[0]; // copy the first item received to the message buffer.
            *len = 1;
            *state = 1;
        }
        break;
    case 1:

        // check if the second byte is message start 2
        if (item[0] == MESSAGE_START2)
        {
            rx_message_buffer[*len] = item[0];
            *len = 2;
            *state = 2;
        }
        else
        {
            // malformed message received; reset all and wait for new data
            *len = 0;
            *state = 0;
        }
        break;

    case UART_MESSAGE_DATA_SIZE_POSITION:
        if (item[0] < MESSAGE_NUMBER_OF_HEADERS || item[0] > sizeof(productInformationMessage)) //if the received data size header is less than the message header size then discard the message
        {
            *state = 0;
            *len = 0;
            *expectedLength = 1;
            break;
        }
        rx_message_buffer[*len] = item[0];
        *len = 3;
        *state = 3;
        *expectedLength = item[0] - UART_MESSAGE_DATA_SIZE_POSITION - 1; //copy the size of the expected message to the variable (-) the already recieved data (3 bytes)
        break;
    case 3:                                                 // getting rest of the message
        memcpy(rx_message_buffer + *len, item, *item_size); // copy the recieved chunk of the message to the message buffer starting from the last position. aka concatenate
        *len += *item_size;
        *expectedLength -= *item_size;
        if (*expectedLength == 0)
        {
            *state = 4;
        }
        else
            break;
    case 4: // check the checksum
        //convert from uint8 to uint16 and copy the received check sum to a temp variable
        *tempCheckSum = rx_message_buffer[8] << 8;
        *tempCheckSum |= rx_message_buffer[7];

        // make the recieved checksum value 0 so it will not be part of the locally calculated checksum
        rx_message_buffer[7] = 0;
        rx_message_buffer[8] = 0;
        //calculate the message's checksum locally
        *tempCalculatedCheckSum = Checksum(rx_message_buffer, rx_message_buffer[UART_MESSAGE_DATA_SIZE_POSITION]);
        if (*tempCalculatedCheckSum != *tempCheckSum)
        {
            for (size_t i = 0; i < *len; i++) // for debug only of status message
            {
                ESP_LOGE("checksum faild ", "%u ", rx_message_buffer[i]);
            }
            printf("recieved checksum was : %hu and the calculated checksum is: %hu \n", *tempCheckSum, *tempCalculatedCheckSum);
            // message is currupt
            printf("message is currupt ");
            memset(rx_message_buffer, 0, *len);
            *len = 0;
            *expectedLength = 1;
            *state = 0;
        }
        else // all good, send message to cloud
        {
            if (((statusUpdateMessage *)rx_message_buffer)->header.messageID == MESSAGE_ID_STATUS_UPDATE) // message is a status update
            {
                Status_Update_Arg statusUpdate;
                statusUpdate.debugStatus = ((statusUpdateMessage *)rx_message_buffer)->debugStatus;
                statusUpdate.status = ((statusUpdateMessage *)rx_message_buffer)->status;
                statusUpdate.YBalance = ((statusUpdateMessage *)rx_message_buffer)->YBalance;
                statusUpdate.temperature = ((statusUpdateMessage *)rx_message_buffer)->temperature;
                statusUpdate.colorH = ((statusUpdateMessage *)rx_message_buffer)->colorH;
                statusUpdate.colorS = ((statusUpdateMessage *)rx_message_buffer)->colorS;
                statusUpdate.range = ((statusUpdateMessage *)rx_message_buffer)->range;
                statusUpdate.position = ((statusUpdateMessage *)rx_message_buffer)->position;
                statusUpdate.mode = ((statusUpdateMessage *)rx_message_buffer)->mode;
                Get_Management_Instance()->Post_Uart_Event(EVENT_ID_UART_HELPER_STATUS_UPDATE, &(statusUpdate), sizeof(statusUpdate));
            }
            else if (((productInformationMessage *)rx_message_buffer)->header.messageID == MESSAGE_ID_PRODUCT_INFORMATION) // message is a product information status update
            {
                Product_Information_Arg productInformation;
                productInformation.justUpdated = ((productInformationMessage *)rx_message_buffer)->justUpdated;
                productInformation.softwareVersionMajor = ((productInformationMessage *)rx_message_buffer)->softwareVersionMajor;
                productInformation.softwareVersionMinor = ((productInformationMessage *)rx_message_buffer)->softwareVersionMinor;
                productInformation.softwareVersionPatch = ((productInformationMessage *)rx_message_buffer)->softwareVersionPatch;
                productInformation.PCBVersionMajor = ((productInformationMessage *)rx_message_buffer)->PCBVersionMajor;
                productInformation.PCBVersionMinor = ((productInformationMessage *)rx_message_buffer)->PCBVersionMinor;
                memcpy(&productInformation.softwaredescription, ((productInformationMessage *)rx_message_buffer)->softwaredescription, SOFTWARE_DESCRIPTION_SIZE);
                productInformation.productType = ((productInformationMessage *)rx_message_buffer)->productType;
                productInformation.productModel = ((productInformationMessage *)rx_message_buffer)->productModel;
                Get_Management_Instance()->Post_Uart_Event(EVENT_ID_UART_HELPER_PRODUCT_INFORMATION, &(productInformation), sizeof(productInformation));
            }
            else if (((responseMessage *)rx_message_buffer)->header.messageID == MESSAGE_ID_RESPONSE_MESSAGE) // message is a response of a command
            {
                Response_Arg response;
                response.ackedMessageNumber = ((responseMessage *)rx_message_buffer)->ackedMessageNumber;
                response.status = ((responseMessage *)rx_message_buffer)->status;
                response.reason = ((responseMessage *)rx_message_buffer)->reason;
                Get_Management_Instance()->Post_Uart_Event(EVENT_ID_UART_HELPER_RESPONSE, &(response), sizeof(response));
            }
            else // message is a unknown
            {
                ESP_LOGI("UART HELPER", "received message ID unknown");
            }

            // reset everything
            memset(rx_message_buffer, 0, *len);
            *len = 0;
            *expectedLength = 1;
            *state = 0;
        }

        break;
    default:
        printf("default");
        break;
    }
}
static void uart_event_task(void *pvParameters)
{
    uint8_t *dtmp = (uint8_t *)malloc(BUF_SIZE);

    uart_event_t event;
    for (;;)
    {
        //Waiting for UART event.
        if (xQueueReceive(uart1_queue, (void *)&event, (portTickType)portMAX_DELAY))
        {
            bzero(dtmp, BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch (event.type)
            {
                // Event of UART receving data
            /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
            case UART_DATA:
                ESP_LOGI(TAG, "[UART DATA]: %d", event.size);

                int number_uart_bytes = uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                //Send an item
                if (number_uart_bytes > 0)
                {
                    //try to send the data to the ring buffer
                    UBaseType_t res = xRingbufferSendFromISR(uart_buf_handle, dtmp, number_uart_bytes, NULL);
                    if (res != pdTRUE)
                    {
                        ESP_LOGI("", "Failed to send item\n");
                    }
                }

                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(EX_UART_NUM);
                xQueueReset(uart1_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider encreasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(EX_UART_NUM);
                xQueueReset(uart1_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                // discard all messages
                uart_flush_input(EX_UART_NUM);
                xQueueReset(uart1_queue);
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            //Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}
void SuspendUartTask()
{
    if (uart_event_task_handle != NULL)
        vTaskSuspend(uart_event_task_handle);
}
void ResumeUartTask()
{
    if (uart_event_task_handle != NULL)
        vTaskResume(uart_event_task_handle);
}
void ResetUartQueue()
{
    if (uart1_queue != NULL)
        xQueueReset(uart1_queue); // discard the events on uart
}
void StartUart()
{

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE, 0, 20, &uart1_queue, 0);
    // setup the task for uart handling
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, &uart_event_task_handle);
    uart_buf_handle = xRingbufferCreate(BUF_SIZE, RINGBUF_TYPE_BYTEBUF);

    xTaskCreate(handleIncomingData, "handleIncomingData_task", 1024 * 5, NULL, 13, &handleIncomingData_handle);
}
void Create_SendMessage_ChangeStatus(uint8_t status, uint8_t messageNumber)
{
    changeStatusMessage message = CreateMessage_ChangeStatus(status, messageNumber);
    uart_write_bytes(UART_NUM_1, (const char *)&message, message.header.messageDataSize);
}
void Create_SendMessage_ChangeColor(uint16_t H, uint8_t S, uint8_t messageNumber)
{
    changeColorMessage message = CreateMessage_ChangeColor(H, S, messageNumber);
    uart_write_bytes(UART_NUM_1, (const char *)&message, message.header.messageDataSize);
}
void Create_SendMessage_ChangeYBalance(uint8_t balance, uint8_t messageNumber)
{
    changeYBalanceMessage message = CreateMessage_ChangeYBalance(balance, messageNumber);
    uart_write_bytes(UART_NUM_1, (const char *)&message, message.header.messageDataSize);
}
void Create_SendMessage_ChangeTemperature(uint16_t temperature, uint8_t messageNumber)
{
    changeTemperatureMessage message = CreateMessage_ChangeTemperature(temperature, messageNumber);
    uart_write_bytes(UART_NUM_1, (const char *)&message, message.header.messageDataSize);
}
void Create_SendMessage_ChangeMode(uint8_t mode, uint8_t messageNumber)
{
    changeModeMessage message = CreateMessage_ChangeMode(mode, messageNumber);
    uart_write_bytes(UART_NUM_1, (const char *)&message, message.header.messageDataSize);
}

void Create_SendMessage_GetProductInformation(uint8_t messageNumber)
{
    uartMessageHeader message = CreateMessage_GetProductInformation(messageNumber);
    uart_write_bytes(UART_NUM_1, (const char *)&message, message.messageDataSize);
}
