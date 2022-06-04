#include <cy_syslib.h>

#include "cycfg.h"
#include "cy_device_headers.h"
#include "cy_scb_uart.h"

/* Standard includes. */
#include <string.h>
#include <stdint.h>
#include <stdio.h>

/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include <task.h>

#include "FreeRTOS-Plus-CLI/FreeRTOS_CLI.h"

#define SINGLE_TICK  (1u)

/* 100-byte wide input buffer lenght */
#define INPUT_STRING_LEN    (100u)
#define OUTPUT_STRING_LEN   (100u)

#define TERMINAL_WRITE(string) Cy_SCB_UART_PutString(UART_Debug_HW, string)
#define ECHO_CHAR(char)        SCB_TX_FIFO_WR(UART_Debug_HW) = char;

#define LED_CMD    "led"

#define ENTER_CHAR '\r'
#define DEL_CHAR   (127)
#define NULL_CHAR  '\0'


typedef enum
{
    ENTER_RECEIVED,
    DEL_RECEIVED,
    UP_ARROW
} enum_cmd_type_t;

static enum_cmd_type_t cmd_type;
static char output_string[OUTPUT_STRING_LEN];
static char input_buffer [INPUT_STRING_LEN];
static char input_string [INPUT_STRING_LEN];
static int8 char_index;
static uint8 data_ok;



static cy_stc_scb_uart_context_t uart_context;


static BaseType_t xLedConfig( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );


/**
 * @brief CLI description:
 * command string: led
 * Number of parameter: 1 (Blink rate in seconds)
 * 
 */
static const CLI_Command_Definition_t CLI_led_cmd = 
{
    LED_CMD,
    "led <Blink rate in seconds>",
    xLedConfig,
    1
};

/**
 * @brief UART RX handler ISR callback function
 * 
 */
void rx_handler(void) {
    /* Read the character */
    input_buffer[char_index] = SCB_RX_FIFO_RD(UART_Debug_HW);

    /* Echo data to the terminal */
    if(char_index >= 0) { ECHO_CHAR(input_buffer[char_index]) };

    /* If an 'ENTER' character has been detected, set the 'data ok' flag to TRUE to begin processings */
    if(input_buffer[char_index] == ENTER_CHAR) 
    {
        cmd_type = ENTER_RECEIVED; 
        data_ok  = pdTRUE;
    }
    else if(input_buffer[char_index] == DEL_CHAR)
    {
        cmd_type = DEL_RECEIVED;
        data_ok  = pdTRUE;
    }
    
    /* Increase the character index on every iteration */
    char_index++;
    
    /* Reset the index and if it exceeds the maximum input string length */
    if(char_index >= INPUT_STRING_LEN) { char_index = 0; }

    /* Clear the RX interrupt */
    Cy_SCB_ClearRxInterrupt(UART_Debug_HW, CY_SCB_RX_INTR_NOT_EMPTY);
}


/**
 * @brief Command to control LED blink rate
 * 
 * @param pcWriteBuffer Buffer to print return string
 * @param xWriteBufferLen  Length of the write buffer
 * @param pcCommandString  Command string
 * @return BaseType_t      Command completion status
 */
static BaseType_t xLedConfig( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
    sprintf(pcWriteBuffer, "LED command received\r\n");

    /* Retrieve the entered parameter */
    return pdFALSE;
}


/**
 * @brief Configure the UART SCB
 * 
 */
static void vConfigUart(void) {
    /* Initializing UART component */
    Cy_SCB_UART_Init(UART_Debug_HW, &UART_Debug_config, &uart_context);
    Cy_SCB_UART_Enable(UART_Debug_HW);

    /* Initialize component interrupt */
    cy_stc_sysint_t uart_debug_int =
    {
        .intrSrc      = scb_5_interrupt_IRQn,
        .intrPriority = 1u,
    };

    Cy_SysInt_Init(&uart_debug_int, &rx_handler);
    NVIC_EnableIRQ(scb_5_interrupt_IRQn);
}


/**
 * @brief UNpack the data from the input buffer into a readable command
 * 
 * @param input_buff Pointer to the input buffer
 */
static void vUnpackData(char* input_buff, char* command_string) {
    switch (cmd_type) {
        case ENTER_RECEIVED:
            /* Replace the enter character with a NULL terminator */
            *(input_buff + (char_index - 1)) = NULL_CHAR;

            strcpy(command_string, input_buff);
            break;

        case DEL_RECEIVED:
            *(input_buff + (char_index - 1)) = NULL_CHAR;
            *(input_buff + (char_index - 2)) = NULL_CHAR;

            /* Decrease the character index by two characters, the delete character and the character to delete */
            char_index -= 2;
            break;
        
        default:
            break;
    }
}


/**
 * @brief Initialization function for command line inteface
 * 
 */
void CLI_start(void) {
    vConfigUart();

    /* Introduction string */
    TERMINAL_WRITE("\r\n");
    TERMINAL_WRITE("Hello world!\r\n");
    TERMINAL_WRITE("Welcome to the Command Line Interface Demo\r\n");
    TERMINAL_WRITE("> ");

    /* Enable UART interrupts. A callback must be provided as an interrupt handle to be executed
     * upon byte reception */
    FreeRTOS_CLIRegisterCommand(&CLI_led_cmd);
}


/**
 * @brief Process the commands once received
 * 
 */
void CLI_process_commands(void) {
    BaseType_t data_incoming;

    /* Hang here while no command has been received */
    while(!data_ok) { vTaskDelay(SINGLE_TICK); }
    
    vUnpackData(input_buffer, input_string);

    /* If the character entered is not ENTER, then return */
    if(cmd_type != ENTER_RECEIVED) { 
        data_ok = pdFALSE;
        return;
    }
    
    /* Move to the next line on the terminal once the command is received */
    TERMINAL_WRITE("\r\n");
    
    /* Process the command and echo to the terminal while there are lines to print */
    do {
        data_incoming = FreeRTOS_CLIProcessCommand(input_string, output_string, sizeof(output_string));

        /* Write the resultant string into the terminal */
        TERMINAL_WRITE(output_string);
    } while(data_incoming == pdTRUE);
    
    /* Reset the character index */
    char_index = 0;

    /* Clear the input buffers */
    memset(input_buffer, 0, sizeof(input_buffer));
    memset(input_string, 0, sizeof(input_string));

    /* Clear the output buffer */
    memset(output_string, 0, sizeof(output_string));

    /* Reset the 'data ok' flag */
    data_ok = pdFALSE;

    TERMINAL_WRITE("\r\n");
    TERMINAL_WRITE("> ");
}
