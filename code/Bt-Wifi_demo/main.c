/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC6 Application
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#include "cycfg.h"
#include "cy_device_headers.h"

#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <task.h>

#include <stdio.h>

#include "cli_controller.h"


#define IO_STDOUT_ENABLE
#define IO_STDIN_ENABLE     
#define IO_STDOUT_UART      Bridge_UART_HW
#define IO_STDIN_UART       Bridge_UART_HW

/* Task defs */
#define LED_TASK_STACK      configMINIMAL_STACK_SIZE
#define CLI_TASK_STACK      configMINIMAL_STACK_SIZE

#define LED_TASK_PRIORITY   (1u) 
#define CLI_TASK_PRIORITY   (1u) 

#define _1S                 (1000UL)
#define ONE_TICK            (1u)


/* Assign divider type and number for UART */
#define UART_CLK_DIV_TYPE     (CY_SYSCLK_DIV_8_BIT)
#define UART_CLK_DIV_NUMBER   (0U)

#define AUTO_GEN              (1u)


void led_task(void* pvPort);
void CLI_task(void* pvPort);


int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable interrupts */
    __enable_irq();
    
    init_cycfg_all();
 
    UBaseType_t ret;

    /* Creating LED task */
    ret = xTaskCreate(led_task, "Led task", LED_TASK_STACK, NULL, LED_TASK_PRIORITY, NULL);

    if(ret == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) { CY_ASSERT(0); }

    /* Creating CLI task */
    ret = xTaskCreate(CLI_task, "CLI task", CLI_TASK_STACK, NULL, CLI_TASK_PRIORITY, NULL);

    if(ret == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) { CY_ASSERT(0); }

    /* Start the kernel */
    vTaskStartScheduler();

    /* Exiting Kernel. The unit must be haltred */
    CY_ASSERT(0);

    for (;;)
    {
        
    }
}


/**
 * @brief Process command line interface
 * 
 * @param pvPort 
 */
void CLI_task(void *pvPort)
{
    CLI_start();

    for(;;)
    {
        CLI_process_commands();

        /* Suspend task for one tick */
        vTaskDelay(ONE_TICK);
    }
}


/**
 * @brief Toggle LED task for physical signaling
 * 
 * @param pvPort 
 */
void led_task(void* pvPort)
{
    /* Initialize GPIO */
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    for(;;)
    {
        cyhal_gpio_toggle(CYBSP_USER_LED);
        
        /* Wait 1 second to toggle the LED */
        vTaskDelay(_1S);
    }
}

/* [] END OF FILE */