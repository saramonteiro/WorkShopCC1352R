/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty.c ========
 */

/* For usleep() */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>

/* Drivers and macros  from SC */
#include "scif.h"
#define BV(x)    (1 << (x))

/* Board Header file */
#include "Board.h"

/* Funcoes para processar o sinal de alerta do SC */
void processTaskAlert(void)
{
    // Limpa a flag de interrupcao fonte da interrupcao
    scifClearAlertIntSource();

    // Atribui o valor de FlagHIGH do SC para a variavel high
    uint8_t high = scifTaskData.triggerLed.state.FlagHIGH;
    // Atualiza o estado do led vermelho de acordo com a flag
    GPIO_write(Board_GPIO_RLED, high);

    // Confirma o evento ao framework
    scifAckAlertEvents();
}

/* Callback Functions */
void scCtrlReadyCallback(void)
{

} // scCtrlReadyCallback

void scTaskAlertCallback(void)
{
    processTaskAlert();
} // scTaskAlertCallback


/*
 *  ======== mainThread ========
 */
void *ThresholdScThread(void *arg0)
{
    // Inicializa o Sensor Controller
    //Inicializa a OSAL (Operating System Abstraction Layer) do framework scif
    scifOsalInit();
    //Registra as duas funções de callback para os sinais de Interrupcao Control READY e Task ALERT do SC
    scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
    scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
    //Inicializa o SC com o driver gerado a partir do SC
    scifInit(&scifDriverSetup);

    // Configura o intervalo para a tarefa do Sensor Controller para 1 segundo
    //Bits 31:16 = segundos
    //Bits 15:0 = 1/65536 de um segundo
    uint32_t rtc_Hz = 1;  // 1Hz RTC
    scifStartRtcTicksNow(0x00010000 / rtc_Hz);

    // Configura a tarefa do Sensor Controller: Inicializa um valor de threshold (800)
    scifTaskData.triggerLed.cfg.threshold = 600;

    // Inicializa a tarefa do Sensor Controller
    // As funções scif lidam com IDs de tarefas do SC e acessam um vetor de bit como argumento
    //O ID está definido em scif.h
    scifStartTasksNbl(BV(SCIF_TRIGGER_LED_TASK_ID));

    /* 1 second delay */
    uint32_t time = 1;

    /* Call driver init functions */
    GPIO_init();
    // I2C_init();
    // SDSPI_init();
    // SPI_init();
    // UART_init();
    // Watchdog_init();

    /* Configure the LED pin */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_OFF);

    while (1)
    {

    }
}
