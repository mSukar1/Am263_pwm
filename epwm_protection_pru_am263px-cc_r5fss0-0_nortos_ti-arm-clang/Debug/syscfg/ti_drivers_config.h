/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Auto generated file
 */

#ifndef TI_DRIVERS_CONFIG_H_
#define TI_DRIVERS_CONFIG_H_

#include <stdint.h>
#include <drivers/hw_include/cslr_soc.h>
#include "ti_dpl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Common Functions
 */
void System_init(void);
void System_deinit(void);

/*
 * MCSPI
 */
#include <drivers/mcspi.h>

/* MCSPI Instance Macros */
#define CONFIG_MCSPI0 (0U)
#define CONFIG_MCSPI_NUM_INSTANCES (1U)
#define CONFIG_MCSPI_NUM_DMA_INSTANCES (0U)

/*
 * EDMA
 */
#include <drivers/edma.h>
#include <drivers/soc.h>

/* EDMA Instance Macros */
#define CONFIG_EDMA0_BASE_ADDR (CSL_TPCC0_U_BASE)
#define CONFIG_EDMA0 (0U)
#define CONFIG_EDMA_NUM_INSTANCES (1U)
/*
 * EPWM
 */
#include <drivers/epwm.h>
#include <drivers/soc.h>

/* EPWM Instance Macros */
#define CONFIG_PWM0_BASE_ADDR (CSL_CONTROLSS_G0_EPWM0_U_BASE)
#define CONFIG_PWM1_BASE_ADDR (CSL_CONTROLSS_G0_EPWM1_U_BASE)
#define CONFIG_PWM2_BASE_ADDR (CSL_CONTROLSS_G0_EPWM2_U_BASE)
#define CONFIG_EPWM_NUM_INSTANCES (3U)

/*
 * GPIO
 */
#include <drivers/gpio.h>
#include <kernel/dpl/AddrTranslateP.h>

/* GPIO PIN Macros */
#define GPIO_IGBT_FAULT_IN_BASE_ADDR (CSL_GPIO0_U_BASE)
#define GPIO_IGBT_FAULT_IN_PIN (51)
#define GPIO_IGBT_FAULT_IN_DIR (GPIO_DIRECTION_INPUT)
#define GPIO_IGBT_FAULT_IN_TRIG_TYPE (GPIO_TRIG_TYPE_NONE)
#define GPIO_OVP_IN_BASE_ADDR (CSL_GPIO0_U_BASE)
#define GPIO_OVP_IN_PIN (53)
#define GPIO_OVP_IN_DIR (GPIO_DIRECTION_INPUT)
#define GPIO_OVP_IN_TRIG_TYPE (GPIO_TRIG_TYPE_NONE)
#define GPIO_IGBT_FAULT_OUT_BASE_ADDR (CSL_GPIO0_U_BASE)
#define GPIO_IGBT_FAULT_OUT_PIN (55)
#define GPIO_IGBT_FAULT_OUT_DIR (GPIO_DIRECTION_OUTPUT)
#define GPIO_IGBT_FAULT_OUT_TRIG_TYPE (GPIO_TRIG_TYPE_NONE)
#define GPIO_OVP_OUT_BASE_ADDR (CSL_GPIO0_U_BASE)
#define GPIO_OVP_OUT_PIN (57)
#define GPIO_OVP_OUT_DIR (GPIO_DIRECTION_OUTPUT)
#define GPIO_OVP_OUT_TRIG_TYPE (GPIO_TRIG_TYPE_NONE)
#define GPIO_OCP_IN_BASE_ADDR (CSL_GPIO0_U_BASE)
#define GPIO_OCP_IN_PIN (52)
#define GPIO_OCP_IN_DIR (GPIO_DIRECTION_INPUT)
#define GPIO_OCP_IN_TRIG_TYPE (GPIO_TRIG_TYPE_NONE)
#define GPIO_SPEED_LEVEL_IN_BASE_ADDR (CSL_GPIO0_U_BASE)
#define GPIO_SPEED_LEVEL_IN_PIN (54)
#define GPIO_SPEED_LEVEL_IN_DIR (GPIO_DIRECTION_INPUT)
#define GPIO_SPEED_LEVEL_IN_TRIG_TYPE (GPIO_TRIG_TYPE_NONE)
#define GPIO_OCP_OUT_BASE_ADDR (CSL_GPIO0_U_BASE)
#define GPIO_OCP_OUT_PIN (56)
#define GPIO_OCP_OUT_DIR (GPIO_DIRECTION_OUTPUT)
#define GPIO_OCP_OUT_TRIG_TYPE (GPIO_TRIG_TYPE_NONE)
#define GPIO_SPEED_LEVEL_OUT_BASE_ADDR (CSL_GPIO0_U_BASE)
#define GPIO_SPEED_LEVEL_OUT_PIN (58)
#define GPIO_SPEED_LEVEL_OUT_DIR (GPIO_DIRECTION_OUTPUT)
#define GPIO_SPEED_LEVEL_OUT_TRIG_TYPE (GPIO_TRIG_TYPE_NONE)
#define CONFIG_GPIO_NUM_INSTANCES (8U)




/*
 * UART
 */
#include <drivers/uart.h>

/* UART Instance Macros */
#define CONFIG_UART0 (0U)
#define CONFIG_UART_NUM_INSTANCES (1U)
#define CONFIG_UART_NUM_DMA_INSTANCES (0U)

/* Macros for configuring ePWM Xbar */
#define EPWM_XBAR_0  (EPWM_XBAR_INPUT_XBAR0)
#define EPWM_XBAR_1  (EPWM_XBAR_INPUT_XBAR1)
#define EPWM_XBAR_2  (EPWM_XBAR_INPUT_XBAR2)
#define EPWM_XBAR_3  (EPWM_XBAR_INPUT_XBAR3)
#define EPWM_XBAR_4  (EPWM_XBAR_INPUT_XBAR4)
#define EPWM_XBAR_5  (EPWM_XBAR_INPUT_XBAR5)
#define EPWM_XBAR_6  (EPWM_XBAR_INPUT_XBAR6)
#define EPWM_XBAR_7  (EPWM_XBAR_INPUT_XBAR7)


#include <drivers/soc.h>
#include <kernel/dpl/CycleCounterP.h>


#ifdef __cplusplus
}
#endif

#endif /* TI_DRIVERS_CONFIG_H_ */
