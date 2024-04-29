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

#include "ti_drivers_config.h"

/*
 * MCSPI
 */
#include "ti_drivers_open_close.h"

uint32_t gMcspiNumCh[1] =
{
    CONFIG_MCSPI0_NUM_CH,
};

/* MCSPI atrributes */
static MCSPI_Attrs gMcspiAttrs[CONFIG_MCSPI_NUM_INSTANCES] =
{
    {
        .baseAddr           = CSL_MCSPI1_U_BASE,
        .inputClkFreq       = 50000000U,
        .intrNum            = CSLR_R5FSS0_CORE0_INTR_MCSPI1_INTR,
        .operMode           = MCSPI_OPER_MODE_INTERRUPT,
        .intrPriority       = 4U,
        .chMode             = MCSPI_CH_MODE_SINGLE,
        .pinMode            = MCSPI_PINMODE_4PIN,
        .initDelay          = MCSPI_INITDLY_0,
        .multiWordAccess    = FALSE,
    },
};
/* MCSPI objects - initialized by the driver */
static MCSPI_Object gMcspiObjects[CONFIG_MCSPI_NUM_INSTANCES];
/* MCSPI driver configuration */
MCSPI_Config gMcspiConfig[CONFIG_MCSPI_NUM_INSTANCES] =
{
    {
        &gMcspiAttrs[CONFIG_MCSPI0],
        &gMcspiObjects[CONFIG_MCSPI0],
    },
};

uint32_t gMcspiConfigNum = CONFIG_MCSPI_NUM_INSTANCES;

#include <drivers/edma.h>
#include <drivers/mcspi/v0/lld/dma/mcspi_dma.h>
#include <drivers/mcspi/v0/lld/dma/edma/mcspi_dma_edma.h>
MCSPI_DmaConfig gMcspiDmaConfig =
{
    .fxns        = NULL,
    .mcspiDmaArgs = (void *)NULL,
};

McspiDma_EdmaArgs gMcspiEdmaArgs =
{
    .drvHandle        = NULL,
};

MCSPI_DmaHandle gMcspiDmaHandle[] =
{
};

uint32_t gMcspiDmaConfigNum = CONFIG_MCSPI_NUM_DMA_INSTANCES;

/*
 * EDMA
 */
/* EDMA atrributes */
static EDMA_Attrs gEdmaAttrs[CONFIG_EDMA_NUM_INSTANCES] =
{
    {

        .baseAddr           = CSL_TPCC0_U_BASE,
        .compIntrNumber     = CSLR_R5FSS0_CORE0_INTR_TPCC0_INT_0,
        .intrAggEnableAddr  = CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_TPCC0_INTAGG_MASK,
        .intrAggEnableMask  = 0x1FF & (~(2U << 0)),
        .intrAggStatusAddr  = CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_TPCC0_INTAGG_STATUS,
        .intrAggClearMask   = (2U << 0),
        .initPrms           =
        {
            .regionId     = 0,
            .queNum       = 0,
            .initParamSet = FALSE,
            .ownResource    =
            {
                .qdmaCh      = 0x03U,
                .dmaCh[0]    = 0xFFFFFFFFU,
                .dmaCh[1]    = 0x000000FFU,
                .tcc[0]      = 0xFFFFFFFFU,
                .tcc[1]      = 0x000000FFU,
                .paramSet[0] = 0xFFFFFFFFU,
                .paramSet[1] = 0xFFFFFFFFU,
                .paramSet[2] = 0xFFFFFFFFU,
                .paramSet[3] = 0xFFFFFFFFU,
                .paramSet[4] = 0xFFFFFFFFU,
                .paramSet[5] = 0xFFFFFFFFU,
                .paramSet[6] = 0xFFFFFFFFU,
                .paramSet[7] = 0x000007FFU,
            },
            .reservedDmaCh[0]    = 0x00000000U,
            .reservedDmaCh[1]    = 0x00000000U,
        },
    },
};

/* EDMA objects - initialized by the driver */
static EDMA_Object gEdmaObjects[CONFIG_EDMA_NUM_INSTANCES];
/* EDMA driver configuration */
EDMA_Config gEdmaConfig[CONFIG_EDMA_NUM_INSTANCES] =
{
    {
        &gEdmaAttrs[CONFIG_EDMA0],
        &gEdmaObjects[CONFIG_EDMA0],
    },
};

uint32_t gEdmaConfigNum = CONFIG_EDMA_NUM_INSTANCES;

/*
 * GPIO
 */

/* ----------- GPIO Direction, Trigger, Interrupt initialization ----------- */

void GPIO_init()
{
    uint32_t    baseAddr;

    /* Instance 0 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_IGBT_FAULT_IN_BASE_ADDR);
    GPIO_setDirMode(baseAddr, GPIO_IGBT_FAULT_IN_PIN, GPIO_IGBT_FAULT_IN_DIR);

    /* Instance 1 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_OVP_IN_BASE_ADDR);
    GPIO_setDirMode(baseAddr, GPIO_OVP_IN_PIN, GPIO_OVP_IN_DIR);

    /* Instance 2 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_IGBT_FAULT_OUT_BASE_ADDR);
    GPIO_setDirMode(baseAddr, GPIO_IGBT_FAULT_OUT_PIN, GPIO_IGBT_FAULT_OUT_DIR);

    /* Instance 3 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_OVP_OUT_BASE_ADDR);
    GPIO_setDirMode(baseAddr, GPIO_OVP_OUT_PIN, GPIO_OVP_OUT_DIR);

    /* Instance 4 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_OCP_IN_BASE_ADDR);
    GPIO_setDirMode(baseAddr, GPIO_OCP_IN_PIN, GPIO_OCP_IN_DIR);

    /* Instance 5 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_SPEED_LEVEL_IN_BASE_ADDR);
    GPIO_setDirMode(baseAddr, GPIO_SPEED_LEVEL_IN_PIN, GPIO_SPEED_LEVEL_IN_DIR);

    /* Instance 6 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_OCP_OUT_BASE_ADDR);
    GPIO_setDirMode(baseAddr, GPIO_OCP_OUT_PIN, GPIO_OCP_OUT_DIR);

    /* Instance 7 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_SPEED_LEVEL_OUT_BASE_ADDR);
    GPIO_setDirMode(baseAddr, GPIO_SPEED_LEVEL_OUT_PIN, GPIO_SPEED_LEVEL_OUT_DIR);
}


/* ----------- GPIO Interrupt de-initialization ----------- */
void GPIO_deinit()
{

}

/*
 * UART
 */

/* UART atrributes */
static UART_Attrs gUartAttrs[CONFIG_UART_NUM_INSTANCES] =
{
    {
        .baseAddr           = CSL_UART0_U_BASE,
        .inputClkFreq       = 48000000U,
    },
};
/* UART objects - initialized by the driver */
static UART_Object gUartObjects[CONFIG_UART_NUM_INSTANCES];
/* UART driver configuration */
UART_Config gUartConfig[CONFIG_UART_NUM_INSTANCES] =
{
    {
        &gUartAttrs[CONFIG_UART0],
        &gUartObjects[CONFIG_UART0],
    },
};

uint32_t gUartConfigNum = CONFIG_UART_NUM_INSTANCES;

#include <drivers/uart/v0/dma/uart_dma.h>


UART_DmaConfig gUartDmaConfig[CONFIG_UART_NUM_DMA_INSTANCES] =
{
};

uint32_t gUartDmaConfigNum = CONFIG_UART_NUM_DMA_INSTANCES;


void Drivers_uartInit(void)
{
    UART_init();
}


void Pinmux_init(void);
void PowerClock_init(void);
void PowerClock_deinit(void);

/*
 * Common Functions
 */
void System_init(void)
{
    /* DPL init sets up address transalation unit, on some CPUs this is needed
     * to access SCICLIENT services, hence this needs to happen first
     */
    Dpl_init();

    
    /* initialize PMU */
    CycleCounterP_init(SOC_getSelfCpuClk());


    PowerClock_init();
    /* Now we can do pinmux */
    Pinmux_init();
    /* finally we initialize all peripheral drivers */
    MCSPI_init();
    EDMA_init();
    /* EPWM */
    {
        /* Enable time base clock for the selected ePWM */
        SOC_setEpwmTbClk(0, TRUE);
        SOC_setEpwmGroup(0, 0);
        SOC_setEpwmTbClk(1, TRUE);
        SOC_setEpwmGroup(1, 0);
        Soc_enableEPWMHalt(1);
        SOC_setEpwmTbClk(2, TRUE);
        SOC_setEpwmGroup(2, 0);
        Soc_enableEPWMHalt(2);
    }
    GPIO_init();
    Drivers_uartInit();
}

void System_deinit(void)
{
    MCSPI_deinit();
    EDMA_deinit();
    /* EPWM */
    {
        /* Disable time base clock for the selected ePWM */
        SOC_setEpwmTbClk(0, FALSE);
	
        SOC_setEpwmTbClk(1, FALSE);
	
        SOC_setEpwmTbClk(2, FALSE);
	
    }
    GPIO_deinit();
    UART_deinit();
    PowerClock_deinit();
    Dpl_deinit();
}
