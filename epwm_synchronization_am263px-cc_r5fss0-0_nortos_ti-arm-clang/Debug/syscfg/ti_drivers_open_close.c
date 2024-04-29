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

#include "ti_drivers_open_close.h"
#include <kernel/dpl/DebugP.h>

void Drivers_open(void)
{

    Drivers_pmicOpen();
    Drivers_edmaOpen();
    Drivers_mcspiOpen();
    Drivers_ecapOpen();
    Drivers_epwmOpen();
    Drivers_inputXbarOpen();
    Drivers_uartOpen();
}

void Drivers_close(void)
{
    Drivers_mcspiClose();
    Drivers_uartClose();
    Drivers_edmaClose();
}

/*
 * MCSPI
 */
/* MCSPI Driver handles */
MCSPI_Handle gMcspiHandle[CONFIG_MCSPI_NUM_INSTANCES];
/* MCSPI Driver Open Parameters */
MCSPI_OpenParams gMcspiOpenParams[CONFIG_MCSPI_NUM_INSTANCES] =
{
    {
        .transferMode           = MCSPI_TRANSFER_MODE_BLOCKING,
        .transferTimeout        = SystemP_WAIT_FOREVER,
        .transferCallbackFxn    = NULL,
        .msMode                 = MCSPI_MS_MODE_CONTROLLER,
        .mcspiDmaIndex = -1,
    },
};
/* MCSPI Driver Channel Configurations */
MCSPI_ChConfig gConfigMcspi0ChCfg[CONFIG_MCSPI0_NUM_CH] =
{
    {
        .chNum              = MCSPI_CHANNEL_0,
        .frameFormat        = MCSPI_FF_POL0_PHA0,
        .bitRate            = 1000000,
        .csPolarity         = MCSPI_CS_POL_LOW,
        .trMode             = MCSPI_TR_MODE_TX_RX,
        .inputSelect        = MCSPI_IS_D1,
        .dpe0               = MCSPI_DPE_ENABLE,
        .dpe1               = MCSPI_DPE_DISABLE,
        .slvCsSelect        = MCSPI_SLV_CS_SELECT_0,
        .startBitEnable     = FALSE,
        .startBitPolarity   = MCSPI_SB_POL_LOW,
        .csIdleTime         = MCSPI_TCS0_0_CLK,
        .turboEnable        = FALSE,
        .defaultTxData      = 0x0U,
        .txFifoTrigLvl      = 16U,
        .rxFifoTrigLvl      = 16U,
    },
};

MCSPI_ChConfig *gConfigMcspiChCfg[1] =
{
    gConfigMcspi0ChCfg,
};



MCSPI_DmaChConfig gMcspiDmaChConfig[1] =
{
    NULL,
};

void Drivers_mcspiOpen(void)
{
    uint32_t instCnt;
    int32_t  status = SystemP_SUCCESS;

    for(instCnt = 0U; instCnt < CONFIG_MCSPI_NUM_INSTANCES; instCnt++)
    {
        gMcspiHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
    }

    /* Open all instances */
    for(instCnt = 0U; instCnt < CONFIG_MCSPI_NUM_INSTANCES; instCnt++)
    {
        gMcspiHandle[instCnt] = MCSPI_open(instCnt, &gMcspiOpenParams[instCnt]);
        if(NULL == gMcspiHandle[instCnt])
        {
            DebugP_logError("MCSPI open failed for instance %d !!!\r\n", instCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    if(SystemP_FAILURE == status)
    {
        Drivers_mcspiClose();   /* Exit gracefully */
    }

    return;
}

void Drivers_mcspiClose(void)
{
    uint32_t instCnt;
    /* Close all instances that are open */
    for(instCnt = 0U; instCnt < CONFIG_MCSPI_NUM_INSTANCES; instCnt++)
    {
        if(gMcspiHandle[instCnt] != NULL)
        {
            MCSPI_close(gMcspiHandle[instCnt]);
            gMcspiHandle[instCnt] = NULL;
        }
    }
    return;
}

/*
 * EDMA
 */
/* EDMA Driver handles */
EDMA_Handle gEdmaHandle[CONFIG_EDMA_NUM_INSTANCES];

/* EDMA Driver Open Parameters */
EDMA_Params gEdmaParams[CONFIG_EDMA_NUM_INSTANCES] =
{
    {
        .intrEnable = TRUE,
    },
};

void Drivers_edmaOpen(void)
{
    uint32_t instCnt;
    int32_t  status = SystemP_SUCCESS;

    for(instCnt = 0U; instCnt < CONFIG_EDMA_NUM_INSTANCES; instCnt++)
    {
        gEdmaHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
    }

    /* Open all instances */
    for(instCnt = 0U; instCnt < CONFIG_EDMA_NUM_INSTANCES; instCnt++)
    {
        gEdmaHandle[instCnt] = EDMA_open(instCnt, &gEdmaParams[instCnt]);
        if(NULL == gEdmaHandle[instCnt])
        {
            DebugP_logError("EDMA open failed for instance %d !!!\r\n", instCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    if(SystemP_FAILURE == status)
    {
        Drivers_edmaClose();   /* Exit gracefully */
    }

    return;
}

void Drivers_edmaClose(void)
{
    uint32_t instCnt;

    /* Close all instances that are open */
    for(instCnt = 0U; instCnt < CONFIG_EDMA_NUM_INSTANCES; instCnt++)
    {
        if(gEdmaHandle[instCnt] != NULL)
        {
            EDMA_close(gEdmaHandle[instCnt]);
            gEdmaHandle[instCnt] = NULL;
        }
    }

    return;
}

void Drivers_ecapOpen()
{
	/* Disables time stamp capture */
	ECAP_disableTimeStampCapture(REF_CAPTURE_BASE_ADDR);
	/* Stops Time stamp counter */
	ECAP_stopCounter(REF_CAPTURE_BASE_ADDR);
	/* Sets eCAP in Capture mode. */
	ECAP_enableCaptureMode(REF_CAPTURE_BASE_ADDR);
	/* Sets the capture mode */
	ECAP_setCaptureMode(REF_CAPTURE_BASE_ADDR,ECAP_CONTINUOUS_CAPTURE_MODE,ECAP_EVENT_1);
	/* Sets the Capture event polarity */
	ECAP_setEventPrescaler(REF_CAPTURE_BASE_ADDR, 0);
	/* Sets the Capture event polarity */
	ECAP_setEventPolarity(REF_CAPTURE_BASE_ADDR,ECAP_EVENT_1,ECAP_EVNT_FALLING_EDGE);
	ECAP_setEventPolarity(REF_CAPTURE_BASE_ADDR,ECAP_EVENT_2,ECAP_EVNT_RISING_EDGE);
	ECAP_setEventPolarity(REF_CAPTURE_BASE_ADDR,ECAP_EVENT_3,ECAP_EVNT_RISING_EDGE);
	ECAP_setEventPolarity(REF_CAPTURE_BASE_ADDR,ECAP_EVENT_4,ECAP_EVNT_RISING_EDGE);
	/* Configure counter reset on events */
	ECAP_disableCounterResetOnEvent(REF_CAPTURE_BASE_ADDR,ECAP_EVENT_1);
	ECAP_disableCounterResetOnEvent(REF_CAPTURE_BASE_ADDR,ECAP_EVENT_2);
	ECAP_disableCounterResetOnEvent(REF_CAPTURE_BASE_ADDR,ECAP_EVENT_3);
	ECAP_disableCounterResetOnEvent(REF_CAPTURE_BASE_ADDR,ECAP_EVENT_4);
	/* Select eCAP input */
    /* Select qualification period to filter out noise*/
	ECAP_selectECAPInput(REF_CAPTURE_BASE_ADDR,ECAP_INPUT_INPUTXBAR0);
    ECAP_selectQualPeriod(REF_CAPTURE_BASE_ADDR,ECAP_PULSE_WIDTH_FILTER_BYPASS);
	/* Sets a phase shift value count */
	ECAP_setPhaseShiftCount(REF_CAPTURE_BASE_ADDR,0);
	/* Enable counter loading with phase shift value */
	ECAP_enableLoadCounter(REF_CAPTURE_BASE_ADDR);
	/* Load time stamp counter */
	ECAP_loadCounter(REF_CAPTURE_BASE_ADDR);
	/* Configures Sync out signal mode */
	ECAP_setSyncOutMode(REF_CAPTURE_BASE_ADDR,ECAP_SYNC_OUT_DISABLED);

	/* Configures emulation mode */
	ECAP_setEmulationMode(REF_CAPTURE_BASE_ADDR,ECAP_EMULATION_STOP);
	/* Set up the source for sync-in pulse */
	ECAP_setSyncInPulseSource(REF_CAPTURE_BASE_ADDR,ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0);

	/* Disables time stamp capture */
	ECAP_disableTimeStampCapture(ECAP1_CAPTURE_BASE_ADDR);
	/* Stops Time stamp counter */
	ECAP_stopCounter(ECAP1_CAPTURE_BASE_ADDR);
	/* Sets eCAP in Capture mode. */
	ECAP_enableCaptureMode(ECAP1_CAPTURE_BASE_ADDR);
	/* Sets the capture mode */
	ECAP_setCaptureMode(ECAP1_CAPTURE_BASE_ADDR,ECAP_CONTINUOUS_CAPTURE_MODE,ECAP_EVENT_1);
	/* Sets the Capture event polarity */
	ECAP_setEventPrescaler(ECAP1_CAPTURE_BASE_ADDR, 0);
	/* Sets the Capture event polarity */
	ECAP_setEventPolarity(ECAP1_CAPTURE_BASE_ADDR,ECAP_EVENT_1,ECAP_EVNT_FALLING_EDGE);
	ECAP_setEventPolarity(ECAP1_CAPTURE_BASE_ADDR,ECAP_EVENT_2,ECAP_EVNT_RISING_EDGE);
	ECAP_setEventPolarity(ECAP1_CAPTURE_BASE_ADDR,ECAP_EVENT_3,ECAP_EVNT_RISING_EDGE);
	ECAP_setEventPolarity(ECAP1_CAPTURE_BASE_ADDR,ECAP_EVENT_4,ECAP_EVNT_RISING_EDGE);
	/* Configure counter reset on events */
	ECAP_disableCounterResetOnEvent(ECAP1_CAPTURE_BASE_ADDR,ECAP_EVENT_1);
	ECAP_disableCounterResetOnEvent(ECAP1_CAPTURE_BASE_ADDR,ECAP_EVENT_2);
	ECAP_disableCounterResetOnEvent(ECAP1_CAPTURE_BASE_ADDR,ECAP_EVENT_3);
	ECAP_disableCounterResetOnEvent(ECAP1_CAPTURE_BASE_ADDR,ECAP_EVENT_4);
	/* Select eCAP input */
    /* Select qualification period to filter out noise*/
	ECAP_selectECAPInput(ECAP1_CAPTURE_BASE_ADDR,ECAP_INPUT_INPUTXBAR1);
    ECAP_selectQualPeriod(ECAP1_CAPTURE_BASE_ADDR,ECAP_PULSE_WIDTH_FILTER_BYPASS);
	/* Sets a phase shift value count */
	ECAP_setPhaseShiftCount(ECAP1_CAPTURE_BASE_ADDR,0);
	/* Enable counter loading with phase shift value */
	ECAP_enableLoadCounter(ECAP1_CAPTURE_BASE_ADDR);
	/* Load time stamp counter */
	ECAP_loadCounter(ECAP1_CAPTURE_BASE_ADDR);
	/* Configures Sync out signal mode */
	ECAP_setSyncOutMode(ECAP1_CAPTURE_BASE_ADDR,ECAP_SYNC_OUT_DISABLED);

	/* Configures emulation mode */
	ECAP_setEmulationMode(ECAP1_CAPTURE_BASE_ADDR,ECAP_EMULATION_STOP);
	/* Set up the source for sync-in pulse */
	ECAP_setSyncInPulseSource(ECAP1_CAPTURE_BASE_ADDR,ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0);

	/* Disables time stamp capture */
	ECAP_disableTimeStampCapture(ECAP2_CAPTURE_BASE_ADDR);
	/* Stops Time stamp counter */
	ECAP_stopCounter(ECAP2_CAPTURE_BASE_ADDR);
	/* Sets eCAP in Capture mode. */
	ECAP_enableCaptureMode(ECAP2_CAPTURE_BASE_ADDR);
	/* Sets the capture mode */
	ECAP_setCaptureMode(ECAP2_CAPTURE_BASE_ADDR,ECAP_CONTINUOUS_CAPTURE_MODE,ECAP_EVENT_1);
	/* Sets the Capture event polarity */
	ECAP_setEventPrescaler(ECAP2_CAPTURE_BASE_ADDR, 0);
	/* Sets the Capture event polarity */
	ECAP_setEventPolarity(ECAP2_CAPTURE_BASE_ADDR,ECAP_EVENT_1,ECAP_EVNT_FALLING_EDGE);
	ECAP_setEventPolarity(ECAP2_CAPTURE_BASE_ADDR,ECAP_EVENT_2,ECAP_EVNT_RISING_EDGE);
	ECAP_setEventPolarity(ECAP2_CAPTURE_BASE_ADDR,ECAP_EVENT_3,ECAP_EVNT_RISING_EDGE);
	ECAP_setEventPolarity(ECAP2_CAPTURE_BASE_ADDR,ECAP_EVENT_4,ECAP_EVNT_RISING_EDGE);
	/* Configure counter reset on events */
	ECAP_disableCounterResetOnEvent(ECAP2_CAPTURE_BASE_ADDR,ECAP_EVENT_1);
	ECAP_disableCounterResetOnEvent(ECAP2_CAPTURE_BASE_ADDR,ECAP_EVENT_2);
	ECAP_disableCounterResetOnEvent(ECAP2_CAPTURE_BASE_ADDR,ECAP_EVENT_3);
	ECAP_disableCounterResetOnEvent(ECAP2_CAPTURE_BASE_ADDR,ECAP_EVENT_4);
	/* Select eCAP input */
    /* Select qualification period to filter out noise*/
	ECAP_selectECAPInput(ECAP2_CAPTURE_BASE_ADDR,ECAP_INPUT_INPUTXBAR2);
    ECAP_selectQualPeriod(ECAP2_CAPTURE_BASE_ADDR,ECAP_PULSE_WIDTH_FILTER_BYPASS);
	/* Sets a phase shift value count */
	ECAP_setPhaseShiftCount(ECAP2_CAPTURE_BASE_ADDR,0);
	/* Enable counter loading with phase shift value */
	ECAP_enableLoadCounter(ECAP2_CAPTURE_BASE_ADDR);
	/* Load time stamp counter */
	ECAP_loadCounter(ECAP2_CAPTURE_BASE_ADDR);
	/* Configures Sync out signal mode */
	ECAP_setSyncOutMode(ECAP2_CAPTURE_BASE_ADDR,ECAP_SYNC_OUT_DISABLED);

	/* Configures emulation mode */
	ECAP_setEmulationMode(ECAP2_CAPTURE_BASE_ADDR,ECAP_EMULATION_STOP);
	/* Set up the source for sync-in pulse */
	ECAP_setSyncInPulseSource(ECAP2_CAPTURE_BASE_ADDR,ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0);

	/* Disables time stamp capture */
	ECAP_disableTimeStampCapture(ECAP3_CAPTURE_BASE_ADDR);
	/* Stops Time stamp counter */
	ECAP_stopCounter(ECAP3_CAPTURE_BASE_ADDR);
	/* Sets eCAP in Capture mode. */
	ECAP_enableCaptureMode(ECAP3_CAPTURE_BASE_ADDR);
	/* Sets the capture mode */
	ECAP_setCaptureMode(ECAP3_CAPTURE_BASE_ADDR,ECAP_CONTINUOUS_CAPTURE_MODE,ECAP_EVENT_1);
	/* Sets the Capture event polarity */
	ECAP_setEventPrescaler(ECAP3_CAPTURE_BASE_ADDR, 0);
	/* Sets the Capture event polarity */
	ECAP_setEventPolarity(ECAP3_CAPTURE_BASE_ADDR,ECAP_EVENT_1,ECAP_EVNT_FALLING_EDGE);
	ECAP_setEventPolarity(ECAP3_CAPTURE_BASE_ADDR,ECAP_EVENT_2,ECAP_EVNT_RISING_EDGE);
	ECAP_setEventPolarity(ECAP3_CAPTURE_BASE_ADDR,ECAP_EVENT_3,ECAP_EVNT_RISING_EDGE);
	ECAP_setEventPolarity(ECAP3_CAPTURE_BASE_ADDR,ECAP_EVENT_4,ECAP_EVNT_RISING_EDGE);
	/* Configure counter reset on events */
	ECAP_disableCounterResetOnEvent(ECAP3_CAPTURE_BASE_ADDR,ECAP_EVENT_1);
	ECAP_disableCounterResetOnEvent(ECAP3_CAPTURE_BASE_ADDR,ECAP_EVENT_2);
	ECAP_disableCounterResetOnEvent(ECAP3_CAPTURE_BASE_ADDR,ECAP_EVENT_3);
	ECAP_disableCounterResetOnEvent(ECAP3_CAPTURE_BASE_ADDR,ECAP_EVENT_4);
	/* Select eCAP input */
    /* Select qualification period to filter out noise*/
	ECAP_selectECAPInput(ECAP3_CAPTURE_BASE_ADDR,ECAP_INPUT_INPUTXBAR3);
    ECAP_selectQualPeriod(ECAP3_CAPTURE_BASE_ADDR,ECAP_PULSE_WIDTH_FILTER_BYPASS);
	/* Sets a phase shift value count */
	ECAP_setPhaseShiftCount(ECAP3_CAPTURE_BASE_ADDR,0);
	/* Enable counter loading with phase shift value */
	ECAP_enableLoadCounter(ECAP3_CAPTURE_BASE_ADDR);
	/* Load time stamp counter */
	ECAP_loadCounter(ECAP3_CAPTURE_BASE_ADDR);
	/* Configures Sync out signal mode */
	ECAP_setSyncOutMode(ECAP3_CAPTURE_BASE_ADDR,ECAP_SYNC_OUT_DISABLED);

	/* Configures emulation mode */
	ECAP_setEmulationMode(ECAP3_CAPTURE_BASE_ADDR,ECAP_EMULATION_STOP);
	/* Set up the source for sync-in pulse */
	ECAP_setSyncInPulseSource(ECAP3_CAPTURE_BASE_ADDR,ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0);

	/* Disables time stamp capture */
	ECAP_disableTimeStampCapture(ECAP4_CAPTURE_BASE_ADDR);
	/* Stops Time stamp counter */
	ECAP_stopCounter(ECAP4_CAPTURE_BASE_ADDR);
	/* Sets eCAP in Capture mode. */
	ECAP_enableCaptureMode(ECAP4_CAPTURE_BASE_ADDR);
	/* Sets the capture mode */
	ECAP_setCaptureMode(ECAP4_CAPTURE_BASE_ADDR,ECAP_CONTINUOUS_CAPTURE_MODE,ECAP_EVENT_1);
	/* Sets the Capture event polarity */
	ECAP_setEventPrescaler(ECAP4_CAPTURE_BASE_ADDR, 0);
	/* Sets the Capture event polarity */
	ECAP_setEventPolarity(ECAP4_CAPTURE_BASE_ADDR,ECAP_EVENT_1,ECAP_EVNT_FALLING_EDGE);
	ECAP_setEventPolarity(ECAP4_CAPTURE_BASE_ADDR,ECAP_EVENT_2,ECAP_EVNT_RISING_EDGE);
	ECAP_setEventPolarity(ECAP4_CAPTURE_BASE_ADDR,ECAP_EVENT_3,ECAP_EVNT_RISING_EDGE);
	ECAP_setEventPolarity(ECAP4_CAPTURE_BASE_ADDR,ECAP_EVENT_4,ECAP_EVNT_RISING_EDGE);
	/* Configure counter reset on events */
	ECAP_disableCounterResetOnEvent(ECAP4_CAPTURE_BASE_ADDR,ECAP_EVENT_1);
	ECAP_disableCounterResetOnEvent(ECAP4_CAPTURE_BASE_ADDR,ECAP_EVENT_2);
	ECAP_disableCounterResetOnEvent(ECAP4_CAPTURE_BASE_ADDR,ECAP_EVENT_3);
	ECAP_disableCounterResetOnEvent(ECAP4_CAPTURE_BASE_ADDR,ECAP_EVENT_4);
	/* Select eCAP input */
    /* Select qualification period to filter out noise*/
	ECAP_selectECAPInput(ECAP4_CAPTURE_BASE_ADDR,ECAP_INPUT_INPUTXBAR4);
    ECAP_selectQualPeriod(ECAP4_CAPTURE_BASE_ADDR,ECAP_PULSE_WIDTH_FILTER_BYPASS);
	/* Sets a phase shift value count */
	ECAP_setPhaseShiftCount(ECAP4_CAPTURE_BASE_ADDR,0);
	/* Enable counter loading with phase shift value */
	ECAP_enableLoadCounter(ECAP4_CAPTURE_BASE_ADDR);
	/* Load time stamp counter */
	ECAP_loadCounter(ECAP4_CAPTURE_BASE_ADDR);
	/* Configures Sync out signal mode */
	ECAP_setSyncOutMode(ECAP4_CAPTURE_BASE_ADDR,ECAP_SYNC_OUT_DISABLED);

	/* Configures emulation mode */
	ECAP_setEmulationMode(ECAP4_CAPTURE_BASE_ADDR,ECAP_EMULATION_STOP);
	/* Set up the source for sync-in pulse */
	ECAP_setSyncInPulseSource(ECAP4_CAPTURE_BASE_ADDR,ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0);

	/* Disables time stamp capture */
	ECAP_disableTimeStampCapture(ECAP5_CAPTURE_BASE_ADDR);
	/* Stops Time stamp counter */
	ECAP_stopCounter(ECAP5_CAPTURE_BASE_ADDR);
	/* Sets eCAP in Capture mode. */
	ECAP_enableCaptureMode(ECAP5_CAPTURE_BASE_ADDR);
	/* Sets the capture mode */
	ECAP_setCaptureMode(ECAP5_CAPTURE_BASE_ADDR,ECAP_CONTINUOUS_CAPTURE_MODE,ECAP_EVENT_1);
	/* Sets the Capture event polarity */
	ECAP_setEventPrescaler(ECAP5_CAPTURE_BASE_ADDR, 0);
	/* Sets the Capture event polarity */
	ECAP_setEventPolarity(ECAP5_CAPTURE_BASE_ADDR,ECAP_EVENT_1,ECAP_EVNT_FALLING_EDGE);
	ECAP_setEventPolarity(ECAP5_CAPTURE_BASE_ADDR,ECAP_EVENT_2,ECAP_EVNT_RISING_EDGE);
	ECAP_setEventPolarity(ECAP5_CAPTURE_BASE_ADDR,ECAP_EVENT_3,ECAP_EVNT_RISING_EDGE);
	ECAP_setEventPolarity(ECAP5_CAPTURE_BASE_ADDR,ECAP_EVENT_4,ECAP_EVNT_RISING_EDGE);
	/* Configure counter reset on events */
	ECAP_disableCounterResetOnEvent(ECAP5_CAPTURE_BASE_ADDR,ECAP_EVENT_1);
	ECAP_disableCounterResetOnEvent(ECAP5_CAPTURE_BASE_ADDR,ECAP_EVENT_2);
	ECAP_disableCounterResetOnEvent(ECAP5_CAPTURE_BASE_ADDR,ECAP_EVENT_3);
	ECAP_disableCounterResetOnEvent(ECAP5_CAPTURE_BASE_ADDR,ECAP_EVENT_4);
	/* Select eCAP input */
    /* Select qualification period to filter out noise*/
	ECAP_selectECAPInput(ECAP5_CAPTURE_BASE_ADDR,ECAP_INPUT_INPUTXBAR5);
    ECAP_selectQualPeriod(ECAP5_CAPTURE_BASE_ADDR,ECAP_PULSE_WIDTH_FILTER_BYPASS);
	/* Sets a phase shift value count */
	ECAP_setPhaseShiftCount(ECAP5_CAPTURE_BASE_ADDR,0);
	/* Enable counter loading with phase shift value */
	ECAP_enableLoadCounter(ECAP5_CAPTURE_BASE_ADDR);
	/* Load time stamp counter */
	ECAP_loadCounter(ECAP5_CAPTURE_BASE_ADDR);
	/* Configures Sync out signal mode */
	ECAP_setSyncOutMode(ECAP5_CAPTURE_BASE_ADDR,ECAP_SYNC_OUT_DISABLED);

	/* Configures emulation mode */
	ECAP_setEmulationMode(ECAP5_CAPTURE_BASE_ADDR,ECAP_EMULATION_STOP);
	/* Set up the source for sync-in pulse */
	ECAP_setSyncInPulseSource(ECAP5_CAPTURE_BASE_ADDR,ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0);

	/* Starts Time stamp counter for REF_CAPTURE */
	ECAP_startCounter(REF_CAPTURE_BASE_ADDR);
	/* Starts Time stamp counter for ECAP1_CAPTURE */
	ECAP_startCounter(ECAP1_CAPTURE_BASE_ADDR);
	/* Starts Time stamp counter for ECAP2_CAPTURE */
	ECAP_startCounter(ECAP2_CAPTURE_BASE_ADDR);
	/* Starts Time stamp counter for ECAP3_CAPTURE */
	ECAP_startCounter(ECAP3_CAPTURE_BASE_ADDR);
	/* Starts Time stamp counter for ECAP4_CAPTURE */
	ECAP_startCounter(ECAP4_CAPTURE_BASE_ADDR);
	/* Starts Time stamp counter for ECAP5_CAPTURE */
	ECAP_startCounter(ECAP5_CAPTURE_BASE_ADDR);
	/* Enables time stamp capture for REF_CAPTURE */
	ECAP_enableTimeStampCapture(REF_CAPTURE_BASE_ADDR);
	/* Enables time stamp capture for ECAP1_CAPTURE */
	ECAP_enableTimeStampCapture(ECAP1_CAPTURE_BASE_ADDR);
	/* Enables time stamp capture for ECAP2_CAPTURE */
	ECAP_enableTimeStampCapture(ECAP2_CAPTURE_BASE_ADDR);
	/* Enables time stamp capture for ECAP3_CAPTURE */
	ECAP_enableTimeStampCapture(ECAP3_CAPTURE_BASE_ADDR);
	/* Enables time stamp capture for ECAP4_CAPTURE */
	ECAP_enableTimeStampCapture(ECAP4_CAPTURE_BASE_ADDR);
	/* Enables time stamp capture for ECAP5_CAPTURE */
	ECAP_enableTimeStampCapture(ECAP5_CAPTURE_BASE_ADDR);



    //-----------------Signal Monitoring--------------------//

    ECAP_selectGlobalLoadStrobe(REF_CAPTURE_BASE_ADDR, ECAP_MUNIT_GLDSTRB_DISABLED);
    ECAP_selectGlobalLoadStrobe(ECAP1_CAPTURE_BASE_ADDR, ECAP_MUNIT_GLDSTRB_DISABLED);
    ECAP_selectGlobalLoadStrobe(ECAP2_CAPTURE_BASE_ADDR, ECAP_MUNIT_GLDSTRB_DISABLED);
    ECAP_selectGlobalLoadStrobe(ECAP3_CAPTURE_BASE_ADDR, ECAP_MUNIT_GLDSTRB_DISABLED);
    ECAP_selectGlobalLoadStrobe(ECAP4_CAPTURE_BASE_ADDR, ECAP_MUNIT_GLDSTRB_DISABLED);
    ECAP_selectGlobalLoadStrobe(ECAP5_CAPTURE_BASE_ADDR, ECAP_MUNIT_GLDSTRB_DISABLED);
}

void Drivers_epwmOpen(void)
{
	/* CONFIG_EPWM0 initialization */

	/* Time Base */
	EPWM_setEmulationMode(CONFIG_EPWM0_BASE_ADDR, EPWM_EMULATION_STOP_AFTER_NEXT_TB);
	EPWM_setClockPrescaler(CONFIG_EPWM0_BASE_ADDR, EPWM_CLOCK_DIVIDER_4, EPWM_HSCLOCK_DIVIDER_1);
	EPWM_setTimeBasePeriod(CONFIG_EPWM0_BASE_ADDR, 20000);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM0_BASE_ADDR, EPWM_GL_REGISTER_TBPRD_TBPRDHR);
	EPWM_setPeriodLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_PERIOD_SHADOW_LOAD);
	EPWM_setTimeBaseCounter(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_setTimeBaseCounterMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_MODE_UP);
	EPWM_setCountModeAfterSync(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
	EPWM_disablePhaseShiftLoad(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setPhaseShift(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_enableSyncOutPulseSource(CONFIG_EPWM0_BASE_ADDR, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);
	EPWM_setSyncInPulseSource(CONFIG_EPWM0_BASE_ADDR, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0);
	EPWM_setOneShotSyncOutTrigger(CONFIG_EPWM0_BASE_ADDR, EPWM_OSHT_SYNC_OUT_TRIG_SYNC);
	

	/* Counter Compare */
	EPWM_setCounterCompareValue(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 1000);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM0_BASE_ADDR, EPWM_GL_REGISTER_CMPA_CMPAHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_B, 500);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM0_BASE_ADDR, EPWM_GL_REGISTER_CMPB_CMPBHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_C, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM0_BASE_ADDR, EPWM_GL_REGISTER_CMPC);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_C, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_D, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM0_BASE_ADDR, EPWM_GL_REGISTER_CMPD);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_D, EPWM_COMP_LOAD_ON_CNTR_ZERO);

	/* Action Qualifier */
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM0_BASE_ADDR, EPWM_GL_REGISTER_AQCSFRC);
	EPWM_setActionQualifierContSWForceShadowMode(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM0_BASE_ADDR, EPWM_GL_REGISTER_AQCTLA_AQCTLA2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_ACTION_QUALIFIER_A);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierT1TriggerSource(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierT2TriggerSource(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM0_BASE_ADDR, EPWM_GL_REGISTER_AQCTLB_AQCTLB2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_ACTION_QUALIFIER_B);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_ACTION_QUALIFIER_B, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);

	/* Events */
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

	/* Trip Zone */
    EPWM_setTripZoneAction(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_disableTripZoneAdvAction(CONFIG_EPWM0_BASE_ADDR);
	EPWM_enableTripZoneSignals(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_enableTripZoneSignals(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_selectCycleByCycleTripZoneClearEvent(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO);
	EPWM_enableTripZoneInterrupt(CONFIG_EPWM0_BASE_ADDR, 0);

	/* Digital Compare */
	EPWM_setDigitalCompareFilterInput(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_WINDOW_SOURCE_DCAEVT1);
	EPWM_disableDigitalCompareBlankingWindow(CONFIG_EPWM0_BASE_ADDR);
    EPWM_setDigitalCompareBlankingEvent(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_WINDOW_START_TBCTR_PERIOD, 0);
	EPWM_setDigitalCompareWindowOffset(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_setDigitalCompareWindowLength(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_disableDigitalCompareWindowInverseMode(CONFIG_EPWM0_BASE_ADDR);
	EPWM_disableDigitalCompareCounterCapture(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setDigitalCompareCounterShadowMode(CONFIG_EPWM0_BASE_ADDR, false);
	EPWM_disableDigitalCompareEdgeFilter(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setDigitalCompareEdgeFilterMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_EDGEFILT_MODE_RISING);
	EPWM_setDigitalCompareEdgeFilterEdgeCount(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_EDGEFILT_EDGECNT_0);
	EPWM_disableValleyCapture(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setValleyTriggerSource(CONFIG_EPWM0_BASE_ADDR, EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE);
	
	EPWM_setValleyTriggerEdgeCounts(CONFIG_EPWM0_BASE_ADDR, 0, 0);
	EPWM_disableValleyHWDelay(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setValleySWDelayValue(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_setValleyDelayDivider(CONFIG_EPWM0_BASE_ADDR, EPWM_VALLEY_DELAY_MODE_SW_DELAY);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM0_BASE_ADDR, 0, EPWM_DC_TYPE_DCAH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM0_BASE_ADDR, 0, EPWM_DC_TYPE_DCAL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A2, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_A);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM0_BASE_ADDR, 0, EPWM_DC_TYPE_DCBH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM0_BASE_ADDR, 0, EPWM_DC_TYPE_DCBL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B2, EPWM_TZ_EVENT_DC_DISABLED);

	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_B);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);

	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);

	/* Deadband */
    EPWM_disableDeadBandControlShadowLoadMode(CONFIG_EPWM0_BASE_ADDR);
    EPWM_setDeadBandControlShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_LOAD_ON_CNTR_ZERO);
	EPWM_setRisingEdgeDeadBandDelayInput(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setFallingEdgeDeadBandDelayInput(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_RED, false);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_FED, false);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_OUTPUT_A, false);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_OUTPUT_B, false);
	
	
	EPWM_disableRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_setRisingEdgeDelayCount(CONFIG_EPWM0_BASE_ADDR, 0);
	
	EPWM_disableFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_setFallingEdgeDelayCount(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_setDeadBandCounterClock(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

	/* Chopper */
	EPWM_disableChopper(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setChopperDutyCycle(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_setChopperFreq(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_setChopperFirstPulseWidth(CONFIG_EPWM0_BASE_ADDR, 0);

	/* Event Trigger */
	EPWM_disableInterrupt(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setInterruptSource(CONFIG_EPWM0_BASE_ADDR, EPWM_INT_TBCTR_ZERO, 0);
	EPWM_setInterruptEventCount(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_disableInterruptEventCountInit(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setInterruptEventCountInitValue(CONFIG_EPWM0_BASE_ADDR, 0);
	
	EPWM_disableADCTrigger(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A);
    EPWM_setADCTriggerSource(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A, EPWM_SOC_DCxEVT1, 0);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A, 0);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A, 0);
	
	EPWM_disableADCTrigger(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_B);
    EPWM_setADCTriggerSource(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_B, EPWM_SOC_DCxEVT1, EPWM_SOC_DCxEVT1);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_B, 0);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_B);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_B, 0);
	

    /* XCMP Mode */
    EPWM_disableXCMPMode(CONFIG_EPWM0_BASE_ADDR);
    EPWM_disableSplitXCMP(CONFIG_EPWM0_BASE_ADDR);
	EPWM_allocAXCMP(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_NONE_CMPA);
    EPWM_setXCMPLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
    EPWM_setXCMPShadowLevel(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWLEVEL_0);
    EPWM_setXCMPShadowBufPtrLoadOnce(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, 0);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, 0);

    /* Write values to Reg */
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP1_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP2_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP3_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP4_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP5_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP6_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP7_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP8_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP1_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP2_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP3_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP4_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP5_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP6_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP7_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP8_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP1_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP2_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP3_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP4_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP5_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP6_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP7_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP8_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP1_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP2_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP3_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP4_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP5_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP6_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP7_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP8_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XTBPRD_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XTBPRD_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XTBPRD_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XTBPRD_SHADOW3, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XMIN_ACTIVE, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XMAX_ACTIVE, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XMIN_SHADOW1, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XMAX_SHADOW1, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XMIN_SHADOW2, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XMAX_SHADOW2, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XMIN_SHADOW3, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XMAX_SHADOW3, 0);

    /* Events */

    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);

    /* Diode Emulation */
    EPWM_disableDiodeEmulationMode(CONFIG_EPWM0_BASE_ADDR);
    EPWM_setDiodeEmulationMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DIODE_EMULATION_CBC);
    EPWM_setDiodeEmulationReentryDelay(CONFIG_EPWM0_BASE_ADDR, 0);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM0_BASE_ADDR, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0, EPWM_DE_TRIPL);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM0_BASE_ADDR, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0, EPWM_DE_TRIPH);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM0_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM0_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_TRIPL);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM0_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM0_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_TRIPL);
    EPWM_nobypassDiodeEmulationLogic(CONFIG_EPWM0_BASE_ADDR);
    
    EPWM_disableDiodeEmulationMonitorModeControl(CONFIG_EPWM0_BASE_ADDR);
    EPWM_setDiodeEmulationMonitorCounterThreshold(CONFIG_EPWM0_BASE_ADDR, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM0_BASE_ADDR, EPWM_DE_COUNT_DOWN, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM0_BASE_ADDR, EPWM_DE_COUNT_UP, 0);

    /* HRPWM */
    HRPWM_disableAutoConversion(CONFIG_EPWM0_BASE_ADDR);
    HRPWM_setMEPControlMode(CONFIG_EPWM0_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setMEPControlMode(CONFIG_EPWM0_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setHiResPhaseShift(CONFIG_EPWM0_BASE_ADDR, 0);
    HRPWM_setSyncPulseSource(CONFIG_EPWM0_BASE_ADDR, HRPWM_PWMSYNC_SOURCE_PERIOD);
    HRPWM_disablePhaseShiftLoad(CONFIG_EPWM0_BASE_ADDR);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM0_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM0_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM0_BASE_ADDR, HRPWM_COUNTER_COMPARE_A, 1);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM0_BASE_ADDR, HRPWM_COUNTER_COMPARE_B, 1);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM0_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM0_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_disablePeriodControl(CONFIG_EPWM0_BASE_ADDR);
    HRPWM_setHiResTimeBasePeriod(CONFIG_EPWM0_BASE_ADDR, 0);
    HRPWM_setDeadbandMEPEdgeSelect(CONFIG_EPWM0_BASE_ADDR, HRPWM_DB_MEP_CTRL_DISABLE);
    HRPWM_setHiResRisingEdgeDelay(CONFIG_EPWM0_BASE_ADDR, 0);
    HRPWM_setRisingEdgeDelayLoadMode(CONFIG_EPWM0_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setHiResFallingEdgeDelayOnly(CONFIG_EPWM0_BASE_ADDR, 0);
    HRPWM_setFallingEdgeDelayLoadMode(CONFIG_EPWM0_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setOutputSwapMode(CONFIG_EPWM0_BASE_ADDR, false);
    HRPWM_setChannelBOutputPath(CONFIG_EPWM0_BASE_ADDR, HRPWM_OUTPUT_ON_B_NORMAL);

    /* ICL */
    EPWM_disableIllegalComboLogic(CONFIG_EPWM0_BASE_ADDR, EPWM_MINDB_BLOCK_A);
    EPWM_disableIllegalComboLogic(CONFIG_EPWM0_BASE_ADDR, EPWM_MINDB_BLOCK_B);

    /* MDL */
    EPWM_disableMinimumDeadBand(CONFIG_EPWM0_BASE_ADDR, EPWM_MINDB_BLOCK_A);
    EPWM_disableMinimumDeadBand(CONFIG_EPWM0_BASE_ADDR, EPWM_MINDB_BLOCK_B);
	/* Global Load */
	EPWM_disableGlobalLoad(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setGlobalLoadTrigger(CONFIG_EPWM0_BASE_ADDR, EPWM_GL_LOAD_PULSE_CNTR_ZERO);
	EPWM_setGlobalLoadEventPrescale(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_disableGlobalLoadOneShotMode(CONFIG_EPWM0_BASE_ADDR);
	
	

	/* EPWM Module */
	EPWM_lockRegisters(CONFIG_EPWM0_BASE_ADDR, 0);
	/* CONFIG_EPWM1 initialization */

	/* Time Base */
	EPWM_setEmulationMode(CONFIG_EPWM1_BASE_ADDR, EPWM_EMULATION_STOP_AFTER_NEXT_TB);
	EPWM_setClockPrescaler(CONFIG_EPWM1_BASE_ADDR, EPWM_CLOCK_DIVIDER_4, EPWM_HSCLOCK_DIVIDER_1);
	EPWM_setTimeBasePeriod(CONFIG_EPWM1_BASE_ADDR, 20000);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM1_BASE_ADDR, EPWM_GL_REGISTER_TBPRD_TBPRDHR);
	EPWM_setPeriodLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_PERIOD_SHADOW_LOAD);
	EPWM_setTimeBaseCounter(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_setTimeBaseCounterMode(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_MODE_UP);
	EPWM_setCountModeAfterSync(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
	EPWM_enablePhaseShiftLoad(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setPhaseShift(CONFIG_EPWM1_BASE_ADDR, 300);
	EPWM_enableSyncOutPulseSource(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_setSyncInPulseSource(CONFIG_EPWM1_BASE_ADDR, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0);
	EPWM_setOneShotSyncOutTrigger(CONFIG_EPWM1_BASE_ADDR, EPWM_OSHT_SYNC_OUT_TRIG_SYNC);
	

	/* Counter Compare */
	EPWM_setCounterCompareValue(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 2000);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM1_BASE_ADDR, EPWM_GL_REGISTER_CMPA_CMPAHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_B, 4000);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM1_BASE_ADDR, EPWM_GL_REGISTER_CMPB_CMPBHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_C, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM1_BASE_ADDR, EPWM_GL_REGISTER_CMPC);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_C, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_D, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM1_BASE_ADDR, EPWM_GL_REGISTER_CMPD);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_D, EPWM_COMP_LOAD_ON_CNTR_ZERO);

	/* Action Qualifier */
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM1_BASE_ADDR, EPWM_GL_REGISTER_AQCSFRC);
	EPWM_setActionQualifierContSWForceShadowMode(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM1_BASE_ADDR, EPWM_GL_REGISTER_AQCTLA_AQCTLA2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_ACTION_QUALIFIER_A);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierT1TriggerSource(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierT2TriggerSource(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM1_BASE_ADDR, EPWM_GL_REGISTER_AQCTLB_AQCTLB2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_ACTION_QUALIFIER_B);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_ACTION_QUALIFIER_B, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);

	/* Events */
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

	/* Trip Zone */
    EPWM_setTripZoneAction(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_disableTripZoneAdvAction(CONFIG_EPWM1_BASE_ADDR);
	EPWM_enableTripZoneSignals(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_enableTripZoneSignals(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_selectCycleByCycleTripZoneClearEvent(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO);
	EPWM_enableTripZoneInterrupt(CONFIG_EPWM1_BASE_ADDR, 0);

	/* Digital Compare */
	EPWM_setDigitalCompareFilterInput(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_WINDOW_SOURCE_DCAEVT1);
	EPWM_disableDigitalCompareBlankingWindow(CONFIG_EPWM1_BASE_ADDR);
    EPWM_setDigitalCompareBlankingEvent(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_WINDOW_START_TBCTR_PERIOD, 0);
	EPWM_setDigitalCompareWindowOffset(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_setDigitalCompareWindowLength(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_disableDigitalCompareWindowInverseMode(CONFIG_EPWM1_BASE_ADDR);
	EPWM_disableDigitalCompareCounterCapture(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setDigitalCompareCounterShadowMode(CONFIG_EPWM1_BASE_ADDR, false);
	EPWM_disableDigitalCompareEdgeFilter(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setDigitalCompareEdgeFilterMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_EDGEFILT_MODE_RISING);
	EPWM_setDigitalCompareEdgeFilterEdgeCount(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_EDGEFILT_EDGECNT_0);
	EPWM_disableValleyCapture(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setValleyTriggerSource(CONFIG_EPWM1_BASE_ADDR, EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE);
	
	EPWM_setValleyTriggerEdgeCounts(CONFIG_EPWM1_BASE_ADDR, 0, 0);
	EPWM_disableValleyHWDelay(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setValleySWDelayValue(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_setValleyDelayDivider(CONFIG_EPWM1_BASE_ADDR, EPWM_VALLEY_DELAY_MODE_SW_DELAY);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM1_BASE_ADDR, 0, EPWM_DC_TYPE_DCAH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM1_BASE_ADDR, 0, EPWM_DC_TYPE_DCAL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A2, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_A);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM1_BASE_ADDR, 0, EPWM_DC_TYPE_DCBH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM1_BASE_ADDR, 0, EPWM_DC_TYPE_DCBL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B2, EPWM_TZ_EVENT_DC_DISABLED);

	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_B);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);

	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);

	/* Deadband */
    EPWM_disableDeadBandControlShadowLoadMode(CONFIG_EPWM1_BASE_ADDR);
    EPWM_setDeadBandControlShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_LOAD_ON_CNTR_ZERO);
	EPWM_setRisingEdgeDeadBandDelayInput(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setFallingEdgeDeadBandDelayInput(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_RED, false);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_FED, false);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_OUTPUT_A, false);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_OUTPUT_B, false);
	
	
	EPWM_disableRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_setRisingEdgeDelayCount(CONFIG_EPWM1_BASE_ADDR, 0);
	
	EPWM_disableFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_setFallingEdgeDelayCount(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_setDeadBandCounterClock(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

	/* Chopper */
	EPWM_disableChopper(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setChopperDutyCycle(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_setChopperFreq(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_setChopperFirstPulseWidth(CONFIG_EPWM1_BASE_ADDR, 0);

	/* Event Trigger */
	EPWM_disableInterrupt(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setInterruptSource(CONFIG_EPWM1_BASE_ADDR, EPWM_INT_TBCTR_ZERO, 0);
	EPWM_setInterruptEventCount(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_disableInterruptEventCountInit(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setInterruptEventCountInitValue(CONFIG_EPWM1_BASE_ADDR, 0);
	
	EPWM_disableADCTrigger(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_A);
    EPWM_setADCTriggerSource(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_A, EPWM_SOC_DCxEVT1, 0);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_A, 0);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_A, 0);
	
	EPWM_disableADCTrigger(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_B);
    EPWM_setADCTriggerSource(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_B, EPWM_SOC_DCxEVT1, EPWM_SOC_DCxEVT1);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_B, 0);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_B);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_B, 0);
	

    /* XCMP Mode */
    EPWM_disableXCMPMode(CONFIG_EPWM1_BASE_ADDR);
    EPWM_disableSplitXCMP(CONFIG_EPWM1_BASE_ADDR);
	EPWM_allocAXCMP(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_NONE_CMPA);
    EPWM_setXCMPLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
    EPWM_setXCMPShadowLevel(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWLEVEL_0);
    EPWM_setXCMPShadowBufPtrLoadOnce(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, 0);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, 0);

    /* Write values to Reg */
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP1_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP2_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP3_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP4_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP5_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP6_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP7_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP8_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP1_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP2_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP3_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP4_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP5_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP6_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP7_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP8_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP1_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP2_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP3_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP4_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP5_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP6_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP7_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP8_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP1_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP2_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP3_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP4_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP5_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP6_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP7_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP8_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XTBPRD_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XTBPRD_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XTBPRD_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XTBPRD_SHADOW3, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XMIN_ACTIVE, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XMAX_ACTIVE, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XMIN_SHADOW1, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XMAX_SHADOW1, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XMIN_SHADOW2, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XMAX_SHADOW2, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XMIN_SHADOW3, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XMAX_SHADOW3, 0);

    /* Events */

    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);

    /* Diode Emulation */
    EPWM_disableDiodeEmulationMode(CONFIG_EPWM1_BASE_ADDR);
    EPWM_setDiodeEmulationMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DIODE_EMULATION_CBC);
    EPWM_setDiodeEmulationReentryDelay(CONFIG_EPWM1_BASE_ADDR, 0);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM1_BASE_ADDR, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0, EPWM_DE_TRIPL);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM1_BASE_ADDR, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0, EPWM_DE_TRIPH);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM1_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM1_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_TRIPL);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM1_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM1_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_TRIPL);
    EPWM_nobypassDiodeEmulationLogic(CONFIG_EPWM1_BASE_ADDR);
    
    EPWM_disableDiodeEmulationMonitorModeControl(CONFIG_EPWM1_BASE_ADDR);
    EPWM_setDiodeEmulationMonitorCounterThreshold(CONFIG_EPWM1_BASE_ADDR, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM1_BASE_ADDR, EPWM_DE_COUNT_DOWN, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM1_BASE_ADDR, EPWM_DE_COUNT_UP, 0);

    /* HRPWM */
    HRPWM_disableAutoConversion(CONFIG_EPWM1_BASE_ADDR);
    HRPWM_setMEPControlMode(CONFIG_EPWM1_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setMEPControlMode(CONFIG_EPWM1_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setHiResPhaseShift(CONFIG_EPWM1_BASE_ADDR, 0);
    HRPWM_setSyncPulseSource(CONFIG_EPWM1_BASE_ADDR, HRPWM_PWMSYNC_SOURCE_PERIOD);
    HRPWM_disablePhaseShiftLoad(CONFIG_EPWM1_BASE_ADDR);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM1_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM1_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM1_BASE_ADDR, HRPWM_COUNTER_COMPARE_A, 1);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM1_BASE_ADDR, HRPWM_COUNTER_COMPARE_B, 1);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM1_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM1_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_disablePeriodControl(CONFIG_EPWM1_BASE_ADDR);
    HRPWM_setHiResTimeBasePeriod(CONFIG_EPWM1_BASE_ADDR, 0);
    HRPWM_setDeadbandMEPEdgeSelect(CONFIG_EPWM1_BASE_ADDR, HRPWM_DB_MEP_CTRL_DISABLE);
    HRPWM_setHiResRisingEdgeDelay(CONFIG_EPWM1_BASE_ADDR, 0);
    HRPWM_setRisingEdgeDelayLoadMode(CONFIG_EPWM1_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setHiResFallingEdgeDelayOnly(CONFIG_EPWM1_BASE_ADDR, 0);
    HRPWM_setFallingEdgeDelayLoadMode(CONFIG_EPWM1_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setOutputSwapMode(CONFIG_EPWM1_BASE_ADDR, false);
    HRPWM_setChannelBOutputPath(CONFIG_EPWM1_BASE_ADDR, HRPWM_OUTPUT_ON_B_NORMAL);

    /* ICL */
    EPWM_disableIllegalComboLogic(CONFIG_EPWM1_BASE_ADDR, EPWM_MINDB_BLOCK_A);
    EPWM_disableIllegalComboLogic(CONFIG_EPWM1_BASE_ADDR, EPWM_MINDB_BLOCK_B);

    /* MDL */
    EPWM_disableMinimumDeadBand(CONFIG_EPWM1_BASE_ADDR, EPWM_MINDB_BLOCK_A);
    EPWM_disableMinimumDeadBand(CONFIG_EPWM1_BASE_ADDR, EPWM_MINDB_BLOCK_B);
	/* Global Load */
	EPWM_disableGlobalLoad(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setGlobalLoadTrigger(CONFIG_EPWM1_BASE_ADDR, EPWM_GL_LOAD_PULSE_CNTR_ZERO);
	EPWM_setGlobalLoadEventPrescale(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_disableGlobalLoadOneShotMode(CONFIG_EPWM1_BASE_ADDR);
	
	

	/* EPWM Module */
	EPWM_lockRegisters(CONFIG_EPWM1_BASE_ADDR, 0);
	/* CONFIG_EPWM2 initialization */

	/* Time Base */
	EPWM_setEmulationMode(CONFIG_EPWM2_BASE_ADDR, EPWM_EMULATION_STOP_AFTER_NEXT_TB);
	EPWM_setClockPrescaler(CONFIG_EPWM2_BASE_ADDR, EPWM_CLOCK_DIVIDER_4, EPWM_HSCLOCK_DIVIDER_1);
	EPWM_setTimeBasePeriod(CONFIG_EPWM2_BASE_ADDR, 20000);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM2_BASE_ADDR, EPWM_GL_REGISTER_TBPRD_TBPRDHR);
	EPWM_setPeriodLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_PERIOD_SHADOW_LOAD);
	EPWM_setTimeBaseCounter(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_setTimeBaseCounterMode(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_MODE_UP);
	EPWM_setCountModeAfterSync(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
	EPWM_enablePhaseShiftLoad(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setPhaseShift(CONFIG_EPWM2_BASE_ADDR, 600);
	EPWM_enableSyncOutPulseSource(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_setSyncInPulseSource(CONFIG_EPWM2_BASE_ADDR, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0);
	EPWM_setOneShotSyncOutTrigger(CONFIG_EPWM2_BASE_ADDR, EPWM_OSHT_SYNC_OUT_TRIG_SYNC);
	

	/* Counter Compare */
	EPWM_setCounterCompareValue(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 1000);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM2_BASE_ADDR, EPWM_GL_REGISTER_CMPA_CMPAHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_B, 500);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM2_BASE_ADDR, EPWM_GL_REGISTER_CMPB_CMPBHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_C, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM2_BASE_ADDR, EPWM_GL_REGISTER_CMPC);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_C, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_D, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM2_BASE_ADDR, EPWM_GL_REGISTER_CMPD);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_D, EPWM_COMP_LOAD_ON_CNTR_ZERO);

	/* Action Qualifier */
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM2_BASE_ADDR, EPWM_GL_REGISTER_AQCSFRC);
	EPWM_setActionQualifierContSWForceShadowMode(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM2_BASE_ADDR, EPWM_GL_REGISTER_AQCTLA_AQCTLA2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_ACTION_QUALIFIER_A);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierT1TriggerSource(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierT2TriggerSource(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM2_BASE_ADDR, EPWM_GL_REGISTER_AQCTLB_AQCTLB2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_ACTION_QUALIFIER_B);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_ACTION_QUALIFIER_B, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);

	/* Events */
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

	/* Trip Zone */
    EPWM_setTripZoneAction(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_disableTripZoneAdvAction(CONFIG_EPWM2_BASE_ADDR);
	EPWM_enableTripZoneSignals(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_enableTripZoneSignals(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_selectCycleByCycleTripZoneClearEvent(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO);
	EPWM_enableTripZoneInterrupt(CONFIG_EPWM2_BASE_ADDR, 0);

	/* Digital Compare */
	EPWM_setDigitalCompareFilterInput(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_WINDOW_SOURCE_DCAEVT1);
	EPWM_disableDigitalCompareBlankingWindow(CONFIG_EPWM2_BASE_ADDR);
    EPWM_setDigitalCompareBlankingEvent(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_WINDOW_START_TBCTR_PERIOD, 0);
	EPWM_setDigitalCompareWindowOffset(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_setDigitalCompareWindowLength(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_disableDigitalCompareWindowInverseMode(CONFIG_EPWM2_BASE_ADDR);
	EPWM_disableDigitalCompareCounterCapture(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setDigitalCompareCounterShadowMode(CONFIG_EPWM2_BASE_ADDR, false);
	EPWM_disableDigitalCompareEdgeFilter(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setDigitalCompareEdgeFilterMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_EDGEFILT_MODE_RISING);
	EPWM_setDigitalCompareEdgeFilterEdgeCount(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_EDGEFILT_EDGECNT_0);
	EPWM_disableValleyCapture(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setValleyTriggerSource(CONFIG_EPWM2_BASE_ADDR, EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE);
	
	EPWM_setValleyTriggerEdgeCounts(CONFIG_EPWM2_BASE_ADDR, 0, 0);
	EPWM_disableValleyHWDelay(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setValleySWDelayValue(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_setValleyDelayDivider(CONFIG_EPWM2_BASE_ADDR, EPWM_VALLEY_DELAY_MODE_SW_DELAY);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM2_BASE_ADDR, 0, EPWM_DC_TYPE_DCAH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM2_BASE_ADDR, 0, EPWM_DC_TYPE_DCAL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A2, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_A);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM2_BASE_ADDR, 0, EPWM_DC_TYPE_DCBH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM2_BASE_ADDR, 0, EPWM_DC_TYPE_DCBL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B2, EPWM_TZ_EVENT_DC_DISABLED);

	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_B);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);

	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);

	/* Deadband */
    EPWM_disableDeadBandControlShadowLoadMode(CONFIG_EPWM2_BASE_ADDR);
    EPWM_setDeadBandControlShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_LOAD_ON_CNTR_ZERO);
	EPWM_setRisingEdgeDeadBandDelayInput(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setFallingEdgeDeadBandDelayInput(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_LOW);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_RED, false);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_FED, false);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_OUTPUT_A, false);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_OUTPUT_B, false);
	
	
	EPWM_disableRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_setRisingEdgeDelayCount(CONFIG_EPWM2_BASE_ADDR, 0);
	
	EPWM_disableFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_setFallingEdgeDelayCount(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_setDeadBandCounterClock(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

	/* Chopper */
	EPWM_disableChopper(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setChopperDutyCycle(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_setChopperFreq(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_setChopperFirstPulseWidth(CONFIG_EPWM2_BASE_ADDR, 0);

	/* Event Trigger */
	EPWM_disableInterrupt(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setInterruptSource(CONFIG_EPWM2_BASE_ADDR, EPWM_INT_TBCTR_ZERO, 0);
	EPWM_setInterruptEventCount(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_disableInterruptEventCountInit(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setInterruptEventCountInitValue(CONFIG_EPWM2_BASE_ADDR, 0);
	
	EPWM_disableADCTrigger(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_A);
    EPWM_setADCTriggerSource(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_A, EPWM_SOC_DCxEVT1, 0);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_A, 0);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_A, 0);
	
	EPWM_disableADCTrigger(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_B);
    EPWM_setADCTriggerSource(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_B, EPWM_SOC_DCxEVT1, EPWM_SOC_DCxEVT1);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_B, 0);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_B);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_B, 0);
	

    /* XCMP Mode */
    EPWM_disableXCMPMode(CONFIG_EPWM2_BASE_ADDR);
    EPWM_disableSplitXCMP(CONFIG_EPWM2_BASE_ADDR);
	EPWM_allocAXCMP(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_NONE_CMPA);
    EPWM_setXCMPLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
    EPWM_setXCMPShadowLevel(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWLEVEL_0);
    EPWM_setXCMPShadowBufPtrLoadOnce(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, 0);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, 0);

    /* Write values to Reg */
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP1_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP2_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP3_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP4_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP5_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP6_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP7_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP8_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP1_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP2_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP3_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP4_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP5_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP6_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP7_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP8_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP1_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP2_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP3_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP4_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP5_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP6_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP7_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP8_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP1_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP2_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP3_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP4_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP5_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP6_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP7_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP8_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XTBPRD_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XTBPRD_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XTBPRD_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XTBPRD_SHADOW3, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XMIN_ACTIVE, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XMAX_ACTIVE, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XMIN_SHADOW1, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XMAX_SHADOW1, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XMIN_SHADOW2, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XMAX_SHADOW2, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XMIN_SHADOW3, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XMAX_SHADOW3, 0);

    /* Events */

    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);

    /* Diode Emulation */
    EPWM_disableDiodeEmulationMode(CONFIG_EPWM2_BASE_ADDR);
    EPWM_setDiodeEmulationMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DIODE_EMULATION_CBC);
    EPWM_setDiodeEmulationReentryDelay(CONFIG_EPWM2_BASE_ADDR, 0);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM2_BASE_ADDR, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0, EPWM_DE_TRIPL);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM2_BASE_ADDR, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0, EPWM_DE_TRIPH);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM2_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM2_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_TRIPL);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM2_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM2_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_TRIPL);
    EPWM_nobypassDiodeEmulationLogic(CONFIG_EPWM2_BASE_ADDR);
    
    EPWM_disableDiodeEmulationMonitorModeControl(CONFIG_EPWM2_BASE_ADDR);
    EPWM_setDiodeEmulationMonitorCounterThreshold(CONFIG_EPWM2_BASE_ADDR, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM2_BASE_ADDR, EPWM_DE_COUNT_DOWN, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM2_BASE_ADDR, EPWM_DE_COUNT_UP, 0);

    /* HRPWM */
    HRPWM_disableAutoConversion(CONFIG_EPWM2_BASE_ADDR);
    HRPWM_setMEPControlMode(CONFIG_EPWM2_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setMEPControlMode(CONFIG_EPWM2_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setHiResPhaseShift(CONFIG_EPWM2_BASE_ADDR, 0);
    HRPWM_setSyncPulseSource(CONFIG_EPWM2_BASE_ADDR, HRPWM_PWMSYNC_SOURCE_PERIOD);
    HRPWM_disablePhaseShiftLoad(CONFIG_EPWM2_BASE_ADDR);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM2_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM2_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM2_BASE_ADDR, HRPWM_COUNTER_COMPARE_A, 1);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM2_BASE_ADDR, HRPWM_COUNTER_COMPARE_B, 1);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM2_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM2_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_disablePeriodControl(CONFIG_EPWM2_BASE_ADDR);
    HRPWM_setHiResTimeBasePeriod(CONFIG_EPWM2_BASE_ADDR, 0);
    HRPWM_setDeadbandMEPEdgeSelect(CONFIG_EPWM2_BASE_ADDR, HRPWM_DB_MEP_CTRL_DISABLE);
    HRPWM_setHiResRisingEdgeDelay(CONFIG_EPWM2_BASE_ADDR, 0);
    HRPWM_setRisingEdgeDelayLoadMode(CONFIG_EPWM2_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setHiResFallingEdgeDelayOnly(CONFIG_EPWM2_BASE_ADDR, 0);
    HRPWM_setFallingEdgeDelayLoadMode(CONFIG_EPWM2_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setOutputSwapMode(CONFIG_EPWM2_BASE_ADDR, false);
    HRPWM_setChannelBOutputPath(CONFIG_EPWM2_BASE_ADDR, HRPWM_OUTPUT_ON_B_NORMAL);

    /* ICL */
    EPWM_disableIllegalComboLogic(CONFIG_EPWM2_BASE_ADDR, EPWM_MINDB_BLOCK_A);
    EPWM_disableIllegalComboLogic(CONFIG_EPWM2_BASE_ADDR, EPWM_MINDB_BLOCK_B);

    /* MDL */
    EPWM_disableMinimumDeadBand(CONFIG_EPWM2_BASE_ADDR, EPWM_MINDB_BLOCK_A);
    EPWM_disableMinimumDeadBand(CONFIG_EPWM2_BASE_ADDR, EPWM_MINDB_BLOCK_B);
	/* Global Load */
	EPWM_disableGlobalLoad(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setGlobalLoadTrigger(CONFIG_EPWM2_BASE_ADDR, EPWM_GL_LOAD_PULSE_CNTR_ZERO);
	EPWM_setGlobalLoadEventPrescale(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_disableGlobalLoadOneShotMode(CONFIG_EPWM2_BASE_ADDR);
	
	

	/* EPWM Module */
	EPWM_lockRegisters(CONFIG_EPWM2_BASE_ADDR, 0);
	/* CONFIG_EPWM3 initialization */

	/* Time Base */
	EPWM_setEmulationMode(CONFIG_EPWM3_BASE_ADDR, EPWM_EMULATION_STOP_AFTER_NEXT_TB);
	EPWM_setClockPrescaler(CONFIG_EPWM3_BASE_ADDR, EPWM_CLOCK_DIVIDER_2, EPWM_HSCLOCK_DIVIDER_1);
	EPWM_setTimeBasePeriod(CONFIG_EPWM3_BASE_ADDR, 2000);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM3_BASE_ADDR, EPWM_GL_REGISTER_TBPRD_TBPRDHR);
	EPWM_setPeriodLoadMode(CONFIG_EPWM3_BASE_ADDR, EPWM_PERIOD_SHADOW_LOAD);
	EPWM_setTimeBaseCounter(CONFIG_EPWM3_BASE_ADDR, 0);
	EPWM_setTimeBaseCounterMode(CONFIG_EPWM3_BASE_ADDR, EPWM_COUNTER_MODE_UP);
	EPWM_setCountModeAfterSync(CONFIG_EPWM3_BASE_ADDR, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
	EPWM_enablePhaseShiftLoad(CONFIG_EPWM3_BASE_ADDR);
	EPWM_setPhaseShift(CONFIG_EPWM3_BASE_ADDR, 900);
	EPWM_enableSyncOutPulseSource(CONFIG_EPWM3_BASE_ADDR, 0);
	EPWM_setSyncInPulseSource(CONFIG_EPWM3_BASE_ADDR, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0);
	EPWM_setOneShotSyncOutTrigger(CONFIG_EPWM3_BASE_ADDR, EPWM_OSHT_SYNC_OUT_TRIG_SYNC);
	

	/* Counter Compare */
	EPWM_setCounterCompareValue(CONFIG_EPWM3_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 1000);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM3_BASE_ADDR, EPWM_GL_REGISTER_CMPA_CMPAHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM3_BASE_ADDR, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM3_BASE_ADDR, EPWM_COUNTER_COMPARE_B, 500);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM3_BASE_ADDR, EPWM_GL_REGISTER_CMPB_CMPBHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM3_BASE_ADDR, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM3_BASE_ADDR, EPWM_COUNTER_COMPARE_C, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM3_BASE_ADDR, EPWM_GL_REGISTER_CMPC);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM3_BASE_ADDR, EPWM_COUNTER_COMPARE_C, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM3_BASE_ADDR, EPWM_COUNTER_COMPARE_D, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM3_BASE_ADDR, EPWM_GL_REGISTER_CMPD);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM3_BASE_ADDR, EPWM_COUNTER_COMPARE_D, EPWM_COMP_LOAD_ON_CNTR_ZERO);

	/* Action Qualifier */
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM3_BASE_ADDR, EPWM_GL_REGISTER_AQCSFRC);
	EPWM_setActionQualifierContSWForceShadowMode(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM3_BASE_ADDR, EPWM_GL_REGISTER_AQCTLA_AQCTLA2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM3_BASE_ADDR, EPWM_ACTION_QUALIFIER_A);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM3_BASE_ADDR, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierT1TriggerSource(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierT2TriggerSource(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM3_BASE_ADDR, EPWM_GL_REGISTER_AQCTLB_AQCTLB2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM3_BASE_ADDR, EPWM_ACTION_QUALIFIER_B);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM3_BASE_ADDR, EPWM_ACTION_QUALIFIER_B, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);

	/* Events */
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

	/* Trip Zone */
    EPWM_setTripZoneAction(CONFIG_EPWM3_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM3_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM3_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM3_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM3_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM3_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_disableTripZoneAdvAction(CONFIG_EPWM3_BASE_ADDR);
	EPWM_enableTripZoneSignals(CONFIG_EPWM3_BASE_ADDR, 0);
	EPWM_enableTripZoneSignals(CONFIG_EPWM3_BASE_ADDR, 0);
	EPWM_selectCycleByCycleTripZoneClearEvent(CONFIG_EPWM3_BASE_ADDR, EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO);
	EPWM_enableTripZoneInterrupt(CONFIG_EPWM3_BASE_ADDR, 0);

	/* Digital Compare */
	EPWM_setDigitalCompareFilterInput(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_WINDOW_SOURCE_DCAEVT1);
	EPWM_disableDigitalCompareBlankingWindow(CONFIG_EPWM3_BASE_ADDR);
    EPWM_setDigitalCompareBlankingEvent(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_WINDOW_START_TBCTR_PERIOD, 0);
	EPWM_setDigitalCompareWindowOffset(CONFIG_EPWM3_BASE_ADDR, 0);
	EPWM_setDigitalCompareWindowLength(CONFIG_EPWM3_BASE_ADDR, 0);
	EPWM_disableDigitalCompareWindowInverseMode(CONFIG_EPWM3_BASE_ADDR);
	EPWM_disableDigitalCompareCounterCapture(CONFIG_EPWM3_BASE_ADDR);
	EPWM_setDigitalCompareCounterShadowMode(CONFIG_EPWM3_BASE_ADDR, false);
	EPWM_disableDigitalCompareEdgeFilter(CONFIG_EPWM3_BASE_ADDR);
	EPWM_setDigitalCompareEdgeFilterMode(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_EDGEFILT_MODE_RISING);
	EPWM_setDigitalCompareEdgeFilterEdgeCount(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_EDGEFILT_EDGECNT_0);
	EPWM_disableValleyCapture(CONFIG_EPWM3_BASE_ADDR);
	EPWM_setValleyTriggerSource(CONFIG_EPWM3_BASE_ADDR, EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE);
	
	EPWM_setValleyTriggerEdgeCounts(CONFIG_EPWM3_BASE_ADDR, 0, 0);
	EPWM_disableValleyHWDelay(CONFIG_EPWM3_BASE_ADDR);
	EPWM_setValleySWDelayValue(CONFIG_EPWM3_BASE_ADDR, 0);
	EPWM_setValleyDelayDivider(CONFIG_EPWM3_BASE_ADDR, EPWM_VALLEY_DELAY_MODE_SW_DELAY);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM3_BASE_ADDR, 0, EPWM_DC_TYPE_DCAH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM3_BASE_ADDR, 0, EPWM_DC_TYPE_DCAL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM3_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM3_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A2, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_MODULE_A);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM3_BASE_ADDR, 0, EPWM_DC_TYPE_DCBH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM3_BASE_ADDR, 0, EPWM_DC_TYPE_DCBL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM3_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM3_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B2, EPWM_TZ_EVENT_DC_DISABLED);

	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_MODULE_B);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);

	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM3_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);

	/* Deadband */
    EPWM_disableDeadBandControlShadowLoadMode(CONFIG_EPWM3_BASE_ADDR);
    EPWM_setDeadBandControlShadowLoadMode(CONFIG_EPWM3_BASE_ADDR, EPWM_DB_LOAD_ON_CNTR_ZERO);
	EPWM_setRisingEdgeDeadBandDelayInput(CONFIG_EPWM3_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setFallingEdgeDeadBandDelayInput(CONFIG_EPWM3_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM3_BASE_ADDR, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM3_BASE_ADDR, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM3_BASE_ADDR, EPWM_DB_RED, false);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM3_BASE_ADDR, EPWM_DB_FED, true);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM3_BASE_ADDR, EPWM_DB_OUTPUT_A, false);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM3_BASE_ADDR, EPWM_DB_OUTPUT_B, false);
	
	
	EPWM_disableRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM3_BASE_ADDR);
	EPWM_setRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM3_BASE_ADDR, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_setRisingEdgeDelayCount(CONFIG_EPWM3_BASE_ADDR, 0);
	
	EPWM_disableFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM3_BASE_ADDR);
	EPWM_setFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM3_BASE_ADDR, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_setFallingEdgeDelayCount(CONFIG_EPWM3_BASE_ADDR, 200);
	EPWM_setDeadBandCounterClock(CONFIG_EPWM3_BASE_ADDR, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

	/* Chopper */
	EPWM_disableChopper(CONFIG_EPWM3_BASE_ADDR);
	EPWM_setChopperDutyCycle(CONFIG_EPWM3_BASE_ADDR, 0);
	EPWM_setChopperFreq(CONFIG_EPWM3_BASE_ADDR, 0);
	EPWM_setChopperFirstPulseWidth(CONFIG_EPWM3_BASE_ADDR, 0);

	/* Event Trigger */
	EPWM_disableInterrupt(CONFIG_EPWM3_BASE_ADDR);
	EPWM_setInterruptSource(CONFIG_EPWM3_BASE_ADDR, EPWM_INT_TBCTR_ZERO, 0);
	EPWM_setInterruptEventCount(CONFIG_EPWM3_BASE_ADDR, 0);
	EPWM_disableInterruptEventCountInit(CONFIG_EPWM3_BASE_ADDR);
	EPWM_setInterruptEventCountInitValue(CONFIG_EPWM3_BASE_ADDR, 0);
	
	EPWM_disableADCTrigger(CONFIG_EPWM3_BASE_ADDR, EPWM_SOC_A);
    EPWM_setADCTriggerSource(CONFIG_EPWM3_BASE_ADDR, EPWM_SOC_A, EPWM_SOC_DCxEVT1, 0);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM3_BASE_ADDR, EPWM_SOC_A, 0);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM3_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM3_BASE_ADDR, EPWM_SOC_A, 0);
	
	EPWM_disableADCTrigger(CONFIG_EPWM3_BASE_ADDR, EPWM_SOC_B);
    EPWM_setADCTriggerSource(CONFIG_EPWM3_BASE_ADDR, EPWM_SOC_B, EPWM_SOC_DCxEVT1, EPWM_SOC_DCxEVT1);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM3_BASE_ADDR, EPWM_SOC_B, 0);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM3_BASE_ADDR, EPWM_SOC_B);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM3_BASE_ADDR, EPWM_SOC_B, 0);
	

    /* XCMP Mode */
    EPWM_disableXCMPMode(CONFIG_EPWM3_BASE_ADDR);
    EPWM_disableSplitXCMP(CONFIG_EPWM3_BASE_ADDR);
	EPWM_allocAXCMP(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_NONE_CMPA);
    EPWM_setXCMPLoadMode(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
    EPWM_setXCMPShadowLevel(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWLEVEL_0);
    EPWM_setXCMPShadowBufPtrLoadOnce(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW2, 0);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW3, 0);

    /* Write values to Reg */
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP1_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP2_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP3_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP4_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP5_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP6_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP7_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP8_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP1_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP2_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP3_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP4_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP5_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP6_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP7_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP8_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP1_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP2_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP3_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP4_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP5_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP6_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP7_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP8_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP1_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP2_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP3_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP4_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP5_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP6_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP7_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP8_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XTBPRD_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XTBPRD_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XTBPRD_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XTBPRD_SHADOW3, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XMIN_ACTIVE, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XMAX_ACTIVE, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XMIN_SHADOW1, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XMAX_SHADOW1, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XMIN_SHADOW2, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XMAX_SHADOW2, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XMIN_SHADOW3, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM3_BASE_ADDR, EPWM_XMAX_SHADOW3, 0);

    /* Events */

    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM3_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);

    /* Diode Emulation */
    EPWM_disableDiodeEmulationMode(CONFIG_EPWM3_BASE_ADDR);
    EPWM_setDiodeEmulationMode(CONFIG_EPWM3_BASE_ADDR, EPWM_DIODE_EMULATION_CBC);
    EPWM_setDiodeEmulationReentryDelay(CONFIG_EPWM3_BASE_ADDR, 0);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM3_BASE_ADDR, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0, EPWM_DE_TRIPL);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM3_BASE_ADDR, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0, EPWM_DE_TRIPH);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM3_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM3_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_TRIPL);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM3_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM3_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_TRIPL);
    EPWM_nobypassDiodeEmulationLogic(CONFIG_EPWM3_BASE_ADDR);
    
    EPWM_disableDiodeEmulationMonitorModeControl(CONFIG_EPWM3_BASE_ADDR);
    EPWM_setDiodeEmulationMonitorCounterThreshold(CONFIG_EPWM3_BASE_ADDR, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM3_BASE_ADDR, EPWM_DE_COUNT_DOWN, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM3_BASE_ADDR, EPWM_DE_COUNT_UP, 0);

    /* HRPWM */
    HRPWM_disableAutoConversion(CONFIG_EPWM3_BASE_ADDR);
    HRPWM_setMEPControlMode(CONFIG_EPWM3_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setMEPControlMode(CONFIG_EPWM3_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setHiResPhaseShift(CONFIG_EPWM3_BASE_ADDR, 0);
    HRPWM_setSyncPulseSource(CONFIG_EPWM3_BASE_ADDR, HRPWM_PWMSYNC_SOURCE_PERIOD);
    HRPWM_disablePhaseShiftLoad(CONFIG_EPWM3_BASE_ADDR);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM3_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM3_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM3_BASE_ADDR, HRPWM_COUNTER_COMPARE_A, 1);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM3_BASE_ADDR, HRPWM_COUNTER_COMPARE_B, 1);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM3_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM3_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_disablePeriodControl(CONFIG_EPWM3_BASE_ADDR);
    HRPWM_setHiResTimeBasePeriod(CONFIG_EPWM3_BASE_ADDR, 0);
    HRPWM_setDeadbandMEPEdgeSelect(CONFIG_EPWM3_BASE_ADDR, HRPWM_DB_MEP_CTRL_DISABLE);
    HRPWM_setHiResRisingEdgeDelay(CONFIG_EPWM3_BASE_ADDR, 0);
    HRPWM_setRisingEdgeDelayLoadMode(CONFIG_EPWM3_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setHiResFallingEdgeDelayOnly(CONFIG_EPWM3_BASE_ADDR, 0);
    HRPWM_setFallingEdgeDelayLoadMode(CONFIG_EPWM3_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setOutputSwapMode(CONFIG_EPWM3_BASE_ADDR, false);
    HRPWM_setChannelBOutputPath(CONFIG_EPWM3_BASE_ADDR, HRPWM_OUTPUT_ON_B_NORMAL);

    /* ICL */
    EPWM_disableIllegalComboLogic(CONFIG_EPWM3_BASE_ADDR, EPWM_MINDB_BLOCK_A);
    EPWM_disableIllegalComboLogic(CONFIG_EPWM3_BASE_ADDR, EPWM_MINDB_BLOCK_B);

    /* MDL */
    EPWM_disableMinimumDeadBand(CONFIG_EPWM3_BASE_ADDR, EPWM_MINDB_BLOCK_A);
    EPWM_disableMinimumDeadBand(CONFIG_EPWM3_BASE_ADDR, EPWM_MINDB_BLOCK_B);
	/* Global Load */
	EPWM_disableGlobalLoad(CONFIG_EPWM3_BASE_ADDR);
	EPWM_setGlobalLoadTrigger(CONFIG_EPWM3_BASE_ADDR, EPWM_GL_LOAD_PULSE_CNTR_ZERO);
	EPWM_setGlobalLoadEventPrescale(CONFIG_EPWM3_BASE_ADDR, 0);
	EPWM_disableGlobalLoadOneShotMode(CONFIG_EPWM3_BASE_ADDR);
	
	

	/* EPWM Module */
	EPWM_lockRegisters(CONFIG_EPWM3_BASE_ADDR, 0);
	/* CONFIG_EPWM4 initialization */

	/* Time Base */
	EPWM_setEmulationMode(CONFIG_EPWM4_BASE_ADDR, EPWM_EMULATION_FREE_RUN);
	EPWM_setClockPrescaler(CONFIG_EPWM4_BASE_ADDR, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
	EPWM_setTimeBasePeriod(CONFIG_EPWM4_BASE_ADDR, 2000);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM4_BASE_ADDR, EPWM_GL_REGISTER_TBPRD_TBPRDHR);
	EPWM_setPeriodLoadMode(CONFIG_EPWM4_BASE_ADDR, EPWM_PERIOD_SHADOW_LOAD);
	EPWM_setTimeBaseCounter(CONFIG_EPWM4_BASE_ADDR, 0);
	EPWM_setTimeBaseCounterMode(CONFIG_EPWM4_BASE_ADDR, EPWM_COUNTER_MODE_STOP_FREEZE);
	EPWM_setCountModeAfterSync(CONFIG_EPWM4_BASE_ADDR, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
	EPWM_disablePhaseShiftLoad(CONFIG_EPWM4_BASE_ADDR);
	EPWM_setPhaseShift(CONFIG_EPWM4_BASE_ADDR, 0);
	EPWM_enableSyncOutPulseSource(CONFIG_EPWM4_BASE_ADDR, 0);
	EPWM_setSyncInPulseSource(CONFIG_EPWM4_BASE_ADDR, EPWM_SYNC_IN_PULSE_SRC_DISABLE);
	EPWM_setOneShotSyncOutTrigger(CONFIG_EPWM4_BASE_ADDR, EPWM_OSHT_SYNC_OUT_TRIG_SYNC);
	

	/* Counter Compare */
	EPWM_setCounterCompareValue(CONFIG_EPWM4_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM4_BASE_ADDR, EPWM_GL_REGISTER_CMPA_CMPAHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM4_BASE_ADDR, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM4_BASE_ADDR, EPWM_COUNTER_COMPARE_B, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM4_BASE_ADDR, EPWM_GL_REGISTER_CMPB_CMPBHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM4_BASE_ADDR, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM4_BASE_ADDR, EPWM_COUNTER_COMPARE_C, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM4_BASE_ADDR, EPWM_GL_REGISTER_CMPC);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM4_BASE_ADDR, EPWM_COUNTER_COMPARE_C, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM4_BASE_ADDR, EPWM_COUNTER_COMPARE_D, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM4_BASE_ADDR, EPWM_GL_REGISTER_CMPD);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM4_BASE_ADDR, EPWM_COUNTER_COMPARE_D, EPWM_COMP_LOAD_ON_CNTR_ZERO);

	/* Action Qualifier */
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM4_BASE_ADDR, EPWM_GL_REGISTER_AQCSFRC);
	EPWM_setActionQualifierContSWForceShadowMode(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM4_BASE_ADDR, EPWM_GL_REGISTER_AQCTLA_AQCTLA2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM4_BASE_ADDR, EPWM_ACTION_QUALIFIER_A);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM4_BASE_ADDR, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierT1TriggerSource(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierT2TriggerSource(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM4_BASE_ADDR, EPWM_GL_REGISTER_AQCTLB_AQCTLB2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM4_BASE_ADDR, EPWM_ACTION_QUALIFIER_B);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM4_BASE_ADDR, EPWM_ACTION_QUALIFIER_B, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);

	/* Events */
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

	/* Trip Zone */
    EPWM_setTripZoneAction(CONFIG_EPWM4_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM4_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM4_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM4_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM4_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM4_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_disableTripZoneAdvAction(CONFIG_EPWM4_BASE_ADDR);
	EPWM_enableTripZoneSignals(CONFIG_EPWM4_BASE_ADDR, 0);
	EPWM_enableTripZoneSignals(CONFIG_EPWM4_BASE_ADDR, 0);
	EPWM_selectCycleByCycleTripZoneClearEvent(CONFIG_EPWM4_BASE_ADDR, EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO);
	EPWM_enableTripZoneInterrupt(CONFIG_EPWM4_BASE_ADDR, 0);

	/* Digital Compare */
	EPWM_setDigitalCompareFilterInput(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_WINDOW_SOURCE_DCAEVT1);
	EPWM_disableDigitalCompareBlankingWindow(CONFIG_EPWM4_BASE_ADDR);
    EPWM_setDigitalCompareBlankingEvent(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_WINDOW_START_TBCTR_PERIOD, 0);
	EPWM_setDigitalCompareWindowOffset(CONFIG_EPWM4_BASE_ADDR, 0);
	EPWM_setDigitalCompareWindowLength(CONFIG_EPWM4_BASE_ADDR, 0);
	EPWM_disableDigitalCompareWindowInverseMode(CONFIG_EPWM4_BASE_ADDR);
	EPWM_disableDigitalCompareCounterCapture(CONFIG_EPWM4_BASE_ADDR);
	EPWM_setDigitalCompareCounterShadowMode(CONFIG_EPWM4_BASE_ADDR, false);
	EPWM_disableDigitalCompareEdgeFilter(CONFIG_EPWM4_BASE_ADDR);
	EPWM_setDigitalCompareEdgeFilterMode(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_EDGEFILT_MODE_RISING);
	EPWM_setDigitalCompareEdgeFilterEdgeCount(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_EDGEFILT_EDGECNT_0);
	EPWM_disableValleyCapture(CONFIG_EPWM4_BASE_ADDR);
	EPWM_setValleyTriggerSource(CONFIG_EPWM4_BASE_ADDR, EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE);
	
	EPWM_setValleyTriggerEdgeCounts(CONFIG_EPWM4_BASE_ADDR, 0, 0);
	EPWM_disableValleyHWDelay(CONFIG_EPWM4_BASE_ADDR);
	EPWM_setValleySWDelayValue(CONFIG_EPWM4_BASE_ADDR, 0);
	EPWM_setValleyDelayDivider(CONFIG_EPWM4_BASE_ADDR, EPWM_VALLEY_DELAY_MODE_SW_DELAY);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM4_BASE_ADDR, 0, EPWM_DC_TYPE_DCAH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM4_BASE_ADDR, 0, EPWM_DC_TYPE_DCAL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM4_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM4_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A2, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_MODULE_A);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM4_BASE_ADDR, 0, EPWM_DC_TYPE_DCBH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM4_BASE_ADDR, 0, EPWM_DC_TYPE_DCBL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM4_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM4_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B2, EPWM_TZ_EVENT_DC_DISABLED);

	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_MODULE_B);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);

	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM4_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);

	/* Deadband */
    EPWM_disableDeadBandControlShadowLoadMode(CONFIG_EPWM4_BASE_ADDR);
    EPWM_setDeadBandControlShadowLoadMode(CONFIG_EPWM4_BASE_ADDR, EPWM_DB_LOAD_ON_CNTR_ZERO);
	EPWM_setRisingEdgeDeadBandDelayInput(CONFIG_EPWM4_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setFallingEdgeDeadBandDelayInput(CONFIG_EPWM4_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM4_BASE_ADDR, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM4_BASE_ADDR, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM4_BASE_ADDR, EPWM_DB_RED, false);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM4_BASE_ADDR, EPWM_DB_FED, false);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM4_BASE_ADDR, EPWM_DB_OUTPUT_A, false);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM4_BASE_ADDR, EPWM_DB_OUTPUT_B, false);
	
	
	EPWM_disableRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM4_BASE_ADDR);
	EPWM_setRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM4_BASE_ADDR, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_setRisingEdgeDelayCount(CONFIG_EPWM4_BASE_ADDR, 0);
	
	EPWM_disableFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM4_BASE_ADDR);
	EPWM_setFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM4_BASE_ADDR, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_setFallingEdgeDelayCount(CONFIG_EPWM4_BASE_ADDR, 0);
	EPWM_setDeadBandCounterClock(CONFIG_EPWM4_BASE_ADDR, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

	/* Chopper */
	EPWM_disableChopper(CONFIG_EPWM4_BASE_ADDR);
	EPWM_setChopperDutyCycle(CONFIG_EPWM4_BASE_ADDR, 0);
	EPWM_setChopperFreq(CONFIG_EPWM4_BASE_ADDR, 0);
	EPWM_setChopperFirstPulseWidth(CONFIG_EPWM4_BASE_ADDR, 0);

	/* Event Trigger */
	EPWM_disableInterrupt(CONFIG_EPWM4_BASE_ADDR);
	EPWM_setInterruptSource(CONFIG_EPWM4_BASE_ADDR, EPWM_INT_TBCTR_ZERO, 0);
	EPWM_setInterruptEventCount(CONFIG_EPWM4_BASE_ADDR, 0);
	EPWM_disableInterruptEventCountInit(CONFIG_EPWM4_BASE_ADDR);
	EPWM_setInterruptEventCountInitValue(CONFIG_EPWM4_BASE_ADDR, 0);
	
	EPWM_disableADCTrigger(CONFIG_EPWM4_BASE_ADDR, EPWM_SOC_A);
    EPWM_setADCTriggerSource(CONFIG_EPWM4_BASE_ADDR, EPWM_SOC_A, EPWM_SOC_DCxEVT1, 0);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM4_BASE_ADDR, EPWM_SOC_A, 0);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM4_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM4_BASE_ADDR, EPWM_SOC_A, 0);
	
	EPWM_disableADCTrigger(CONFIG_EPWM4_BASE_ADDR, EPWM_SOC_B);
    EPWM_setADCTriggerSource(CONFIG_EPWM4_BASE_ADDR, EPWM_SOC_B, EPWM_SOC_DCxEVT1, EPWM_SOC_DCxEVT1);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM4_BASE_ADDR, EPWM_SOC_B, 0);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM4_BASE_ADDR, EPWM_SOC_B);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM4_BASE_ADDR, EPWM_SOC_B, 0);
	

    /* XCMP Mode */
    EPWM_disableXCMPMode(CONFIG_EPWM4_BASE_ADDR);
    EPWM_disableSplitXCMP(CONFIG_EPWM4_BASE_ADDR);
	EPWM_allocAXCMP(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_NONE_CMPA);
    EPWM_setXCMPLoadMode(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
    EPWM_setXCMPShadowLevel(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWLEVEL_0);
    EPWM_setXCMPShadowBufPtrLoadOnce(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW2, 0);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW3, 0);

    /* Write values to Reg */
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP1_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP2_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP3_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP4_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP5_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP6_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP7_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP8_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP1_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP2_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP3_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP4_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP5_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP6_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP7_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP8_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP1_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP2_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP3_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP4_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP5_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP6_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP7_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP8_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP1_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP2_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP3_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP4_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP5_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP6_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP7_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP8_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XTBPRD_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XTBPRD_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XTBPRD_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XTBPRD_SHADOW3, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XMIN_ACTIVE, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XMAX_ACTIVE, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XMIN_SHADOW1, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XMAX_SHADOW1, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XMIN_SHADOW2, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XMAX_SHADOW2, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XMIN_SHADOW3, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM4_BASE_ADDR, EPWM_XMAX_SHADOW3, 0);

    /* Events */

    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM4_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);

    /* Diode Emulation */
    EPWM_disableDiodeEmulationMode(CONFIG_EPWM4_BASE_ADDR);
    EPWM_setDiodeEmulationMode(CONFIG_EPWM4_BASE_ADDR, EPWM_DIODE_EMULATION_CBC);
    EPWM_setDiodeEmulationReentryDelay(CONFIG_EPWM4_BASE_ADDR, 0);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM4_BASE_ADDR, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0, EPWM_DE_TRIPL);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM4_BASE_ADDR, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0, EPWM_DE_TRIPH);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM4_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM4_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_TRIPL);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM4_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM4_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_TRIPL);
    EPWM_nobypassDiodeEmulationLogic(CONFIG_EPWM4_BASE_ADDR);
    
    EPWM_disableDiodeEmulationMonitorModeControl(CONFIG_EPWM4_BASE_ADDR);
    EPWM_setDiodeEmulationMonitorCounterThreshold(CONFIG_EPWM4_BASE_ADDR, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM4_BASE_ADDR, EPWM_DE_COUNT_DOWN, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM4_BASE_ADDR, EPWM_DE_COUNT_UP, 0);

    /* HRPWM */
    HRPWM_disableAutoConversion(CONFIG_EPWM4_BASE_ADDR);
    HRPWM_setMEPControlMode(CONFIG_EPWM4_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setMEPControlMode(CONFIG_EPWM4_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setHiResPhaseShift(CONFIG_EPWM4_BASE_ADDR, 0);
    HRPWM_setSyncPulseSource(CONFIG_EPWM4_BASE_ADDR, HRPWM_PWMSYNC_SOURCE_PERIOD);
    HRPWM_disablePhaseShiftLoad(CONFIG_EPWM4_BASE_ADDR);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM4_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM4_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM4_BASE_ADDR, HRPWM_COUNTER_COMPARE_A, 1);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM4_BASE_ADDR, HRPWM_COUNTER_COMPARE_B, 1);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM4_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM4_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_disablePeriodControl(CONFIG_EPWM4_BASE_ADDR);
    HRPWM_setHiResTimeBasePeriod(CONFIG_EPWM4_BASE_ADDR, 0);
    HRPWM_setDeadbandMEPEdgeSelect(CONFIG_EPWM4_BASE_ADDR, HRPWM_DB_MEP_CTRL_DISABLE);
    HRPWM_setHiResRisingEdgeDelay(CONFIG_EPWM4_BASE_ADDR, 0);
    HRPWM_setRisingEdgeDelayLoadMode(CONFIG_EPWM4_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setHiResFallingEdgeDelayOnly(CONFIG_EPWM4_BASE_ADDR, 0);
    HRPWM_setFallingEdgeDelayLoadMode(CONFIG_EPWM4_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setOutputSwapMode(CONFIG_EPWM4_BASE_ADDR, false);
    HRPWM_setChannelBOutputPath(CONFIG_EPWM4_BASE_ADDR, HRPWM_OUTPUT_ON_B_NORMAL);

    /* ICL */
    EPWM_disableIllegalComboLogic(CONFIG_EPWM4_BASE_ADDR, EPWM_MINDB_BLOCK_A);
    EPWM_disableIllegalComboLogic(CONFIG_EPWM4_BASE_ADDR, EPWM_MINDB_BLOCK_B);

    /* MDL */
    EPWM_disableMinimumDeadBand(CONFIG_EPWM4_BASE_ADDR, EPWM_MINDB_BLOCK_A);
    EPWM_disableMinimumDeadBand(CONFIG_EPWM4_BASE_ADDR, EPWM_MINDB_BLOCK_B);
	/* Global Load */
	EPWM_disableGlobalLoad(CONFIG_EPWM4_BASE_ADDR);
	EPWM_setGlobalLoadTrigger(CONFIG_EPWM4_BASE_ADDR, EPWM_GL_LOAD_PULSE_CNTR_ZERO);
	EPWM_setGlobalLoadEventPrescale(CONFIG_EPWM4_BASE_ADDR, 0);
	EPWM_disableGlobalLoadOneShotMode(CONFIG_EPWM4_BASE_ADDR);
	
	

	/* EPWM Module */
	EPWM_lockRegisters(CONFIG_EPWM4_BASE_ADDR, 0);
	/* CONFIG_EPWM5 initialization */

	/* Time Base */
	EPWM_setEmulationMode(CONFIG_EPWM5_BASE_ADDR, EPWM_EMULATION_FREE_RUN);
	EPWM_setClockPrescaler(CONFIG_EPWM5_BASE_ADDR, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
	EPWM_setTimeBasePeriod(CONFIG_EPWM5_BASE_ADDR, 2000);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM5_BASE_ADDR, EPWM_GL_REGISTER_TBPRD_TBPRDHR);
	EPWM_setPeriodLoadMode(CONFIG_EPWM5_BASE_ADDR, EPWM_PERIOD_SHADOW_LOAD);
	EPWM_setTimeBaseCounter(CONFIG_EPWM5_BASE_ADDR, 0);
	EPWM_setTimeBaseCounterMode(CONFIG_EPWM5_BASE_ADDR, EPWM_COUNTER_MODE_STOP_FREEZE);
	EPWM_setCountModeAfterSync(CONFIG_EPWM5_BASE_ADDR, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
	EPWM_disablePhaseShiftLoad(CONFIG_EPWM5_BASE_ADDR);
	EPWM_setPhaseShift(CONFIG_EPWM5_BASE_ADDR, 0);
	EPWM_enableSyncOutPulseSource(CONFIG_EPWM5_BASE_ADDR, 0);
	EPWM_setSyncInPulseSource(CONFIG_EPWM5_BASE_ADDR, EPWM_SYNC_IN_PULSE_SRC_DISABLE);
	EPWM_setOneShotSyncOutTrigger(CONFIG_EPWM5_BASE_ADDR, EPWM_OSHT_SYNC_OUT_TRIG_SYNC);
	

	/* Counter Compare */
	EPWM_setCounterCompareValue(CONFIG_EPWM5_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM5_BASE_ADDR, EPWM_GL_REGISTER_CMPA_CMPAHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM5_BASE_ADDR, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM5_BASE_ADDR, EPWM_COUNTER_COMPARE_B, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM5_BASE_ADDR, EPWM_GL_REGISTER_CMPB_CMPBHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM5_BASE_ADDR, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM5_BASE_ADDR, EPWM_COUNTER_COMPARE_C, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM5_BASE_ADDR, EPWM_GL_REGISTER_CMPC);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM5_BASE_ADDR, EPWM_COUNTER_COMPARE_C, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM5_BASE_ADDR, EPWM_COUNTER_COMPARE_D, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM5_BASE_ADDR, EPWM_GL_REGISTER_CMPD);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM5_BASE_ADDR, EPWM_COUNTER_COMPARE_D, EPWM_COMP_LOAD_ON_CNTR_ZERO);

	/* Action Qualifier */
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM5_BASE_ADDR, EPWM_GL_REGISTER_AQCSFRC);
	EPWM_setActionQualifierContSWForceShadowMode(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM5_BASE_ADDR, EPWM_GL_REGISTER_AQCTLA_AQCTLA2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM5_BASE_ADDR, EPWM_ACTION_QUALIFIER_A);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM5_BASE_ADDR, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierT1TriggerSource(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierT2TriggerSource(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM5_BASE_ADDR, EPWM_GL_REGISTER_AQCTLB_AQCTLB2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM5_BASE_ADDR, EPWM_ACTION_QUALIFIER_B);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM5_BASE_ADDR, EPWM_ACTION_QUALIFIER_B, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);

	/* Events */
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

	/* Trip Zone */
    EPWM_setTripZoneAction(CONFIG_EPWM5_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM5_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM5_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM5_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM5_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(CONFIG_EPWM5_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_disableTripZoneAdvAction(CONFIG_EPWM5_BASE_ADDR);
	EPWM_enableTripZoneSignals(CONFIG_EPWM5_BASE_ADDR, 0);
	EPWM_enableTripZoneSignals(CONFIG_EPWM5_BASE_ADDR, 0);
	EPWM_selectCycleByCycleTripZoneClearEvent(CONFIG_EPWM5_BASE_ADDR, EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO);
	EPWM_enableTripZoneInterrupt(CONFIG_EPWM5_BASE_ADDR, 0);

	/* Digital Compare */
	EPWM_setDigitalCompareFilterInput(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_WINDOW_SOURCE_DCAEVT1);
	EPWM_disableDigitalCompareBlankingWindow(CONFIG_EPWM5_BASE_ADDR);
    EPWM_setDigitalCompareBlankingEvent(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_WINDOW_START_TBCTR_PERIOD, 0);
	EPWM_setDigitalCompareWindowOffset(CONFIG_EPWM5_BASE_ADDR, 0);
	EPWM_setDigitalCompareWindowLength(CONFIG_EPWM5_BASE_ADDR, 0);
	EPWM_disableDigitalCompareWindowInverseMode(CONFIG_EPWM5_BASE_ADDR);
	EPWM_disableDigitalCompareCounterCapture(CONFIG_EPWM5_BASE_ADDR);
	EPWM_setDigitalCompareCounterShadowMode(CONFIG_EPWM5_BASE_ADDR, false);
	EPWM_disableDigitalCompareEdgeFilter(CONFIG_EPWM5_BASE_ADDR);
	EPWM_setDigitalCompareEdgeFilterMode(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_EDGEFILT_MODE_RISING);
	EPWM_setDigitalCompareEdgeFilterEdgeCount(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_EDGEFILT_EDGECNT_0);
	EPWM_disableValleyCapture(CONFIG_EPWM5_BASE_ADDR);
	EPWM_setValleyTriggerSource(CONFIG_EPWM5_BASE_ADDR, EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE);
	
	EPWM_setValleyTriggerEdgeCounts(CONFIG_EPWM5_BASE_ADDR, 0, 0);
	EPWM_disableValleyHWDelay(CONFIG_EPWM5_BASE_ADDR);
	EPWM_setValleySWDelayValue(CONFIG_EPWM5_BASE_ADDR, 0);
	EPWM_setValleyDelayDivider(CONFIG_EPWM5_BASE_ADDR, EPWM_VALLEY_DELAY_MODE_SW_DELAY);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM5_BASE_ADDR, 0, EPWM_DC_TYPE_DCAH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM5_BASE_ADDR, 0, EPWM_DC_TYPE_DCAL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM5_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM5_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A2, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_MODULE_A);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM5_BASE_ADDR, 0, EPWM_DC_TYPE_DCBH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM5_BASE_ADDR, 0, EPWM_DC_TYPE_DCBL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM5_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM5_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B2, EPWM_TZ_EVENT_DC_DISABLED);

	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_MODULE_B);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);

	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM5_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);

	/* Deadband */
    EPWM_disableDeadBandControlShadowLoadMode(CONFIG_EPWM5_BASE_ADDR);
    EPWM_setDeadBandControlShadowLoadMode(CONFIG_EPWM5_BASE_ADDR, EPWM_DB_LOAD_ON_CNTR_ZERO);
	EPWM_setRisingEdgeDeadBandDelayInput(CONFIG_EPWM5_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setFallingEdgeDeadBandDelayInput(CONFIG_EPWM5_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM5_BASE_ADDR, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM5_BASE_ADDR, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM5_BASE_ADDR, EPWM_DB_RED, false);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM5_BASE_ADDR, EPWM_DB_FED, false);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM5_BASE_ADDR, EPWM_DB_OUTPUT_A, false);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM5_BASE_ADDR, EPWM_DB_OUTPUT_B, false);
	
	
	EPWM_disableRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM5_BASE_ADDR);
	EPWM_setRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM5_BASE_ADDR, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_setRisingEdgeDelayCount(CONFIG_EPWM5_BASE_ADDR, 0);
	
	EPWM_disableFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM5_BASE_ADDR);
	EPWM_setFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM5_BASE_ADDR, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_setFallingEdgeDelayCount(CONFIG_EPWM5_BASE_ADDR, 0);
	EPWM_setDeadBandCounterClock(CONFIG_EPWM5_BASE_ADDR, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

	/* Chopper */
	EPWM_disableChopper(CONFIG_EPWM5_BASE_ADDR);
	EPWM_setChopperDutyCycle(CONFIG_EPWM5_BASE_ADDR, 0);
	EPWM_setChopperFreq(CONFIG_EPWM5_BASE_ADDR, 0);
	EPWM_setChopperFirstPulseWidth(CONFIG_EPWM5_BASE_ADDR, 0);

	/* Event Trigger */
	EPWM_disableInterrupt(CONFIG_EPWM5_BASE_ADDR);
	EPWM_setInterruptSource(CONFIG_EPWM5_BASE_ADDR, EPWM_INT_TBCTR_ZERO, 0);
	EPWM_setInterruptEventCount(CONFIG_EPWM5_BASE_ADDR, 0);
	EPWM_disableInterruptEventCountInit(CONFIG_EPWM5_BASE_ADDR);
	EPWM_setInterruptEventCountInitValue(CONFIG_EPWM5_BASE_ADDR, 0);
	
	EPWM_disableADCTrigger(CONFIG_EPWM5_BASE_ADDR, EPWM_SOC_A);
    EPWM_setADCTriggerSource(CONFIG_EPWM5_BASE_ADDR, EPWM_SOC_A, EPWM_SOC_DCxEVT1, 0);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM5_BASE_ADDR, EPWM_SOC_A, 0);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM5_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM5_BASE_ADDR, EPWM_SOC_A, 0);
	
	EPWM_disableADCTrigger(CONFIG_EPWM5_BASE_ADDR, EPWM_SOC_B);
    EPWM_setADCTriggerSource(CONFIG_EPWM5_BASE_ADDR, EPWM_SOC_B, EPWM_SOC_DCxEVT1, EPWM_SOC_DCxEVT1);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM5_BASE_ADDR, EPWM_SOC_B, 0);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM5_BASE_ADDR, EPWM_SOC_B);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM5_BASE_ADDR, EPWM_SOC_B, 0);
	

    /* XCMP Mode */
    EPWM_disableXCMPMode(CONFIG_EPWM5_BASE_ADDR);
    EPWM_disableSplitXCMP(CONFIG_EPWM5_BASE_ADDR);
	EPWM_allocAXCMP(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_NONE_CMPA);
    EPWM_setXCMPLoadMode(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
    EPWM_setXCMPShadowLevel(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWLEVEL_0);
    EPWM_setXCMPShadowBufPtrLoadOnce(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW2, 0);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW3, 0);

    /* Write values to Reg */
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP1_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP2_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP3_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP4_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP5_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP6_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP7_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP8_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP1_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP2_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP3_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP4_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP5_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP6_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP7_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP8_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP1_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP2_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP3_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP4_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP5_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP6_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP7_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP8_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP1_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP2_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP3_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP4_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP5_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP6_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP7_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP8_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XTBPRD_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XTBPRD_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XTBPRD_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XTBPRD_SHADOW3, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XMIN_ACTIVE, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XMAX_ACTIVE, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XMIN_SHADOW1, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XMAX_SHADOW1, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XMIN_SHADOW2, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XMAX_SHADOW2, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XMIN_SHADOW3, 0);
    EPWM_setXMINMAXRegValue(CONFIG_EPWM5_BASE_ADDR, EPWM_XMAX_SHADOW3, 0);

    /* Events */

    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM5_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);

    /* Diode Emulation */
    EPWM_disableDiodeEmulationMode(CONFIG_EPWM5_BASE_ADDR);
    EPWM_setDiodeEmulationMode(CONFIG_EPWM5_BASE_ADDR, EPWM_DIODE_EMULATION_CBC);
    EPWM_setDiodeEmulationReentryDelay(CONFIG_EPWM5_BASE_ADDR, 0);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM5_BASE_ADDR, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0, EPWM_DE_TRIPL);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM5_BASE_ADDR, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0, EPWM_DE_TRIPH);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM5_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM5_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_TRIPL);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM5_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM5_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_TRIPL);
    EPWM_nobypassDiodeEmulationLogic(CONFIG_EPWM5_BASE_ADDR);
    
    EPWM_disableDiodeEmulationMonitorModeControl(CONFIG_EPWM5_BASE_ADDR);
    EPWM_setDiodeEmulationMonitorCounterThreshold(CONFIG_EPWM5_BASE_ADDR, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM5_BASE_ADDR, EPWM_DE_COUNT_DOWN, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM5_BASE_ADDR, EPWM_DE_COUNT_UP, 0);

    /* HRPWM */
    HRPWM_disableAutoConversion(CONFIG_EPWM5_BASE_ADDR);
    HRPWM_setMEPControlMode(CONFIG_EPWM5_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setMEPControlMode(CONFIG_EPWM5_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setHiResPhaseShift(CONFIG_EPWM5_BASE_ADDR, 0);
    HRPWM_setSyncPulseSource(CONFIG_EPWM5_BASE_ADDR, HRPWM_PWMSYNC_SOURCE_PERIOD);
    HRPWM_disablePhaseShiftLoad(CONFIG_EPWM5_BASE_ADDR);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM5_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM5_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM5_BASE_ADDR, HRPWM_COUNTER_COMPARE_A, 1);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM5_BASE_ADDR, HRPWM_COUNTER_COMPARE_B, 1);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM5_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM5_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_disablePeriodControl(CONFIG_EPWM5_BASE_ADDR);
    HRPWM_setHiResTimeBasePeriod(CONFIG_EPWM5_BASE_ADDR, 0);
    HRPWM_setDeadbandMEPEdgeSelect(CONFIG_EPWM5_BASE_ADDR, HRPWM_DB_MEP_CTRL_DISABLE);
    HRPWM_setHiResRisingEdgeDelay(CONFIG_EPWM5_BASE_ADDR, 0);
    HRPWM_setRisingEdgeDelayLoadMode(CONFIG_EPWM5_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setHiResFallingEdgeDelayOnly(CONFIG_EPWM5_BASE_ADDR, 0);
    HRPWM_setFallingEdgeDelayLoadMode(CONFIG_EPWM5_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setOutputSwapMode(CONFIG_EPWM5_BASE_ADDR, false);
    HRPWM_setChannelBOutputPath(CONFIG_EPWM5_BASE_ADDR, HRPWM_OUTPUT_ON_B_NORMAL);

    /* ICL */
    EPWM_disableIllegalComboLogic(CONFIG_EPWM5_BASE_ADDR, EPWM_MINDB_BLOCK_A);
    EPWM_disableIllegalComboLogic(CONFIG_EPWM5_BASE_ADDR, EPWM_MINDB_BLOCK_B);

    /* MDL */
    EPWM_disableMinimumDeadBand(CONFIG_EPWM5_BASE_ADDR, EPWM_MINDB_BLOCK_A);
    EPWM_disableMinimumDeadBand(CONFIG_EPWM5_BASE_ADDR, EPWM_MINDB_BLOCK_B);
	/* Global Load */
	EPWM_disableGlobalLoad(CONFIG_EPWM5_BASE_ADDR);
	EPWM_setGlobalLoadTrigger(CONFIG_EPWM5_BASE_ADDR, EPWM_GL_LOAD_PULSE_CNTR_ZERO);
	EPWM_setGlobalLoadEventPrescale(CONFIG_EPWM5_BASE_ADDR, 0);
	EPWM_disableGlobalLoadOneShotMode(CONFIG_EPWM5_BASE_ADDR);
	
	

	/* EPWM Module */
	EPWM_lockRegisters(CONFIG_EPWM5_BASE_ADDR, 0);
}

void Drivers_inputXbarOpen()
{
    /* INPUT XBAR */
    SOC_xbarSelectInputXBarInputSource(CSL_CONTROLSS_INPUTXBAR_U_BASE, 0, 0, INPUT_XBAR_GPIO43, 0);
    SOC_xbarSelectInputXBarInputSource(CSL_CONTROLSS_INPUTXBAR_U_BASE, 1, 0, INPUT_XBAR_GPIO45, 0);
    SOC_xbarSelectInputXBarInputSource(CSL_CONTROLSS_INPUTXBAR_U_BASE, 2, 0, INPUT_XBAR_GPIO47, 0);
    SOC_xbarSelectInputXBarInputSource(CSL_CONTROLSS_INPUTXBAR_U_BASE, 3, 0, INPUT_XBAR_GPIO49, 0);
    SOC_xbarSelectInputXBarInputSource(CSL_CONTROLSS_INPUTXBAR_U_BASE, 4, 0, INPUT_XBAR_GPIO51, 0);
    SOC_xbarSelectInputXBarInputSource(CSL_CONTROLSS_INPUTXBAR_U_BASE, 5, 0, INPUT_XBAR_GPIO53, 0);
}
/*
 * UART
 */

/* UART Driver handles */
UART_Handle gUartHandle[CONFIG_UART_NUM_INSTANCES];

#include <drivers/uart/v0/dma/uart_dma.h>

/* UART Driver Parameters */
UART_Params gUartParams[CONFIG_UART_NUM_INSTANCES] =
{
    {
        .baudRate           = 115200,
        .dataLength         = UART_LEN_8,
        .stopBits           = UART_STOPBITS_1,
        .parityType         = UART_PARITY_NONE,
        .readMode           = UART_TRANSFER_MODE_BLOCKING,
        .readReturnMode     = UART_READ_RETURN_MODE_FULL,
        .writeMode          = UART_TRANSFER_MODE_BLOCKING,
        .readCallbackFxn    = NULL,
        .writeCallbackFxn   = NULL,
        .hwFlowControl      = FALSE,
        .hwFlowControlThr   = UART_RXTRIGLVL_16,
        .transferMode       = UART_CONFIG_MODE_INTERRUPT,
        .skipIntrReg         = FALSE,
        .uartDmaIndex = -1,
        .intrNum            = 38U,
        .intrPriority       = 4U,
        .operMode           = UART_OPER_MODE_16X,
        .rxTrigLvl          = UART_RXTRIGLVL_8,
        .txTrigLvl          = UART_TXTRIGLVL_32,
        .rxEvtNum           = 0U,
        .txEvtNum           = 0U,

    },
};

void Drivers_uartOpen(void)
{
    uint32_t instCnt;
    int32_t  status = SystemP_SUCCESS;

    for(instCnt = 0U; instCnt < CONFIG_UART_NUM_INSTANCES; instCnt++)
    {
        gUartHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
    }

    /* Open all instances */
    for(instCnt = 0U; instCnt < CONFIG_UART_NUM_INSTANCES; instCnt++)
    {
        gUartHandle[instCnt] = UART_open(instCnt, &gUartParams[instCnt]);
        if(NULL == gUartHandle[instCnt])
        {
            DebugP_logError("UART open failed for instance %d !!!\r\n", instCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    if(SystemP_FAILURE == status)
    {
        Drivers_uartClose();   /* Exit gracefully */
    }

    return;
}

void Drivers_uartClose(void)
{
    uint32_t instCnt;
    /* Close all instances that are open */
    for(instCnt = 0U; instCnt < CONFIG_UART_NUM_INSTANCES; instCnt++)
    {
        if(gUartHandle[instCnt] != NULL)
        {
            UART_close(gUartHandle[instCnt]);
            gUartHandle[instCnt] = NULL;
        }
    }

    return;
}

