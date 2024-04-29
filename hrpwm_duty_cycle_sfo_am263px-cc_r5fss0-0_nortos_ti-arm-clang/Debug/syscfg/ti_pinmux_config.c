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
#include <drivers/pinmux.h>

static Pinmux_PerCfg_t gPinMuxMainDomainCfg[] = {
            /* SPI1 pin config */
    /* SPI1_CLK -> SPI1_CLK (A10) */
    {
        PIN_SPI1_CLK,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* SPI1 pin config */
    /* SPI1_D0 -> SPI1_D0 (B10) */
    {
        PIN_SPI1_D0,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* SPI1 pin config */
    /* SPI1_D1 -> SPI1_D1 (D9) */
    {
        PIN_SPI1_D1,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },

            /* SPI1_CS0 pin config */
    /* SPI1_CS0 -> SPI1_CS0 (C9) */
    {
        PIN_SPI1_CS0,
        ( PIN_MODE(0) | PIN_PULL_DISABLE )
    },

            /* EPWM0 pin config */
    /* EPWM0_A -> EPWM0_A (B2) */
    {
        PIN_EPWM0_A,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* EPWM0 pin config */
    /* EPWM0_B -> EPWM0_B (B1) */
    {
        PIN_EPWM0_B,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
            /* EPWM9 pin config */
    /* EPWM9_A -> EPWM9_A (G1) */
    {
        PIN_EPWM9_A,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* EPWM9 pin config */
    /* EPWM9_B -> EPWM9_B (J2) */
    {
        PIN_EPWM9_B,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
            /* EPWM8 pin config */
    /* EPWM8_A -> EPWM8_A (G3) */
    {
        PIN_EPWM8_A,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* EPWM8 pin config */
    /* EPWM8_B -> EPWM8_B (H2) */
    {
        PIN_EPWM8_B,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
            /* EPWM7 pin config */
    /* EPWM7_A -> EPWM7_A (F4) */
    {
        PIN_EPWM7_A,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* EPWM7 pin config */
    /* EPWM7_B -> EPWM10_A (G4) */
    {
        PIN_EPWM10_A,
        ( PIN_MODE(10) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
            /* EPWM6 pin config */
    /* EPWM6_A -> EPWM6_A (E1) */
    {
        PIN_EPWM6_A,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* EPWM6 pin config */
    /* EPWM6_B -> EPWM6_B (F3) */
    {
        PIN_EPWM6_B,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
            /* EPWM5 pin config */
    /* EPWM5_A -> EPWM5_A (F2) */
    {
        PIN_EPWM5_A,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* EPWM5 pin config */
    /* EPWM5_B -> EPWM5_B (G2) */
    {
        PIN_EPWM5_B,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
            /* EPWM4 pin config */
    /* EPWM4_A -> EPWM4_A (D1) */
    {
        PIN_EPWM4_A,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* EPWM4 pin config */
    /* EPWM4_B -> EPWM4_B (E4) */
    {
        PIN_EPWM4_B,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
            /* EPWM3 pin config */
    /* EPWM3_A -> EPWM3_A (E2) */
    {
        PIN_EPWM3_A,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* EPWM3 pin config */
    /* EPWM3_B -> EPWM3_B (E3) */
    {
        PIN_EPWM3_B,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
            /* EPWM2 pin config */
    /* EPWM2_A -> EPWM2_A (C2) */
    {
        PIN_EPWM2_A,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* EPWM2 pin config */
    /* EPWM2_B -> EPWM2_B (C1) */
    {
        PIN_EPWM2_B,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
            /* EPWM1 pin config */
    /* EPWM1_A -> EPWM1_A (D3) */
    {
        PIN_EPWM1_A,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* EPWM1 pin config */
    /* EPWM1_B -> EPWM1_B (D2) */
    {
        PIN_EPWM1_B,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },

            /* UART0 pin config */
    /* UART0_RXD -> UART0_RXD (A7) */
    {
        PIN_UART0_RXD,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* UART0 pin config */
    /* UART0_TXD -> UART0_TXD (A6) */
    {
        PIN_UART0_TXD,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },

    {PINMUX_END, PINMUX_END}
};


/*
 * Pinmux
 */


void Pinmux_init(void)
{
    Pinmux_config(gPinMuxMainDomainCfg, PINMUX_DOMAIN_ID_MAIN);

    
}


