/*
 
Copyright (c) 2009-2020 ARM Limited. All rights reserved.

    SPDX-License-Identifier: Apache-2.0

Licensed under the Apache License, Version 2.0 (the License); you may
not use this file except in compliance with the License.
You may obtain a copy of the License at

    www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an AS IS BASIS, WITHOUT
WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

NOTICE: This file has been modified by Nordic Semiconductor ASA.

*/

    .syntax unified
    .arch armv7e-m

    .section .Vectors
    .align 2
    .globl __Vectors
__Vectors:
    .long   __StackTop                  /* Top of Stack */
    .long   Reset_Handler
    .long   NMI_Handler
    .long   HardFault_Handler
    .long   MemoryManagement_Handler
    .long   BusFault_Handler
    .long   UsageFault_Handler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   SVC_Handler
    .long   DebugMon_Handler
    .long   0                           /*Reserved */
    .long   PendSV_Handler
    .long   SysTick_Handler

  /* External Interrupts */
    .long   POWER_CLOCK_IRQHandler
    .long   RADIO_IRQHandler
    .long   UARTE0_UART0_IRQHandler_v
    .long   TWIM0_TWIS0_TWI0_SPIM1_SPIS1_SPI1_IRQHandler_v
    .long   SPIM0_SPIS0_SPI0_IRQHandler_v
    .long   0                           /*Reserved */
    .long   GPIOTE_IRQHandler_v
    .long   SAADC_IRQHandler_v
    .long   TIMER0_IRQHandler_v
    .long   TIMER1_IRQHandler_v
    .long   TIMER2_IRQHandler_v
    .long   RTC0_IRQHandler
    .long   TEMP_IRQHandler_v
    .long   RNG_IRQHandler
    .long   ECB_IRQHandler
    .long   CCM_AAR_IRQHandler
    .long   WDT_IRQHandler_v
    .long   RTC1_IRQHandler_v
    .long   QDEC_IRQHandler_v
    .long   COMP_IRQHandler_v
    .long   SWI0_EGU0_IRQHandler_v
    .long   SWI1_EGU1_IRQHandler_v
    .long   SWI2_IRQHandler_v
    .long   SWI3_IRQHandler_v
    .long   SWI4_IRQHandler
    .long   SWI5_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   PWM0_IRQHandler_v
    .long   PDM_IRQHandler_v
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */


    .size    __Vectors, . - __Vectors

/* Reset Handler */

    .text
    .thumb
    .thumb_func
    .align 1
    .globl    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    .fnstart


/*     Loop to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in
 *      linker script.
 *      __etext: End of code section, i.e., begin of data sections to copy from.
 *      __data_start__/__data_end__: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */

    ldr    r1, =__etext
    ldr    r2, =__data_start__
    ldr    r3, =__data_end__

    subs    r3, r2
    ble     .LC0

.LC1:
    subs    r3, 4
    ldr    r0, [r1,r3]
    str    r0, [r2,r3]
    bgt    .LC1
.LC0:

    LDR     R0, =SystemInit
    BLX     R0
    LDR     R0, =nrf_reloc_vector_table
    BLX     R0
    LDR     R0, =_start
    BX      R0

    .pool
    .cantunwind
    .fnend
    .size   Reset_Handler,.-Reset_Handler

    .section ".text"


/* Dummy Exception Handlers (infinite loops which can be modified) */

    .weak   NMI_Handler
    .type   NMI_Handler, %function
NMI_Handler:
    B       .
    .size   NMI_Handler, . - NMI_Handler


    .weak   HardFault_Handler
    .type   HardFault_Handler, %function
HardFault_Handler:
    B       .
    .size   HardFault_Handler, . - HardFault_Handler


    .weak   MemoryManagement_Handler
    .type   MemoryManagement_Handler, %function
MemoryManagement_Handler:
    B       .
    .size   MemoryManagement_Handler, . - MemoryManagement_Handler


    .weak   BusFault_Handler
    .type   BusFault_Handler, %function
BusFault_Handler:
    B       .
    .size   BusFault_Handler, . - BusFault_Handler


    .weak   UsageFault_Handler
    .type   UsageFault_Handler, %function
UsageFault_Handler:
    B       .
    .size   UsageFault_Handler, . - UsageFault_Handler


    .weak   SVC_Handler
    .type   SVC_Handler, %function
SVC_Handler:
    B       .
    .size   SVC_Handler, . - SVC_Handler


    .weak   DebugMon_Handler
    .type   DebugMon_Handler, %function
DebugMon_Handler:
    b       .
    .size   DebugMon_Handler, . - DebugMon_Handler


    .weak   PendSV_Handler
    .type   PendSV_Handler, %function
PendSV_Handler:
    B       .
    .size   PendSV_Handler, . - PendSV_Handler


    .weak   SysTick_Handler
    .type   SysTick_Handler, %function
SysTick_Handler:
    B       .
    .size   SysTick_Handler, . - SysTick_Handler


/* IRQ Handlers */

    .globl  Default_Handler
    .type   Default_Handler, %function
Default_Handler:
    B       .
    .size   Default_Handler, . - Default_Handler

    .macro  IRQ handler
    .weak   \handler
    .set    \handler, Default_Handler
    .endm

    IRQ  POWER_CLOCK_IRQHandler                          /* restricted */
    IRQ  RADIO_IRQHandler                                /* blocked    */
    IRQ  UARTE0_UART0_IRQHandler_v
    IRQ  TWIM0_TWIS0_TWI0_SPIM1_SPIS1_SPI1_IRQHandler_v
    IRQ  SPIM0_SPIS0_SPI0_IRQHandler_v
    IRQ  GPIOTE_IRQHandler_v
    IRQ  SAADC_IRQHandler_v
    IRQ  TIMER0_IRQHandler_v
    IRQ  TIMER1_IRQHandler_v
    IRQ  TIMER2_IRQHandler_v
    IRQ  RTC0_IRQHandler                                 /* blocked    */
    IRQ  TEMP_IRQHandler_v
    IRQ  RNG_IRQHandler                                  /* restricted */
    IRQ  ECB_IRQHandler                                  /* restricted */
    IRQ  CCM_AAR_IRQHandler                              /* blocked    */
    IRQ  WDT_IRQHandler_v
    IRQ  RTC1_IRQHandler_v
    IRQ  QDEC_IRQHandler_v
    IRQ  COMP_IRQHandler_v
    IRQ  SWI0_EGU0_IRQHandler_v
    IRQ  SWI1_EGU1_IRQHandler_v
    IRQ  SWI2_IRQHandler_v
    IRQ  SWI3_IRQHandler_v
    IRQ  SWI4_IRQHandler
    IRQ  SWI5_IRQHandler
    IRQ  PWM0_IRQHandler_v
    IRQ  PDM_IRQHandler_v

  .end
