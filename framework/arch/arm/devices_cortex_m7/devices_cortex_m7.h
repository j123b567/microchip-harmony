/**
 * Copyright (c) 2016-2017 Microchip Technology Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _DEVICES_CORTEX_M7_H_INCLUDED_
#define _DEVICES_CORTEX_M7_H_INCLUDED_

/* PIC32CZ CA70 */
#if defined(__PIC32CZ2038CA70064__)
  #include "PIC32CZ2038CA70064/pic32cz2038ca70064.h"
#elif defined(__PIC32CZ1038CA70100__)
  #include "PIC32CZ1038CA70100/pic32cz1038ca70100.h"
#elif defined(__PIC32CZ5125CA70100__)
  #include "PIC32CZ5125CA70100/pic32cz5125ca70100.h"
#elif defined(__PIC32CZ2038CA70100__)
  #include "PIC32CZ2038CA70100/pic32cz2038ca70100.h"
#elif defined(__PIC32CZ2038CA70144__)
  #include "PIC32CZ2038CA70144/pic32cz2038ca70144.h"
#elif defined(__PIC32CZ1038CA70144__)
  #include "PIC32CZ1038CA70144/pic32cz1038ca70144.h"

/* PIC32CZ GC70 */
#elif defined(__PIC32CZ1038GC70100__)
  #include "PIC32CZ1038GC70100/pic32cz1038gc70100.h"
#elif defined(__PIC32CZ2038GC70100__)
  #include "PIC32CZ2038GC70100/pic32cz2038gc70100.h"
#elif defined(__PIC32CZ1038GC70144__)
  #include "PIC32CZ1038GC70144/pic32cz1038gc70144.h"

/* PIC32CZ DA70 */
#elif defined(__PIC32CZ1038DA70176__)
  #include "PIC32CZ1038DA70176/pic32cz1038da70176.h"
#else
  #error "The specified device isn't supported"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined __CM7_REV
typedef struct _CoreVectors
{
  /* Stack pointer */
  void* pvStack;

  /* Cortex-M handlers */
  void* pfnReset_Handler;                        /* -15 Reset Vector, invoked on Power up and warm reset  */
  void* pfnNonMaskableInt_Handler;               /* -14 Non maskable Interrupt, cannot be stopped or preempted  */
  void* pfnHardFault_Handler;                    /* -13 Hard Fault, all classes of Fault     */
  void* pfnMemoryManagement_Handler;             /* -12 Memory Management, MPU mismatch, including Access Violation and No Match  */
  void* pfnBusFault_Handler;                     /* -11 Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory related Fault  */
  void* pfnUsageFault_Handler;                   /* -10 Usage Fault, i.e. Undef Instruction, Illegal State Transition  */
  void* pvReservedC9;
  void* pvReservedC8;
  void* pvReservedC7;
  void* pvReservedC6;
  void* pfnSVCall_Handler;                       /*  -5 System Service Call via SVC instruction  */
  void* pfnDebugMonitor_Handler;                 /*  -4 Debug Monitor                        */
  void* pvReservedC3;
  void* pfnPendSV_Handler;                       /*  -2 Pendable request for system service  */
  void* pfnSysTick_Handler;                      /*  -1 System Tick Timer                    */
} CoreVectors;
#endif /* defined __CM7_REV */

#ifdef __cplusplus
}
#endif


#endif /* _DEVICES_CORTEX_M7_H_INCLUDED_ */
