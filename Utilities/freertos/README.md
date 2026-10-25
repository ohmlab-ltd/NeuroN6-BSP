# FreeRTOS in a Neuro N6 BSP project

What a project needs to run the BSP under FreeRTOS. With `BSP_USE_FREERTOS`
at 0 none of this is compiled and the bare-metal BSP is unchanged.

## What a project adds

1. **The kernel.** Compile every `.c` under `Middlewares/FreeRTOS/Source`
   (the kernel, `portable/GCC/ARM_CM55_NTZ/non_secure`, `MemMang/heap_4.c`,
   and the CMSIS-RTOS2 wrapper if wanted). Include paths:
   ```
   Middlewares/FreeRTOS/Source/include
   Middlewares/FreeRTOS/Source/portable/GCC/ARM_CM55_NTZ/non_secure
   Middlewares/FreeRTOS/Source/CMSIS_RTOS_V2      (only for the wrapper)
   Utilities/freertos                             (FreeRTOSConfig.h)
   ```
2. **The configuration.** `FreeRTOSConfig.h` here: 1 kHz tick, 56 priorities,
   96 KB heap, static allocation, stack-overflow checking, newlib reentrancy,
   idle hook, `configMAX_SYSCALL_INTERRUPT_PRIORITY` 5. `neuron6_rtos.c`
   supplies its assert handler `n6_configassert_fail` and a weak, empty
   `vApplicationIdleHook`.
3. **The BSP glue.** Compile `neuron6_rtos.c` and set `BSP_USE_FREERTOS` to 1
   in `neuron6_conf.h`. The same switch selects TinyUSB's FreeRTOS layer in
   `tusb_config.h` and the encoder's event-group wait in `ewl_conf.h`. If the
   build passes `-DCFG_TUSB_OS=OPT_OS_NONE` (the CubeIDE projects here do),
   change it to `OPT_OS_FREERTOS` or remove it.
4. **The HAL tick.** The kernel owns SysTick. Compile
   `stm32n6xx_hal_timebase_tim.c` from here (HAL tick on TIM16, which
   `neuron6_io` and `neuron6_pwm` leave free) and enable
   `HAL_TIM_MODULE_ENABLED` with the TIM sources. While the scheduler is
   suspended, as it is for every I2C transaction, `HAL_GetTick` keeps counting
   from the cycle counter, so HAL timeouts still expire. The `rtos` console
   command checks this.
5. **The vectors.** In `stm32n6xx_it.c`, remove `SVC_Handler`,
   `PendSV_Handler` and `SysTick_Handler` (the port defines them) and add
   `TIM16_IRQHandler` calling `HAL_TIM_IRQHandler(&htim16)`. The examples do
   both under `#if BSP_USE_FREERTOS`.
6. **The USB task.** After `BSP_COM_Init` and before `vTaskStartScheduler`,
   call `BSP_RTOS_StartUsbTask(0, 0)`. It services the stack (interrupt-woken,
   1 ms fallback) and keeps the console's `b` and 1200-baud routes to the
   bootloader working. Nothing else should call `BSP_COM_Task`.
7. **Start the scheduler** once the peripherals are up. Bring-up is single
   threaded and every BSP lock is a no-op until the scheduler runs.

## Rules

- An interrupt that calls a `*FromISR` function must be at priority 5..15.
  The BSP's are: capture 5, audio and BLE 6, header pins and tone 8, button
  14, HAL tick 15. The exception is the watchdog early warning at 0, whose
  callback must not use the kernel. Application callbacks run at the priority
  of the BSP interrupt that calls them.
- `HAL_Delay` yields under the kernel (`neuron6_rtos.c` overrides it). It
  still spins before the scheduler starts and inside an interrupt.
- The weak `BSP_I2C1/I2C2/SPI2/ADC/WIFI_Lock` hooks are recursive mutexes:
  a no-op before the scheduler and in an interrupt, an unbounded take
  otherwise.
- `BSP_I2C_TransactionBegin/End` suspends the scheduler for one I2C
  transaction (interrupts still run). A task switch mid-transaction holds SCL
  stretched, and the magnetometer then stores a corrupted register.
- The kernel heap (`ucHeap`, 96 KB, `.bss`) holds stacks, queues and
  semaphores. The encoder uses the C heap, which `neuron6_rtos.c` locks.
- A stack overflow or failed `configASSERT` calls `BSP_RTOS_FatalHook`. The
  default resets the chip; the examples write a breadcrumb and reboot into
  the bootloader.

## USB before the scheduler

The examples start the scheduler first and let the USB task enumerate.
Pumping USB before the scheduler relies on the local patches in the kernel
port (`Middlewares/FreeRTOS/README.md`) and in
`Middlewares/tinyusb/osal/osal_freertos.h`, which cuts every wait to zero
until the scheduler starts.
