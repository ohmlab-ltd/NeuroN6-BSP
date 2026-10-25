# The part this BSP targets, and how it differs from the Discovery kit

The BSP follows the shape of ST's STM32N6570-DK BSP, but the Neuro N6 fits a
smaller package. Something existing on the Discovery kit is not enough reason
to add a module here.

## The two parts

| | Neuro N6 | STM32N6570-DK |
|---|---|---|
| Part number | STM32N657I0H3Q | STM32N657X0H3Q |
| Package | VFBGA178, 12 x 12 mm | VFBGA264 |
| I/O pins | 106 | more |

They share the die family, the core, the NPU and the 4.2 MB of internal RAM,
and neither has internal flash. With 86 fewer balls, some peripherals reach no
pin here, and others reach only some of their listed pins. The firmware builds
with `STM32N657I0HXQ_LRUN.ld` and `startup_stm32n657i0hxq.s`.

## What that rules out

- `stm32n657xx.h` describes the die, not the package. A peripheral defined there may have no pins here.
- **TIM8 has no output.** `neuron6_pwm` uses TIM1, TIM2, TIM15 and TIM17. Header position D13 has no hardware PWM. D12 has none either, because its only timer is the HAL time base.
- **The rule:** treat a peripheral as available only once something on this board uses it.

Peripherals the BSP uses:

- GPIO ports A to H; N, O and P carry the external memories the boot loader owns;
- ADC1 and ADC2;
- TIM1, TIM2, TIM3, TIM15, TIM16 and TIM17;
- I2C1 and I2C2; SPI1 and SPI2; USART6; SDMMC1;
- SAI1, MDF1 and GPDMA1;
- DCMIPP with its CSI-2 receiver, the video encoder, the NPU and LTDC;
- the USB high-speed controller;
- IWDG, RAMCFG, CACHEAXI, RIFSC, RTC and TAMP;
- both XSPI controllers and the multiplexer between them.

## Discovery kit modules this BSP does not have

| Module | Why |
|---|---|
| `_ts` (touch screen) | Not written. The TFT-W panel's Goodix GT911, at 0x14 on the sensor bus, is not driven. |
| `_sd` | No card socket. SDMMC1 is the radio's, and SDMMC2 reaches no usable pin. |
| `_usbpd_pwr` | No power-delivery controller: the USB-C port is device-only. |
| `_xspi` | The boot loader owns both external memories ([BOOT-CONTRACT.md](BOOT-CONTRACT.md)). |

- **Missing camera calls:** `BSP_CAMERA_HwReset` and `BSP_CAMERA_PwrDown` are absent, because no GPIO reaches the sensor's RESETB or PWDN. `BSP_CAMERA_BackgroundProcess` is absent, because ST's software ISP is not carried; the hardware stages are reachable through `BSP_CAMERA_SetIsp*`.
- **Different parts, same modules:** the camera is an OV5640, not an IMX335. Audio is a class-D amplifier and a PDM microphone, with no codec. The panel is a different 800x480 panel on the same controller.

## Discovery kit conventions not followed

- **No weak `MX_<PERIPH>_Init` / `MX_<PERIPH>_ClockConfig`.** Configuration is computed from clocks measured at run time (LTDC, SAI, MDF, I2C2, SPI, timers). CubeMX constants would be wrong on other boot loader builds, and silently so.
- **Narrower overrides instead:** `BUS_I2C1_TIMING_OVERRIDE` and `BUS_I2C2_FREQUENCY`, `BSP_SPI2_SetConfig`, `BSP_LCD_SetRefreshRate` and `BSP_AUDIO_*_SetSampleRate`. Each validates its input against the measured clock.
- **`RegisterMspCallbacks`** exists only for the camera, the console and the buses (`BSP_CAMERA_`, `BSP_COM_`, `BSP_I2C1_`, `BSP_I2C2_`, `BSP_SPI2_`), under `USE_HAL_<PERIPH>_REGISTER_CALLBACKS`. At the shipped setting of 0 the BSP defines only `HAL_DCMIPP_MspInit` and `HAL_PCD_MspInit`. An application's own `HAL_<PERIPH>_MspInit` must therefore check the handle's instance.

## What this BSP has that the Discovery kit does not

- A six-axis IMU and a magnetometer (`neuron6_motion_sensor`).
- A radiometric thermal camera (`neuron6_thermal`).
- A Wi-Fi and BLE module (`neuron6_wifi`, `neuron6_bt`).
- The expansion header: digital, analogue in and PWM (`neuron6_io`, `neuron6_adc`, `neuron6_pwm`).

## A pin wanted by two things

| Pin | Wanted by | Resolution |
|---|---|---|
| PD3 | the thermal camera's power line (`neuron6_thermal.h`) and the BLE link's inbound flow control (`neuron6_bt.h`) | Choose with `USE_BSP_BT_FLOW_CONTROL`. Without flow control the BLE link stays at its starting speed. |

## Sources

- STM32N657I0H3Q package and I/O count: [DigiKey](https://www.digikey.com/en/products/detail/stmicroelectronics/STM32N657I0H3Q/25935640)
- STM32N657X0H3Q on the Discovery kit: [STM32N6570-DK data brief](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/8951/497_stm32n6570-dk.pdf)
- Pin and alternate-function tables: [STM32N6x5xx / STM32N6x7xx datasheet](https://www.st.com/resource/en/datasheet/stm32n657a0.pdf)
