![Freehand Drawing.svg](https://dsuj2mkiosyd2.cloudfront.net/fusion-360/170415/7795/11ec0c51/raasrendering-9a010002-ce0d-4237-bea3-5811ef1b8e23_600-auto.webp)

# STM32F446RE HC-SR04 HAL Driver

### Ultrasonic distance measurement with temperature compensation, 3-sample moving average, and UART output

![License: GPLv3](https://img.shields.io/badge/License-GPLv3-blue.svg)

## Overview

This project implements an ultrasonic distance measurement system using the HC-SR04 sensor and an STM32F446RE Nucleo-64 board. It features temperature-adjusted sound speed, a 3-sample moving average for stable readings, pulse width filtering to reject invalid measurements, timeout handling for lost signals, and diagnostic output via UART.

> Step-by-step Russian guide available at: [guideRU.md](https://github.com/cybernethica-cordis/STM32-HCSR04-HAL/blob/main/guideRU.md).

## Key Features

- **Hardware**: Direct interfacing with HC-SR04 using 5V tolerant pins (PB8 — ECHO, PB9 — TRIG)
    
- **Timing**: TIM2 configured at 1 µs resolution
    
- **Compensation & Filtering**: Temperature-based sound speed calculation and 3-sample averaging
    
- **Robustness**: Pulse width validation (100–25000 µs, or ~1.7–425 cm) with hardware timeouts
    
- **Debug Output**: UART2 at 115200 baud for diagnostic messages (to Linux host via Minicom)
    

## Hardware Connections

| HC-SR04 | STM32F446RE (Nucleo) |
| --- | --- |
| VCC | 5V  |
| TRIG | PB9 (Arduino D14) |
| ECHO | PB8 (Arduino D15) |
| GND | GND |

## Development Setup

- **Clock**: HSI 16 MHz → SYSCLK 16 MHz
    
- **TIM2**: Internal clock, Prescaler=15 (1 µs resolution)
    
- **USART2**: 115200 baud, 8N1
    
- **GPIO**:
    
    - **PB9**: Push-pull output (TRIG)
    - **PB8**: Floating input (ECHO)
- **Compiler Flag**:
    
    ```bash
    -u _printf_float  # Enable floating-point support in printf
    ```
    

## Measurement Algorithm

### **Overview**

- **Trigger Generation**: Set TRIG LOW for stabilization (4 µs), then HIGH for 10 µs, and back to LOW
    
- **Echo Detection & Timeout**: Wait for the rising edge on ECHO with a timeout to avoid blocking
    
- **Pulse Measurement**: Record pulse start and end times (accounting for timer overflow).
    
- **Validation**: Discard pulses shorter than 100 µs or longer than 25000 µs.
    
- **Distance Calculation**:
    
    ```C
    distance = (pulse_width * sound_speed * calibration_factor) / 2.0;
    ```
    
- **Averaging**: Use a static 3-sample buffer to compute a moving average.
    
- **Output**: Transmit the result over UART.
    

### Realisation

> **Note**: All necessary variables (e.g., `timeout`, `pulse_start`, `pulse_end`, `pulse_width`, `sound_speed`, `calibration_factor`, `uart_buffer`) were declared earlier. The `sound_speed` variable is computed based on the ambient temperature for accurate compensation.

```c
while (1) {
  /* *** Generate trigger pulse for HC-SR04 *** */
  // Reset the TRIG pin to LOW and wait 4 µs to stabilize the sensor.
  HAL_GPIO_WritePin(GPIOB, TRIG_PIN, GPIO_PIN_RESET);
  micro_delay(4);

  // Generate a HIGH pulse lasting 10 µs.
  HAL_GPIO_WritePin(GPIOB, TRIG_PIN, GPIO_PIN_SET);
  micro_delay(10);
  HAL_GPIO_WritePin(GPIOB, TRIG_PIN, GPIO_PIN_RESET);

  /* *** Wait for the signal to appear on the ECHO pin *** */
  // Use the timer to limit the waiting time (timeout) to avoid an infinite loop if no signal is received.
  uint32_t timeout_timer = __HAL_TIM_GET_COUNTER(&htim2); // Record the start time of the waiting period.
  while (HAL_GPIO_ReadPin(GPIOB, ECHO_PIN) == GPIO_PIN_RESET) {
    // If the difference between the current time and the start time exceeds the timeout, break the waiting loop.
    if ((__HAL_TIM_GET_COUNTER(&htim2) - timeout_timer) > timeout) {
      break;
    }
  }

  // After the waiting loop, check if the ECHO pin has been set to HIGH:
  // If the pin is still LOW, the signal was not received and the measurement is considered invalid.
  if (HAL_GPIO_ReadPin(GPIOB, ECHO_PIN) != GPIO_PIN_SET) {
    // Send an error message via UART and skip the current measurement.
    HAL_UART_Transmit(&huart2, (uint8_t*)"Error: No echo\r\n", strlen("Error: No echo\r\n"), 100);
    HAL_Delay(2000);
    continue;
  }

  /* *** Measure the pulse duration on the ECHO pin *** */
  // Record the time at the start of the pulse.
  pulse_start = __HAL_TIM_GET_COUNTER(&htim2);
  // Set an additional timeout for the pulse measurement.
  uint32_t pulse_timeout = pulse_start + timeout;

  // Wait while the ECHO pin remains HIGH:
  // If the measurement time exceeds the timeout, break out of the loop.
  while (HAL_GPIO_ReadPin(GPIOB, ECHO_PIN) == GPIO_PIN_SET) {
    if (__HAL_TIM_GET_COUNTER(&htim2) > pulse_timeout) {
      break;
    }
  }

  // Record the time at the end of the pulse.
  pulse_end = __HAL_TIM_GET_COUNTER(&htim2);

  /* *** Calculate the pulse duration accounting for timer overflow *** */
  // If the timer did not overflow, the difference between pulse_end and pulse_start gives the duration.
  // If an overflow occurred (pulse_end < pulse_start), adjust the value accordingly.
  if (pulse_end >= pulse_start) {
    pulse_width = pulse_end - pulse_start;
  } else {
    pulse_width = (0xFFFFFFFF - pulse_start) + pulse_end;
  }

  /* *** Filter the measurements *** */
  // Discard measurements that are too short (< 100 µs, which corresponds to ~1.7 cm)
  // or too long (> 25000 µs, which corresponds to ~425 cm), as they are likely erroneous.
  if (pulse_width > 25000 || pulse_width < 100) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"Invalid pulse\r\n", strlen("Invalid pulse\r\n"), 100);
    continue;
  }

  /* *** Calculate the distance *** */
  // Use the formula: distance = (pulse_width * sound_speed * calibration_factor) / 2.
  // Division by 2 is required because the measured time corresponds to the signal traversing the path twice.
  distance = (pulse_width * sound_speed * calibration_factor) / 2.0;

  /* *** Averaging multiple measurements *** */
  // To improve result stability, store the last three measurements in a static buffer,
  // then calculate their average.
  static float avg_buf[3] = {0};
  static uint8_t idx = 0;
  avg_buf[idx++] = distance;
  if (idx >= 3) idx = 0;
  float avg = (avg_buf[0] + avg_buf[1] + avg_buf[2]) / 3;

  /* *** Send the result via UART *** */
  // Form a string with the averaged distance value and send it via UART.
  sprintf(uart_buffer, "Distance: %.1f cm\r\n", avg);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);

  // A short delay (200 ms) between measurements.
  HAL_Delay(200);
}
```

## Data Transmission

**Linux Host Configuration**:

```bash
minicom -D /dev/ttyACM0 -b 115200
```

**Output Format**:

```bash
Distance: 153.2 cm   # Averaged from 3 samples
Error: No echo       # Timeout occurred
Invalid pulse        # Out-of-range measurement
```

## Build & Deployment

- Generate code from **STM32CubeMX**
- Build the project using **STM32CubeIDE**
- Flash the firmware via **STM32CubeProgrammer**

## Pre-built Binaries

- **HCSR04-UART-HAL.elf**: Firmware file in ELF format (for debugging and development)
- **HCSR04-UART-HAL.bin**: Firmware file in binary format (for flashing onto the MCU)

## Checksums

- **HCSR04-UART-HAL.elf**: `f9b950dd1d6cd54f41f8bb25eae6cf492fc0faf51f0e445a234141d0c680f81c`
- **HCSR04-UART-HAL.bin**: `08924f3c5479d4a6d00dd415fa0d301d67dd09f6d61f4fddd2fd9069bd846cc6`

> To check the integrity of the downloaded binaries, run the following command in your terminal:

```bash
sha256sum HCSR04-UART-HAL.elf  
sha256sum HCSR04-UART-HAL.bin
```

> Alternatively, you can use the following command to directly check the checksums:

```bash
echo "f9b950dd1d6cd54f41f8bb25eae6cf492fc0faf51f0e445a234141d0c680f81c HCSR04-UART-HAL.elf" | sha256sum --check  
echo "08924f3c5479d4a6d00dd415fa0d301d67dd09f6d61f4fddd2fd9069bd846cc6 HCSR04-UART-HAL.bin" | sha256sum --check
```

> **Note**: If the output does not match the provided checksums, do not proceed with flashing the firmware, as the file may have been corrupted during download. In such a case, re-download the file and check the checksum again.

## Calibration & Optimization

| Parameter | Default | Adjustment Method |
| --- | --- | --- |
| Calibration factor | 1.0 | Based on empirical measurements (e.g., with a laser reference) |
| Measurement delay | 200 ms | Modifiable in `HAL_Delay()` |
| Averaging depth | 3 samples | Fixed via a static buffer |

> **Note**: Ensure a stable 5V power supply and consider adding a 100nF decoupling capacitor near the HC-SR04 VCC. Adjust timeout and calibration parameters as needed.

## References

- [STM32 Nucleo Boards Documentation](https://www.st.com/en/evaluation-tools/stm32-nucleo-boards/documentation.html)
    
- [STM32F446RE Documentation](https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html#documentation)

- [HC-SR04 Product Features](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
    
- [STM32CubeIDE User Guide](https://www.st.com/resource/en/user_manual/dm00629856-stm32cubeide-user-guide-stmicroelectronics.pdf)
