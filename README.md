# 📌 STM32 Bare-Metal Learning Labs

This repository contains a collection of **bare-metal STM32F411 experiments**
implemented using **CMSIS** (no HAL or LL drivers). It's my progression of learning Bare
Metal MCU programming

Each subdirectory is a self-contained lab focused on **one peripheral or system
concept and it's practical use**

Target MCU: **STM32F411 (Cortex-M4) aka The Black Pill**

---

## 🎯 Goals of This Repository

- Understand STM32 at **register level**
- Learn how peripherals interact (DMA ↔ Timer ↔ GPIO ↔ NVIC)
- Avoid dependency on vendor abstraction layers
- How to use best practices in **c++** for maximum efficiency

This is not production firmware — it is a **silicon exploration lab**.

---

## 🗂 Projects [Other will be added soon this repo is not complete yet]
- [01 - Interrupt Based Binary Counter](https://github.com/Muhammad-Hassan-Tariq/Bare-Metal-stm32f411-projects/tree/main/01%20-%20Interrupt%20Based%20Binary%20Counter)
- [02 - DMA Based Copying Array](https://github.com/Muhammad-Hassan-Tariq/Bare-Metal-stm32f411-projects/tree/main/02%20-%20DMA%20Based%20Copying%20Array)

---

### 📁 Project Directories

Each project directory consists of:

- `main.cpp`  
  Main project source code

- `schematic.png`  [It may or may not be present based on project itself]
  Schematic to show wiring & overall working

- `demo.mp4`  
  Short recording showing real hardware behavior
  
- `Makefile`  
  A make file responsible for compiling, linking and flashing with one command
  
- `README.md`  
  Explanation of:
  - what the project demonstrates
  - peripheral configuration summary
  - register-level flow

Projects are designed to be **read independently**.

---

## ⚙️ Toolchain Used

All projects use the same base toolchain:

- Compiler: `arm-none-eabi-gcc`
- Assembler: `arm-none-eabi-as`
- Linker script: `/toolchain/linker.ld`
- Startup code: `/toolchain/startup.s`

Flashing via:

- `dfu-util`  

Build system: simple `Makefile` per project or shared include.

---

## ▶️ How to Build a Project

Example:

```bash
cd dma-gpio
make
st-flash write build/main.bin 0x08000000


