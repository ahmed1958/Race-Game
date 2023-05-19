# Race Game README

## Introduction

This README file provides an overview of the project, "Race Game," developed by our team using the Tiva C TM4C123GH6PM microcontroller. The game utilizes various features of the microcontroller, including ADC, interrupts, timers, Nokia5110 screen, GPIO driver, and a timer driver. This document aims to provide essential information about the project, its features, setup instructions, and team members.

## Features

The Race Game project incorporates the following features:

1. **ADC:** The Analog-to-Digital Converter (ADC) module is used to read analog input from various sources, such as a joystick or potentiometer, to control the game's movement or other actions.

2. **Interrupts:** Interrupts are utilized to handle external events and ensure responsive gameplay. They enable efficient utilization of processor time by allowing the microcontroller to respond to specific events promptly.

3. **Timers:** Timers are employed to track time intervals, update screen elements, handle game logic, and control various aspects of the game's behavior.

4. **Nokia5110 Screen:** The Nokia5110 screen is a graphical display used to render the game's visuals, such as the race track, cars, obstacles, and scoreboards. It provides a clear and easy-to-read display for an enhanced user experience.

5. **GPIO Driver:** The General-Purpose Input/Output (GPIO) driver facilitates interaction with the microcontroller's pins, enabling us to connect and control external devices, such as buttons, LEDs, and the Nokia5110 screen.

6. **Timer Driver:** A custom timer driver is implemented to manage the timing requirements of the game. It enables precise control over timer configurations and allows us to efficiently handle game-related timing operations.

## Project Setup

To set up the Race Game project, follow these steps:

1. **Hardware Requirements:**
   - Tiva C TM4C123GH6PM microcontroller board
   - Nokia5110 graphical display
   - Joystick or potentiometer for input control
   - Additional hardware components as desired (e.g., buttons, LEDs)

2. **Software Requirements:**
   - Keil 4 software (or any other compatible Integrated Development Environment, IDE)

3. **Project Configuration:**
   - Connect the necessary hardware components (e.g., Nokia5110 screen, potentiometer,LED) to the appropriate pins of the Tiva C TM4C123GH6PM microcontroller.
   - Import the project into your Keil 4 IDE.
   - Set up the necessary project configurations, including compiler flags, device settings, and linker options, to ensure proper compilation and execution.

4. **Build and Flash:**
   - Build the project in Keil 4 to ensure there are no compilation errors.
   - Flash the compiled binary onto the Tiva C TM4C123GH6PM microcontroller using an appropriate flashing tool or method.

5. **Running the Game:**
   - Power on the microcontroller board and ensure that all the necessary hardware components are connected correctly.
   - The game should start automatically upon power-up or device reset.
   - Use the joystick or potentiometer to control the game and enjoy playing!

## Demo Video

A demo video showcasing the Race Game project is available at the following link: [Race Game Demo Video](https://www.youtube.com/watch?v=tWmYWelguOM)

Please click the link to watch the demo and get a glimpse of the game in action.

## Team Members

The Race Game project was developed by the following team members:

1. [Ahmed Mohamed Ali](https://github.com/ahmedaliv)
2. [Ahmed Sayed Saber](https://github.com/ahmed1958)
3. [Ahmed Mohamed Awaad](https://github.com/Ahmed-Awwad99)
4. [Mohamed Mamdouh Khalil](https://github.com/mohamedmamdouh22)

Please feel free to reach out to any of the team members if you have any questions or require further assistance regarding the project.

