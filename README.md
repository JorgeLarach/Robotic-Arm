_Devlog 12/09/2025_

Since I finished my Ball and Beam project last month, I've been researching and preparing for this new project, the Robotic Arm. Because it uses multiple servos, I figured a PCA9685 I2C controller would be appropriate for this use-case. I also chose to continue to use the STM32 Nucleo line of microcontrollers. For the electronics and wiring section of this project, I had three goals:
* I wanted to make a project that doesn't rely on battery for voltage. It was kind of inconvenient and wanted to try out using a wall powered DC power supply instead.
* I wanted to power the microcontroller externally, rather than supplying USB power, which is what I had done in my previous projects.
* I wanted to challenge myself to not use breadboards at all; something I had relied on previously.

<img src="./Documentation/Pictures/Dec_9_Wiring_Setup.jpg"
     alt="December 9th Wiring Setup"
     style="width:100%; height:auto%;">

As of today, I've taken care of the EV5 (5V) for the Nucleo, the V+ (5V) for the PCA9685, and the Vcc (3.3V) for the PCA9685, all using a 3-12V adjustable DC power supply and an Elegoo Breadboard Power Supply module I found lying around. So far, it looks like I'm within spec for all these components. Next, I'll start working on the I2C driver to control just a single servo motor. 
