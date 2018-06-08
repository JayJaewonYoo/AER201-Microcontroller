# AER201-Microcontroller
Software for a hardware fastener sorting machine. Used in AER201, 2018 Winter. For PIC18F4620. MPLAB X IDE used. 

All the code is found in main.c to avoid file linking issues. Each set of functions designated for specific tasks are divided by editor-folds.

How to download:
1) Create a new project using MPLAB X configured for a PIC18F4620 microcontroller.
2) Delete the automatically generated main.c
3) Download all files in the repository excluding README.md and FinalReport_MicrocontrollerSections_JayJaewonYoo.pdf and move them to the new MPLAB X IDE project directory. 
4) In the MPLAB X IDE, right click the Source Files folder under the project and select "Add Existing Items" then select all .c files. 
5) Right click the Header Files folder under the project and select "Add Existing Items" then select all .h files. 
6) Connect to the PIC18F4620 microcontroller and select "Make and Program Device Main Project." 

Available functions:
- Keypad and LCD Interfacing
- Integration with Physical Components
- Real-Time Clock
- Operation Time Calculation
- GLCD Interfacing with Text Output and Progress Bar
- EEPROM Storage
- PC Interfacing

Sensors and Actuators Used:
- Nema 17 Stepper Motor Bipolar L=39mm w/ Gear Ratio 19:1 Planetary Gearbox
- 5V Mini Pull Type Solenoid
- Micro Switch
- Character Liquid-Crystal Display (LCD) and Keypad with Encoder Chip
- Real-time Clock (RTC) Chip and Coin Battery
- TSL235R Light Sensor
- Mini Vibration Motor - 10mm
- DC Motors

Note, operator inputs of high values have not been tested extensively and could contain bugs.

For more information on the code, refer to the file "FinalReport_MicrocontrollerSections_JayJaewonYoo" which details the purpose and functionality of each section of the code. This file refers to Appendix E which is not included. Appendix E is simply a copy of the code and line numbers specified for Appendix E match that of main.c.
