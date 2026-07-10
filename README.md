# Simulation and Hardware Implementation of Robust Torque Control of DC Motor

*Objective* : Estimation of disturbance and unknown load, feedforward for robust control. 

## Folder Included
* Hardware Implementation - contains all sketch(.ino) and plot (.py) files
* Motor Docs - contains accquired documents on motor
* Simulations - contains a Params file(.m) and Simulink model (.slx)

## Sketches used
* `Fig9 Resources\code_fig9_sketch` - runs the motor at a constant PWM and accquires current, encoder and RPM readings
* `Fig10 Resources\code_fig10_sketch` - runs the motor at step RPM and accquires current, encoder and RPM readings
* `Fig11 Resources\code_fig11_sketch` - runs the motor at sine RPM and accquires current, encoder and RPM readings

## Key Function - Common to All

### GPT Implementation
* Single hardware GPT timer (`FspTimer`) ticks at `SAMPLE_HZ` (5 kHz), calling `sampleISR()`
* ISR reads current, filters it (CSKF), reads encoder angle, latches last commanded PWM - all in one synchronized tick
* Modulo counters inside the ISR dispatch slower-rate work off the same tick: RPM update every `OUTER_DIV` ticks, log snapshot every `LOG_DIV` ticks

### Data Logging
* On `LOG_DIV` boundary, ISR writes a `Sample` struct (t_us, current_A, filtered_A, pwmCmd, encCount, rpm) into `latestSample` and sets `logReady`
* `loop()` polls `logReady`, prints CSV over Serial - keeps all `Serial.print` out of the ISR
* `START` / `STOP` serial commands toggle `isRunning`, gating the ISR and motor drive

## User Manaual 
Step by step guide to implement this project on hardware
Hardware used : Arduino UNO R4 Wifi, Cytron MDD10A, HW872 / ACS712, Quadrature Hall effect encoder, Planetary geared DC motor 

### Softwares Used
- MATLAB and SIMULINK for simulations; along with control systems library
- Arduino IDE for flashing and programming the arduino uno
- Jupyter notebook for `.ipynb` files
- VS code for `.py` files and for result plots

### Libraries used 
- FspTimer - [Github](https://github.com/embedded-kiddie/CallbackTimerR4)
- Pwm - [Github](https://github.com/terryjmyers/PWM)
- TinyEKF - [Github](https://github.com/simondlevy/TinyEKF.git)

### Usage 
1. First, ensure the connections are correct and steady
2. Verify the pins used in code and hardware before flashing
3. Then select the file which is to be run on the hardware
4. Ensure that necessary libraries are installed
5. `Upload` the files on Arudino
6. Use the Serial Monitor set at `115200` baud rate to obtain the Serial Logs.

### Data Plots 
1. Copy the serial log and create a `.csv` file
2. Rename the file and keep in the same directory as of the `<example>_plot.py` (Plot `,py` files in `SubmissionDCL/Hardware Implementation Fig9,10,11/Fig10 Resources`
3. This will plot the results and also keep in mind about the column name in the plots.

To check if the code has is working correctly, press the `RESET` button on Arduino and this will print the `offSet` voltage of the current sensor on serial monitor.
