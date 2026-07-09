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

