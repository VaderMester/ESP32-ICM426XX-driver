# ExampleHigh Rate Logger (8/16/32KHz)


## Overview

This example allows user to stream and collect data at high rate (8, 16 or 32 KHz). The Microchip G55 microcontroler
reads Accel and/or Gyro data from registers using SPI at 24 MHz. Data are then streamed to a Windows computer through
UART at 3 MHz.

A Python script is provided to collect and process the data. Please note that the processing is done once the collect
is completed to prevent any slow-down which could cause data loss.

At 8 or 16 KHz, all 6-axis are reported as well as a counter over a byte (0 to 255)
At 32 KHz, only 1 axis can be reported at a time with the counter. 
ODR and axis selection can be changed in the `example-high-rate-logger.h` file.

The following processing are available:
* __Printer__: Print all data on the terminal (warning: there will be a lot of lines)
* __Checker__: Check counter value and report error if a data is lost (delta > 1)
* __Logger__: Log all data into a file
* __Plotter__: Plot data using Matplotlib

## Requirements

Python is used to collect and process the data. Please install Python 3.8 or newer version.
For UART communication, Pyserial is used. 
For plotting, Matplotlib is used. 

You can install them with the following commands:
```
python -m pip install -U pip
python -m pip install -U pyserial
python -m pip install -U matplotlib
```

In case you are facing the following error: `AttributeError: module 'serial' has no attribute 'Serial'`, 
please uninstall both `pyserial` and `serial` module before reinstalling `pyserial`:
```
python -m pip uninstall serial
python -m pip uninstall pyserial
python -m pip install -U pyserial
```

## Step-by-step

* Build and flash the _example-high-rate-logger_ in Release mode (optimized in speed)
* Launch Python script:
 * Get help screen:
 ```
 python uart_logger.py -h
 ```
 * Example of usage (10 sec on port COM23, data will be logged and plotted): 
 ```
 python uart_logger.py --port COM23 --duration 10 --logger --plotter
 ```
