# ESP32 ICM426XX driver

This component is a port of the official ICM426XX driver by Invensys.

This has only been tested with ICM42605 sensor.

Should work with ESP-IDF V4.3 release branch. Developed using IDF 4.4 master branch.
There are some tweaks in InvenSensen's driver implementation here and there, so any direct update of the driver may not be straight forward.

# IMPORTANT
This driver does not yet work. For some reason device configuration is not how it should be, there are problems with reading the FIFO. Waiting for InvenSense to check my findings.

# STATUS

Documentation is currently being written, stay tuned.