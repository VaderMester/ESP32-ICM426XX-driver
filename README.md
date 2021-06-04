# ESP32 ICM426XX driver

This component is a port of the official ICM426XX driver by Invensys.

This has only been tested with ICM42605 sensor.

Should work with ESP-IDF V4.3 release branch. Developed using IDF 4.4 master branch.
There are some tweaks in InvenSensen's driver implementation here and there, so any direct update of the driver may not be straight forward.

# STATUS

Driver in this current state works nicely.
I haven't tested it with SPI, and other ICM devices, but they also should work.

# CURRENT EXAMPLE

Currently, main.c uses the device FIFO to read out sensor data, and log them to console converted to their respective units.

Output should be this:
```
W (34393) ICM426XX-Example: Waiting for an interrupt to happen!
W (34473) ICM426XX-Example: --> INTERRUPTS: Found: 2, detected: 0
I (34473) ICM426XX-Example: INTR_ICM426XX_UI_DRDY interrupt detected (0)
I (34473) ICM426XX-Example: INTR_ICM426XX_FIFO_THS interrupt detected (1)
I (34493) ICM426XX-Example: FIFO -> 10 packets read
I (34493) ICM426XX-Example: RAW AG DATA: <1>ms   temp:24.0C     [x]-3   [y]--19 [z]-997 -       [Rx]-0  [Ry]-0  [Rz]-0
I (34503) ICM426XX-Example: RAW AG DATA: <1>ms   temp:24.0C     [x]-3   [y]--17 [z]-997 -       [Rx]-0  [Ry]-0  [Rz]-0
I (34513) ICM426XX-Example: RAW AG DATA: <1>ms   temp:24.0C     [x]-3   [y]--18 [z]-997 -       [Rx]-0  [Ry]-0  [Rz]-0
I (34523) ICM426XX-Example: RAW AG DATA: <1>ms   temp:24.0C     [x]-3   [y]--18 [z]-998 -       [Rx]-0  [Ry]-0  [Rz]-0
I (34533) ICM426XX-Example: RAW AG DATA: <1>ms   temp:24.0C     [x]-4   [y]--17 [z]-997 -       [Rx]-0  [Ry]-0  [Rz]-0
I (34543) ICM426XX-Example: RAW AG DATA: <1>ms   temp:24.0C     [x]-4   [y]--17 [z]-996 -       [Rx]-0  [Ry]-0  [Rz]-0
I (34553) ICM426XX-Example: RAW AG DATA: <2>ms   temp:24.0C     [x]-3   [y]--18 [z]-997 -       [Rx]-0  [Ry]-0  [Rz]-0
I (34563) ICM426XX-Example: RAW AG DATA: <2>ms   temp:24.0C     [x]-4   [y]--17 [z]-996 -       [Rx]-0  [Ry]-0  [Rz]-0
I (34573) ICM426XX-Example: RAW AG DATA: <2>ms   temp:24.0C     [x]-3   [y]--16 [z]-996 -       [Rx]-0  [Ry]-0  [Rz]-0
I (34583) ICM426XX-Example: RAW AG DATA: <2>ms   temp:24.0C     [x]-3   [y]--18 [z]-996 -       [Rx]-0  [Ry]-0  [Rz]-0
```

# TODOs

- Implement Self-Test and zeroing calibration with values stored in flash
- Implement APEX, Pedometer, SMD, and other functionalities of the device.
- Do some more debugging if needed.

# SERIAL INTERFACES

You can easily include your specific serial interface drivers, and plug them into the example code.
You only need to add them to user code, driver don't need to be changed.

In main.c there are the following functions.
You can then include the header of the serial driver to main.c and include read/write/init functions here
An example I2C driver has been provided by default.

```
static int ESP32_HAL_read_reg(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len)
{
	switch (serif->serif_type)
	{
	case ICM426XX_AUX1_SPI3:
	case ICM426XX_AUX1_SPI4:
	case ICM426XX_AUX2_SPI3:
	case ICM426XX_UI_SPI4:
	//Add your SPI read function here
	case ICM426XX_UI_I2C:
	//Add your I2C read functions here
	default:
		return -1;
	}
	return 1;
}

static int ESP32_HAL_write_reg(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len)
{
	switch (serif->serif_type)
	{
	case ICM426XX_AUX1_SPI3:
	case ICM426XX_AUX1_SPI4:
	case ICM426XX_AUX2_SPI3:
	case ICM426XX_UI_SPI4:
	//Add your SPI write function here
	case ICM426XX_UI_I2C:
	// Add your I2C write functions here
	default:
		return -1;
	}
	return 1;
}

static int ESP32_icm_serif_init(struct inv_icm426xx_serif *serif)
{
	int rc;
	switch (serif->serif_type)
	{
	case ICM426XX_AUX1_SPI3:
	case ICM426XX_AUX1_SPI4:
	case ICM426XX_AUX2_SPI3:
	case ICM426XX_UI_SPI4:
    //Add your SPI init here
	case ICM426XX_UI_I2C:
	//Add your I2C init here
	default:
		return -1;
	}
	return 1;
}
```