#include <stdint.h>

#include "driver/i2c.h"
#include "I2Cdev.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "string.h"
#include "esp_err.h"

esp_err_t inited = ESP_FAIL; //checker to see if I2Cdev_init() has been called before
uint32_t timeOut = 10;

#define I2C_TAG "I2Cdev"

#define I2C_CHECK_RET(ret)  do{esp_err_t err_rc_ = (ret); if(err_rc_ != ESP_OK) {                    \
        ESP_LOGE(I2C_TAG,"%s(%d)=%d: %s", __FUNCTION__, __LINE__, err_rc_, esp_err_to_name(err_rc_));    \
        return (err_rc_);}                                                                           \
        } while(0)

#define I2C_CHECK(ret)  do{esp_err_t err_rc_ = (ret); if(err_rc_ != ESP_OK) {                        \
        ESP_LOGE(I2C_TAG,"%s(%d)=%d: %s", __FUNCTION__, __LINE__, err_rc_, esp_err_to_name(err_rc_));    \
        }} while(0)

esp_err_t I2Cdev_writeByte(uint8_t devAdd, uint8_t regAddr, uint8_t data) {
	i2c_cmd_handle_t cmd = {0};

	cmd = i2c_cmd_link_create();
	I2C_CHECK(i2c_master_start(cmd));
	I2C_CHECK(i2c_master_write_byte(cmd, (devAdd << 1), 1));
	I2C_CHECK(i2c_master_write_byte(cmd, regAddr, 1));
	I2C_CHECK(i2c_master_write_byte(cmd, data, 1));
	I2C_CHECK(i2c_master_stop(cmd));
	I2C_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, timeOut/portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);

	return ESP_OK;
}

esp_err_t I2Cdev_SelectRegister(uint8_t dev, uint8_t reg){
	esp_err_t ret = ESP_OK;
	i2c_cmd_handle_t cmd = {0};

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, dev, 1);
	i2c_master_write_byte(cmd, reg, 1);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, timeOut/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	I2C_CHECK(ret);
	return ret;
}

esp_err_t I2Cdev_writeBytes(uint8_t devAdd, uint8_t regAddr, uint32_t length, uint8_t *data)
{
	esp_err_t ret = ESP_OK;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, devAdd << 1, 1);
	i2c_master_write_byte(cmd, regAddr, 1);
	if (length > 1)
	{
		i2c_master_write(cmd, data, (size_t)(length-1), 0);
		i2c_master_write_byte(cmd, data[length-1], 1);
	}
	else
	{
		//ESP_LOGW(I2C_TAG, "DEB: byte: 0x%02X at addr: 0x%02X", *data, regAddr);
		i2c_master_write_byte(cmd, data[0], 1);
	}
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, timeOut / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	I2C_CHECK(ret);
	return ret;
}

esp_err_t I2Cdev_readBytes(uint8_t devAdd, uint8_t regAddr, uint32_t length, uint8_t *data)
{
	esp_err_t ret = ESP_OK;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, devAdd << 1, 1);
	i2c_master_write_byte(cmd, regAddr, 1);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (devAdd << 1)| 0x01, 1);

	if (length > 1)
	{
		/*
	i2c_master_read(cmd, data, length-1, 0));
	i2c_master_read_byte(cmd, data+length-1, 1));
	*/
		i2c_master_read(cmd, data, length, I2C_MASTER_LAST_NACK);
	}
	else
	{
		i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
	}

	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, timeOut / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	I2C_CHECK(ret);
	return ret;
}

uint8_t I2Cdev_readByte(uint8_t devAdd, uint8_t regAddr)
{
	uint8_t data;
	I2Cdev_readBytes(devAdd, regAddr, 1, &data);
	return data;
}

uint8_t I2Cdev_readBits(uint8_t devAdd, uint8_t regAddr, uint8_t bitStart, uint32_t length, uint8_t *data) {

    uint8_t count, b;
    if ((count = I2Cdev_readBytes(devAdd, regAddr, 1, &b)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

esp_err_t I2Cdev_writeBits(uint8_t devAdd, uint8_t regAddr, uint8_t bitStart, uint32_t length, uint8_t data) {
	uint8_t b = 0;
    if (I2Cdev_readBytes(devAdd, regAddr, 1, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return I2Cdev_writeByte(devAdd, regAddr, b); //
    } else {
        return ESP_FAIL;
    }
}

esp_err_t I2Cdev_writeBit(uint8_t devAdd, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    I2Cdev_readBytes(devAdd, regAddr, 1, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return I2Cdev_writeByte(devAdd, regAddr, b);
}

uint8_t I2Cdev_readWord(uint8_t devAdd, uint8_t regAddr, uint16_t *data){
	uint8_t msb[2] = {0,0};
	I2Cdev_readBytes(devAdd, regAddr, 2, msb);
	*data = (int16_t)((msb[0] << 8) | msb[1]);
	return 0;
}

esp_err_t I2Cdev_writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data){

	uint8_t data1[] = {(uint8_t)(data>>8), (uint8_t)(data & 0xff)};
	//uint8_t data2[] = {(uint8_t)(data & 0xff), (uint8_t)(data>>8)};
	I2Cdev_writeBytes(devAddr, regAddr, 2, data1);
	return ESP_OK;
}

esp_err_t I2Cdev_init(int gpio_pin_sda, int gpio_pin_scl)
{
	if (inited != ESP_OK)
	{
		ESP_LOGI(I2C_TAG, "Installing I2C master driver");
		i2c_config_t conf = {0};
		conf.mode = I2C_MODE_MASTER;
		if (gpio_pin_sda < 0)
		{
			conf.sda_io_num = (gpio_num_t)CONFIG_I2CDEV_SDA_DEF_GPIO;
		}
		else
		{
			conf.sda_io_num = (gpio_num_t)gpio_pin_sda;
		}
		if (gpio_pin_scl < 0)
		{
			conf.scl_io_num = (gpio_num_t)CONFIG_I2CDEV_SCL_DEF_GPIO;
		}
		else
		{
			conf.sda_io_num = gpio_pin_scl;
		}
#ifdef CONFIG_I2CDEV_SDA_PULLUP_EN
		conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
#else
		conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
#endif //#ifdef CONFIG_I2CDEV_SDA_PULLUP_EN
#ifdef CONFIG_I2CDEV_SCL_PULLUP_EN
		conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
#else
		conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
#endif //#ifdef CONFIG_I2CDEV_SCL_PULLUP_EN
		conf.master.clk_speed = (uint32_t) CONFIG_I2CDEV_DEF_SCL_FREQ;
		esp_err_t rc = ESP_OK;
		if ((rc = i2c_param_config(I2C_NUM_0, &conf)) != ESP_OK)
		{
			return rc;
		}
		else
		{
			if ((rc = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0)) != ESP_OK)
			{
				return rc;
			}
			else
			{
				inited = ESP_OK;
				return ESP_OK;
			}
		}
		return rc;
	}
	return ESP_OK;
}

esp_err_t I2Cdev_scan(uint8_t *foundAddr, int size)
{
	if(inited == ESP_OK)
	{
	char *output;
	output = malloc(500);
	if(output) {
	int i, f;
	f = 0;
	esp_err_t espRc;
	sprintf(output, "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	sprintf(output+strlen(output), "00:         ");
	for (i=3; i< 0x78; i++) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, i << 1, 1 /* expect ack */);
		i2c_master_stop(cmd);

		espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
		if (i%16 == 0) {
			sprintf(output+strlen(output),"\n%.2x:", i);
		}
		if (espRc == 0) {
			sprintf(output+strlen(output), " %.2x", i);
			if(f < size) {
			foundAddr[f] = i;
			f++;
			}
		} else {
			sprintf(output+strlen(output),  " --");
		}
		//ESP_LOGD(tag, "i=%d, rc=%d (0x%x)", i, espRc, espRc);
		i2c_cmd_link_delete(cmd);
	}
	ESP_LOGI(I2C_TAG, "--------- I2C SCAN ---------\n%s", output);
	return ESP_OK;
	}
	}
	ESP_LOGE(I2C_TAG, "I2C driver not installed!");
	return ESP_FAIL;
}