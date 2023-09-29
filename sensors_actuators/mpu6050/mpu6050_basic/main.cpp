#include <iostream>
#include <pigpiod_if2.h>

#define MPU6050_ADDR 			0x68

//	mpu6050 configuration registers
#define MPU6050_REG_GYRO_CONFIG		0x1B
#define MPU6050_REG_ACCEL_CONFIG	0x1C
#define MPU6050_REG_PWR_MGMT_1		0x6B
#define MPU6050_REG_PWR_MGMT_2		0x6C

//	mpu6050 accelerometer data registers
#define MPU6050_REG_ACCEL_XOUT_H	0x3B
#define MPU6050_REG_ACCEL_XOUT_L	0x3C
#define MPU6050_REG_ACCEL_YOUT_H	0x3D
#define MPU6050_REG_ACCEL_YOUT_L	0x3E
#define MPU6050_REG_ACCEL_ZOUT_H	0x3F
#define MPU6050_REG_ACCEL_ZOUT_L	0x40

//	mpu6050 gyroscope data registers
#define MPU6050_REG_GYRO_XOUT_H		0x43
#define MPU6050_REG_GYRO_XOUT_L		0x44
#define MPU6050_REG_GYRO_YOUT_H		0x45
#define MPU6050_REG_GYRO_YOUT_L		0x46
#define MPU6050_REG_GYRO_ZOUT_H		0x47
#define MPU6050_REG_GYRO_ZOUT_L		0x48

//	i2c bus settings
#define PI_I2C_BUS 			1
#define PI_I2C_FLAGS			0

int pi;

float mpu6050_accel_range = 16384.0;
float mpu6050_gyro_range = 131.0;

void pi_init(void);
void mpu6050_reg_config(int mpu_handle);
float get_imu_data(int mpu_handle, int imu_datatype);

int main(void)
{
    pi_init();

	int mpu6050_handle = i2c_open(pi, PI_I2C_BUS, MPU6050_ADDR, PI_I2C_FLAGS);

    for(int i = 3; i>0; i--)
    {
        std::cout << "Sending in " << i << " seconds" << std::endl;
        time_sleep(1);
    }

	//	write to the configuration registers
	mpu6050_reg_config(mpu6050_handle);
	time_sleep(0.2);

	//	declare imu data
	float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
	int ax = 0, ay = 1, az = 2, gx = 3, gy = 4, gz = 5;

    while(1)
    {
        //	get data from the imu
        accel_x = get_imu_data(mpu6050_handle, ax);
        accel_y = get_imu_data(mpu6050_handle, ay);
        accel_z = get_imu_data(mpu6050_handle, az);
        gyro_x = get_imu_data(mpu6050_handle, gx);
        gyro_y = get_imu_data(mpu6050_handle, gy);
        gyro_z = get_imu_data(mpu6050_handle, gz);

        //  print data to user
        std::cout << "Accelerometer (g's)    x: " << accel_x << ", y: " << accel_y << ", z: " << accel_z << std::endl;
        std::cout << "Gyroscope (deg/s)      x: " << gyro_x << ", y: " << gyro_y << ", z: " << gyro_z << std::endl << std::endl;

        time_sleep(.5);
    }

	i2c_close(pi, mpu6050_handle);
	pigpio_stop(pi);
	return 0;
}

void pi_init(void)
{
	char *addr_str = NULL;
	char *port_str = NULL;

	pi = pigpio_start(addr_str, port_str);

}

void mpu6050_reg_config(int mpu_handle)
{
	//	configure power management registers
	i2c_write_byte_data(pi, mpu_handle, MPU6050_REG_PWR_MGMT_1, 0x00);
	time_sleep(0.2);
	i2c_write_byte_data(pi, mpu_handle, MPU6050_REG_PWR_MGMT_2, 0x00);
	time_sleep(0.2);

	//	configure accel and gyro configuration registers
	i2c_write_byte_data(pi, mpu_handle, MPU6050_REG_ACCEL_CONFIG, 0x00);
	time_sleep(0.2);
	i2c_write_byte_data(pi, mpu_handle, MPU6050_REG_GYRO_CONFIG, 0x00);
	time_sleep(0.2);
}

float get_imu_data(int mpu_handle, int imu_datatype)
{
	float raw_data;
	float data;
	int data_reg_h, data_reg_l;

	//	get raw values from mpu registers for desired imu data
	if(imu_datatype == 0)
	{
        data_reg_h = i2c_read_byte_data(pi, mpu_handle, MPU6050_REG_ACCEL_XOUT_H);
        data_reg_l = i2c_read_byte_data(pi, mpu_handle, MPU6050_REG_ACCEL_XOUT_L);
	}
	else if(imu_datatype == 1)
	{
        data_reg_h = i2c_read_byte_data(pi, mpu_handle, MPU6050_REG_ACCEL_YOUT_H);
        data_reg_l = i2c_read_byte_data(pi, mpu_handle, MPU6050_REG_ACCEL_YOUT_L);
	}
	else if(imu_datatype == 2)
    {
        data_reg_h = i2c_read_byte_data(pi, mpu_handle, MPU6050_REG_ACCEL_ZOUT_H);
        data_reg_l = i2c_read_byte_data(pi, mpu_handle, MPU6050_REG_ACCEL_ZOUT_L);
    }
    else if(imu_datatype == 3)
    {
        data_reg_h = i2c_read_byte_data(pi, mpu_handle, MPU6050_REG_GYRO_XOUT_H);
        data_reg_l = i2c_read_byte_data(pi, mpu_handle, MPU6050_REG_GYRO_XOUT_L);
    }
    else if(imu_datatype == 4)
    {
        data_reg_h = i2c_read_byte_data(pi, mpu_handle, MPU6050_REG_GYRO_YOUT_H);
        data_reg_h = i2c_read_byte_data(pi, mpu_handle, MPU6050_REG_GYRO_YOUT_L);
    }
    else if(imu_datatype == 5)
    {
        data_reg_h = i2c_read_byte_data(pi, mpu_handle, MPU6050_REG_GYRO_ZOUT_H);
        data_reg_l = i2c_read_byte_data(pi, mpu_handle, MPU6050_REG_GYRO_ZOUT_L);
    }
    else
    {
        std::cout << "error imu-datatype " << imu_datatype << std::endl;
    }

	//	concatenate high and low values
	raw_data = (float)((int16_t)(data_reg_h<<8|data_reg_l));

	if(imu_datatype == 0 || imu_datatype == 1 || imu_datatype == 2)
	{
		data = raw_data/mpu6050_accel_range;
	}
	else
	{
		data = raw_data/mpu6050_gyro_range;
	}

	return data;

}















