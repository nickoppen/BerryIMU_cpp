#ifndef BERRYIMU_H
#define BERRYIMU_H

#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <string>
#include <thread>
#include <functional>

//#include <time.h>
#include <chrono> // Needed for waiting time required for ...
#include <thread> // ... temperature & pressure sensors

#include <unistd.h> // for close
extern "C"
{
	#include <i2c/smbus.h>
	#include <linux/i2c-dev.h>
	#include <linux/i2c.h>
	#include <sys/ioctl.h>
}

#ifdef __linux__
#include <linux/i2c-dev.h>
#else
#include "i2cdummy.h" // REPLACE
#endif

#include "berryimuDefinitions.h"

/*  Usage:
 * *                    REPLACE include i2cdummy.h by the installed i2c protocol. Needs to implement
 *                          ioctl(int, int, int);
				i2c_smbus_read_i2c_block_data(int,int,int,uint8_t*);
				i2c_smbus_read_byte_data(int,int);
				i2c_smbus_write_byte_data(int , int , int );

	This class provides the direct interaction with the IMU sensor.

 *  class BerryIMU:
 *  enableIMU() : activates i2c protocol and enables all sensors with the parameters set in
 *                  _accState, _magState, _gyrState, _configState
 *  configMag(..),configAcc(..),configGyr(..)
 *              : modify content of accState,magState,gyrState,configState and send it to the sensor
 *
 *  bool read(sensor_type type, int * output, bool reuse_device=false):
 *              : reads data to output. acc&mag can reuse_device
 *              : syntactic sugar: readACC,readMAG,readGYR
 *
 *  void readTandP(float &T, float &P): Reads temp & pressure, this function sleeps at least 10ms!!
 *  char * getLastMessage(): reads last generated message (warning error etc)
 *  Helper functions:
 *  getAltitude : implements barimetric formula
 *
 *
 *  This namespage defines a number of enums and constants which correspond to the allowed values of the hardware parameters
 *      and addresses
 *
 * *  Future changes:
 *                  - add modifier functions for all parameters which in turn call the adequate configX function
 *                  - CTRL_REG0_XM : set fifo mode
 *                  - CTRL_REG4_XM is used to set interrupt generators on INT2_XM
			Bits (7-0): P2_TAP P2_INT1 P2_INT2 P2_INTM P2_DRDYA P2_DRDYM P2_Overrun P2_WTM
			xmWriteByte(CTRL_REG4_XM, 0x04); // Magnetometer data ready on INT2_XM (0x08)
			- use https://github.com/sparkfun/SparkFun_LSM9DS0_Arduino_Library/blob/master/src/SFE_LSM9DS0.cpp

			  =>FIFO usage:
			  ==>https://www.raspberrypi.org/forums/viewtopic.php?t=111710&p=766487
 * */
namespace BerryIMU
{

	class IMU
	{
		// The following enums contain the bit flags which are used to set
		// the mode in the device

	public:
		IMU ()
		{
		}

		virtual ~IMU()
		{
			if (isEnabled())
				disableIMU();
		}

	public:
		bool getSensorID (uint8_t &chip_id, uint8_t &version)
		{
			uint8_t buffer[2];
			selectDevice(m_i2c_file, TP_ADDRESS);
			if (!i2c_smbus_read_i2c_block_data(TP_ADDRESS, 0xD0, 2, buffer))
			{
				setMessage("Error:  Could not read sensor ID and Version.");
				return false;
			}
			chip_id = buffer[1];
			version = buffer[0];
			return true;
		}

		bool isEnabled () { return (m_i2c_file >= 0); }

		const char* getLastMessage() { return m_message.c_str(); }

		protected:
		bool enableIMU ()
		{
			if (m_i2c_file >= 0) // If enabled, disable first
				disableIMU();

			char filename[20];
			sprintf(filename, "/dev/i2c-%d", 1);	/// << using port 1 (hardcoded!)
			m_i2c_file = open(filename, O_RDWR);
			if (m_i2c_file < 0)
			{
				setMessage("Unable to open I2C bus!");
				return false;
			}
			return true;
		}

		bool disableIMU ()
		{
			// TODO: add a power down to setting to stop the readings

			bool ret = close(m_i2c_file);
			if (ret)
				m_i2c_file = -1;
			return ret;
		}

		// if reuse_device, do not call select device. useful since MAG_ADDRESS==ACC_ADDRESS
		bool readRaw (const int registerOffset, int16_t *output, bool reuse_device = false)
		{
			// readBlock(0x80 | OUT_X_L_A, sizeof(block), block);
			/*
			An array of 6 bytes is first created to store the values.
			The I2C slave address is selected for the accelerometer, by passing the accelerometer address of ACC_ADDRESS or 0x1E  to the
			selectDevice() function. Using the readBlock() function from i2c-dev.h, we read 6 bytes starting at OUT_X_L_A (0x28). This
			is shown on page 61 of the datasheet. The values are expressed in 2’s complement (MSB for the sign and then 15 bits for the
			value) so we need to combine; block[0] & block[1] for X axis block[2] & block[3] for Y axis block[4] & block[5] for Z axis
			*/
			uint8_t register_address = READ_MULTIPLE_BYTES_FLAG;
			uint8_t block[6] = {0, 0, 0, 0, 0, 0};
			if (!reuse_device)
				selectDevice();
			register_address |= (uint8_t)registerOffset;
			if (readBlock(register_address, sizeof(block), block))
			{
				output[0] = (int16_t)(block[0] | block[1] << 8);
				output[1] = (int16_t)(block[2] | block[3] << 8);
				output[2] = (int16_t)(block[4] | block[5] << 8);
				return true;
			}
			return false;
		}


	  private:
		bool readBlock (uint8_t command, uint8_t size, uint8_t *data)
		{
			int result = i2c_smbus_read_i2c_block_data(m_i2c_file, command, size, data);
			if (result != size)
			{
				setMessage("Failed to read block from I2C.: %i");
				return false;
			}
			return true;
		}

		// return two bytes from data as a signed 16-bit value
		int16_t get_short(uint8_t* data, int index) { return ((data[index] << 8) + data[index + 1]); }
		uint16_t get_ushort(uint8_t* data, int index) { return ((data[index] << 8) + data[index + 1]); }

	protected:
		virtual float	gain() = 0;
		virtual bool	selectDevice() = 0;
		bool			selectDevice (int file, const int addr)
		{
			if (ioctl(file, I2C_SLAVE, addr) < 0)
			{
				setMessage("Failed to select I2C device.");
				return false;
			}
			return true;
		}
		void			readReg (uint8_t command, uint8_t &data) { data = i2c_smbus_read_byte_data(m_i2c_file, command); }
		uint8_t			readReg(uint8_t command) { return i2c_smbus_read_byte_data(m_i2c_file, command); }
		bool			writeReg (uint8_t reg, uint8_t value)
		{
			selectDevice();
			if (-1 == i2c_smbus_write_byte_data(m_i2c_file, reg, value))
			{
				setMessage("Failed to write byte to I2C.");
				return false;
			}
			return true;
		}
		uint8_t			writeBitPatternToReg(uint8_t reg, uint8_t bitPattern)
		{
			uint8_t c = readReg(reg);
			writeReg(reg, c | bitPattern);
			return readReg(reg);
		}
		void			setMessage (std::string msg) { m_message = msg; }
		void			usleep (int microseconds) { std::this_thread::sleep_for (std::chrono::microseconds (microseconds)); }
		void			msleep (int milliseconds) { usleep (1000 * milliseconds); }

		int m_i2c_file = -1;
		std::string m_message;
		configuration _configState;
		//call back function (initialised to null)

	};

	class fifoDevice
	{

	public:
		fifoDevice() {}

		virtual void enableFIFO() = 0;
		virtual void disableFIFO() = 0;
		virtual int retrieveFIFOData() = 0;
		virtual bool initiateDataReadyWithCallback(std::function<bool(int16_t*, int)> callBackFn) = 0;

		bool isFIFOEnabled() { return usingFifo; }

	protected:

		bool readDataAndExecuteCallback()
		{
			int xyzValuesRead;
			for (uint8_t i; i < 96; i++)
				fifoData[i] = 0;

			xyzValuesRead = this->retrieveFIFOData();

			// call the stored call back function
			return dataReadyCallBack(fifoData, xyzValuesRead);
		}

		static 	void pollThread(fifoDevice & device, int waitTime)
		{
			bool continuePolling = true;

			while (continuePolling && device.isFIFOEnabled())	// continuePolling is the response from the call back; isFifoEnabled returns false after disableFifo has been called 
			{
				//sleep for most of the time
				std::this_thread::sleep_for(std::chrono::milliseconds(waitTime));
				continuePolling = device.readDataAndExecuteCallback();	// testing - replace 32 with the actual number of values read
			}
		}

		bool usingFifo = false;
		int waitTime = 0;	// the amount of time the poll thread should wait before reading the fifo buffer
		int16_t fifoData[92];

		std::function<bool(int16_t*, int)> dataReadyCallBack = NULL;
		std::thread* pollingThread;
	};

	class Acc : public IMU, public fifoDevice
	{

	private:
		/// <summary>
		/// Private constructor to ensure that there is only one instance
		/// </summary>
		Acc() : IMU(), fifoDevice()
		{
			_accState.odr = A_ODR_200Hz;
			_accState.scale = A_SCALE_2g;
			_accState.aa_bw = A_BANDWIDTH_50Hz;
		}

	public:
		/// <summary>
		///  Return a reference to the singleton instance
		/// </summary>
		/// <returns></returns>
		static Acc& Get_Instance()
		{
			static Acc acc_instance;
			return acc_instance;
		}

		/// <summary>
		/// Ensure no copy constructors
		/// </summary>
		/// <param name=""></param>
		/// <returns></returns>
		Acc operator=(const Acc&) = delete;
		Acc(const Acc&) = delete;

		bool enable()
		{
			if (IMU::enableIMU())
			{
				// Enable accelerometer.
				configure(_accState.scale, _accState.aa_bw, _accState.selftest, _accState.spi_interface_mode);
				configure(_accState.odr, _accState.bdu, _accState.enableX, _accState.enableY, _accState.enableZ);
				return true;
			}
			return false;
		}
		bool disable()	
		{
			if (usingFifo)
				disableFIFO(); // kill the polling thread and wait for it to end

			return IMU::disableIMU();
		}

		bool read(int16_t * output)
		{
			return IMU::readRaw(OUT_X_L_A, output, false);
		}
		bool acceleration(float & xAcc, float & yAcc, float & zAcc, acceleration_units unit = MPSPS)
		{
			int16_t accData[3];
			float _gain = gain();

			if(IMU::readRaw(OUT_X_L_A, accData, false))
			{
				xAcc = (float)accData[0] * _gain;
				yAcc = (float)accData[1] * _gain;
				zAcc = (float)accData[2] * _gain;

				return true;
			}
			return false;
		}
		bool rawAcceleration(float& xAcc, float& yAcc, float& zAcc)
		{
			int16_t accData[3];

			if (IMU::readRaw(OUT_X_L_A, accData, false))
			{
				xAcc = (float)accData[0];
				yAcc = (float)accData[1];
				zAcc = (float)accData[2];

				return true;
			}
			return false;

		}

		void setDatarate(acc_odr datarate)
		{ // see mag
			_accState.odr = datarate;
			uint8_t value = readReg(CTRL_REG1_XM);
			value &= 0xFF ^ (0xF << 4);
			value |= (datarate << 4);
			writeReg(CTRL_REG1_XM, value);
		}

		void enableFIFO()  // add fifo mode selection
		{
			//uint8_t c;
			//c = readReg(CTRL_REG0_XM);
			//writeReg(CTRL_REG0_XM, c | 0x40);		
			writeBitPatternToReg(CTRL_REG0_XM, 0x40);// Enable acc FIFO
			msleep(20);								// Wait for change to take effect
			writeReg(FIFO_CTRL_REG, 0x20 | 0x1F);	// Enable acc FIFO mode (0x20) and set watermark at 32 samples (0x1F)
			usingFifo = true;
		}
		void disableFIFO()
		{
			uint8_t c;
			c = readReg(CTRL_REG0_XM);
			writeReg(CTRL_REG0_XM, c & ~0x40); // Disable acc FIFO
			msleep(20);
			writeReg(FIFO_CTRL_REG, 0x00); // Enable acc bypass mode
			usingFifo = false;
			pollingThread->join();
		}
		

		// fake the LSM9DS0's i2c data ready callback
		// the user's call back function must have the form void func(uint8_t *, int)
		virtual bool initiateDataReadyWithCallback(std::function<bool(int16_t *, int)> callBackFn)
		{
			if (usingFifo)
			{
				//uint8_t c;
				//c = readReg(CTRL_REG4_XM);
				//writeReg(CTRL_REG4_XM, c & 0x08);
				writeBitPatternToReg(CTRL_REG4_XM, 0x08);
				msleep(20);

				// store the call back address and start the polling loop
				dataReadyCallBack = std::bind(callBackFn, std::placeholders::_1, std::placeholders::_2);

				switch (_accState.odr)
				{
				case A_POWER_DOWN: // Power-down mode (0x0)
					return false;
					break;
				case A_ODR_3p125Hz:    // 3.125 Hz	(0x1)
					waitTime = (int)(32 * 1000 / 3.125);		// 32 samples in the FIFO buffer and 1000 to convert to milliseconds
					break;
				case A_ODR_6p25Hz:     // 6.25 Hz (0x2)
					waitTime = (int)(32 * 1000 / 6.25);
					break;
				case A_ODR_12p5Hz:     // 12.5 Hz (0x3)
					waitTime = (int)(32 * 1000 / 12.5);
					break;
				case A_ODR_25Hz:       // 25 Hz (0x4)
					waitTime = (int)(32 * 1000 / 25);
					break;
				case A_ODR_50Hz:       // 50 Hz (0x5)
					waitTime = (int)(32 * 1000 / 50);
					break;
				case A_ODR_100Hz:      // 100 Hz (0x6)
					waitTime = (int)(32 * 1000 / 100);
					break;
				case A_ODR_200Hz:      // 200 Hz (0x7)
					waitTime = (int)(32 * 1000 / 200);
					break;
				case A_ODR_400Hz:      // 400 Hz (0x8)
					waitTime = (int)(32 * 1000 / 400);
					break;
				case A_ODR_800Hz:      // 800 Hz (9)
					waitTime = (int)(32 * 1000 / 800);
					break;
				case A_ODR_1600Hz:      // 1600 Hz (0xA)
					waitTime = (int)(32 * 1000 / 1600);
					break;
				default:
					return false;
				}

				pollingThread = new std::thread(pollThread, std::ref(*this), waitTime);
				return true;
			}
			else
				return false;
		}

	protected:
		float gain()
		{
			// Possible gains
			switch (_accState.scale)
			{
			case A_SCALE_2g:
				return 0.061f;
			case A_SCALE_4g:
				return 0.122f;
			case A_SCALE_6g:
				return 0.183f;
			case A_SCALE_8g:
				return 0.244f;
			case A_SCALE_16g:
				return 0.732f;
			default:
				throw (2);
			}
		}

		bool selectDevice()
		{
			return IMU::selectDevice(m_i2c_file, ACC_ADDRESS);
		}

		virtual int retrieveFIFOData()
		{
			uint8_t command = FIFO_SRC_REG;
			// see page 62 of the data sheet
			int numberOfValuesInFIFO = (int)(readReg(command) & 0x1F);
			
			for (int i = 0; i < numberOfValuesInFIFO; i++)
			{
				IMU::readRaw(OUT_X_L_A, &(fifoDevice::fifoData[i]), true);
			}

			// Read number of stored samples. They can be accessed using a for loop over read(...)
			return (numberOfValuesInFIFO);
		}


	private:
		bool configure(acc_odr datarate = A_ODR_3p125Hz, acc_bdu bdu = A_CONTINUOUS_UPDATE, bool enableX = true, bool enableY = true, bool enableZ = true)
		{
			// Configuration according to Tab. 71
			// Bitmasking just in case
			// bdu : Block data update for acceleration and magnetic data. Default value: 0
			//       (0: continuous update; 1: output registers not updated until MSB and LSB have been read)
			_accState.odr = datarate;
			_accState.bdu = bdu;
			_accState.enableX = enableX;
			_accState.enableY = enableY;
			_accState.enableZ = enableZ;

			uint8_t value;
			value = 0b1 & enableX;
			value |= (0b1 & enableY) << 1;
			value |= (0b1 & enableZ) << 2;
			value |= (0b1 & bdu) << 3;
			value |= (0b1111 & datarate) << 4;
			return writeReg(CTRL_REG1_XM, value);
		}
		bool configure(acc_scale scale = A_SCALE_2g, acc_aa_bandwidth anti_alias_bandwidth = A_BANDWIDTH_773Hz, acc_selftest_mode selftest = A_TEST_OFF, bool spi_interface_mode = 0)
		{
			// Configuration according to Tab. 74
			// Bitmasking just in case
			_accState.scale = scale;
			_accState.selftest = selftest;
			_accState.aa_bw = anti_alias_bandwidth;
			_accState.spi_interface_mode = spi_interface_mode;

			uint8_t value = 0;
			value = 1 & spi_interface_mode;
			value |= (0b11 & selftest) << 1;
			value |= (0b111 & scale) << 3;
			value |= (0b11 & anti_alias_bandwidth) << 6;

			return writeReg(CTRL_REG2_XM, value);
		}

		AccState _accState;

	};	// Class Acc

	class Gyr : public IMU, public fifoDevice
	{

	/// <summary>
	/// Private contructor for a Singleton class
	/// </summary>
	private:
		Gyr() : IMU(), fifoDevice()
		{
			_gyrState.scale = G_SCALE_2000dps;
			_gyrState.odr = G_ODR_190_BW_125;
			_gyrState.highpasscutoff = G_HIGH_MAX;
		}

	public:
		/// <summary>
		/// Provides access to the single static instance of Gyr
		/// </summary>
		/// <returns></returns>
		static Gyr& Get_Instance()
		{
			static Gyr instance_gyr;
			return instance_gyr;
		}

		/// <summary>
		/// Enforces use of a reference disallowing copy constructors
		/// </summary>
		/// <param name=""></param>
		/// <returns></returns>
		Gyr operator=(const Gyr&) = delete;
		Gyr(const Gyr&) = delete;

		/// End Singleton 

		/// <summary>
		/// 
		/// </summary>
		/// <returns></returns>
		bool enable()
		{
			if (IMU::enableIMU())
			{
				// Enable Gyro
				configure(_gyrState.scale, _gyrState.bdu, _configState.bigendian, _gyrState.selftest, _gyrState.spi_interface_mode);
				configure(_gyrState.highpassmode, _gyrState.highpasscutoff);
				configure(_gyrState.odr, _gyrState.power, _gyrState.enableX, _gyrState.enableY, _gyrState.enableZ);
				return true;
			}
			return false;
		}
		bool disable()
		{
			return IMU::disableIMU();
		}

		bool read(int16_t * output)
		{
			return IMU::readRaw(OUT_X_L_G, output, false);
		}
		bool rotation(float &xRate, float &yRate, float &zRate, rotation_units unit = DEGPERSEC)
		{
			int16_t rotData[6];			// 12 bytes

			// 2000bps to degrees: 0.07		/// according to the ozzmaker web site
			// conversions used below are empirical using a turntable at 33 1/3 45 and 78 rpm
			// biases are based on 1200 reading taken when gyro is stationary
			float rawConversionAt2000dps[3];
			float biasAt2000dps[3];

			if (unit == DEGPERSEC)
			{
				 rawConversionAt2000dps[0] = 0.069065f;
				 rawConversionAt2000dps[1] = 0.068585f;
				 rawConversionAt2000dps[2] = 0.066168f;
			}
			else
			{
				if (unit == RADPERSEC)
				{
					rawConversionAt2000dps[0] = 0.00120525f;
					rawConversionAt2000dps[1] = 0.001196952f;
					rawConversionAt2000dps[2] = 0.00115482f;
				}
				else
				{
					if (unit == RPM)
					{
						rawConversionAt2000dps[0] = 0.0115094f;
						rawConversionAt2000dps[1] = 0.0114299f;
						rawConversionAt2000dps[2] = 0.0110277f;
					}
					else
						throw (2);	// probably not 2
				}
			}

			biasAt2000dps[0] = -11.406666f;
			biasAt2000dps[1] = -65.157500f;
			biasAt2000dps[2] = 84.860833f;


			clock_gettime(CLOCK_REALTIME, &lastReadingTime);	// Store the reading time to calculate the degrees turned on the next call
			IMU::readRaw(OUT_X_L_G, rotData, false);

			xRate = ((float)rotData[0] + biasAt2000dps[0]) * rawConversionAt2000dps[0];
			yRate = ((float)rotData[1] + biasAt2000dps[1]) * rawConversionAt2000dps[1];
			zRate = ((float)rotData[2] + biasAt2000dps[2]) * rawConversionAt2000dps[2];
			return true;
		}
		bool rawRotation(float& xRawRate, float& yRawRate, float& zRawRate)
		{
			int16_t rotData[6];			// 12 bytes
			IMU::readRaw(OUT_X_L_G, rotData, false);

			xRawRate = (float)rotData[0];
			yRawRate = (float)rotData[1];
			zRawRate = (float)rotData[2];
			return true;
		}

		void setDatarate(gyro_odr datarate)
		{
			_gyrState.odr = datarate;
			uint8_t value = readReg(CTRL_REG1_G);
			value &= 0xFF ^ (0xF << 4);
			value |= (datarate << 4);
			writeReg(CTRL_REG1_G, value);
		}

		void enableFIFO()		// Add fifo mode to include all modes (except ByPass which disables fifo)
		{
			//uint8_t c;
			//c = readReg(CTRL_REG5_G);
			//writeReg(CTRL_REG5_G, c | 0x40);		
			writeBitPatternToReg(CTRL_REG5_G, 0x40); // Enable gyro FIFO (see page 44 section 8.6)
			msleep(20);								// Wait for change to take effect
			writeReg(FIFO_CTRL_REG_G, 0x20 | 0x1F); // Enable gyro FIFO mode (0x20) and set watermark at 32 samples (0x1F)
			usingFifo = true;
		}
		void disableFIFO()
		{
			uint8_t c;
			c = readReg(CTRL_REG5_G);
			writeReg(CTRL_REG5_G, c & ~0x40); // Disable gyro FIFO
			msleep(20);
			writeReg(FIFO_CTRL_REG_G, 0x00); // Enable gyro bypass mode
			usingFifo = false;
		}

		// fake the LSM9DS0's i2c data ready callback
		// the user's call back function must have the form void func(uint8_t *, int)
		virtual bool initiateDataReadyWithCallback(std::function<bool(int16_t*, int)> callBackFn)
		{
			if (usingFifo)
			{
				//uint8_t c;
				//c = readReg(CTRL_REG4_XM);
				//writeReg(CTRL_REG4_XM, c & 0x08);
				writeBitPatternToReg(CTRL_REG4_XM, 0x08);
				msleep(20);

				// store the call back address and start the polling loop
				dataReadyCallBack = std::bind(callBackFn, std::placeholders::_1, std::placeholders::_2);

				switch (_gyrState.odr)
				{								// value		ODR (Hz)	cutoff for filter
				case G_ODR_95_BW_125:			// = 0x0,	//   95         12.5
				case G_ODR_95_BW_25:			// = 0x1,	//   95          25								// 0x2 and 0x3 define the same data rate and bandwidth as 0x1
					waitTime = (int)(32 * 1000 / 95);
					break;
				case G_ODR_190_BW_125:			// = 0x4,	//   190        12.5
				case G_ODR_190_BW_25:			// = 0x5,	//   190         25
				case G_ODR_190_BW_50:			// = 0x6,	//   190         50
				case G_ODR_190_BW_70:			// = 0x7,	//   190         70
					waitTime = (int)(32 * 1000 / 190);
					break;
				case G_ODR_380_BW_20:			// = 0x8,	//   380         20
				case G_ODR_380_BW_25:			// = 0x9,	//   380         25
				case G_ODR_380_BW_50:			// = 0xA,	//   380         50
				case G_ODR_380_BW_100:			// = 0xB,	//   380         100
					waitTime = (int)(32 * 1000 / 380);
					break;
				case G_ODR_760_BW_30:			// = 0xC,	//   760         30
				case G_ODR_760_BW_35:			// = 0xD,	//   760         35
				case G_ODR_760_BW_50:			// = 0xE,	//   760         50
				case G_ODR_760_BW_100:			// = 0xF,	//   760         100
					waitTime = (int)(32 * 1000 / 760);
					break;
				default:
					return false;
				}

				fifoDevice::pollingThread = new std::thread(pollThread, std::ref(*this), waitTime);
				return true;
			}
			else
				return false;
		}
		virtual int retrieveFIFOData()
		{
			uint8_t command = FIFO_SRC_REG_G;
			int numberOfValuesInFIFO = (int)(readReg(command) & 0x1F);
			for (int i = 0; i < numberOfValuesInFIFO; i++)
			{
				IMU::readRaw(OUT_X_L_G, &(fifoDevice::fifoData[i]), true);
			}
			return numberOfValuesInFIFO;
		}

	protected:
		float gain()
		{
			switch (_gyrState.scale)
			{
			case G_SCALE_250dps:
				return 8.75f;//invert
			case G_SCALE_500dps:
				return 17.50f;//invert
			case G_SCALE_2000dps:
				return 0.07f;
			default:
				throw (2);
			}
		}

		bool selectDevice()
		{
			return IMU::selectDevice(m_i2c_file, GYR_ADDRESS);
		}

	private:
		bool configure(gyro_odr odr = G_ODR_95_BW_125, gyr_power_mode pd = G_POWER_NORMAL, bool enableX = true, bool enableY = true, bool enableZ = true)
		{
			// dr & bw set the data rate (ODR) and cutoff for low-pass filter (Cutoff)
			_gyrState.odr = odr;
			_gyrState.power = pd;
			_gyrState.enableX = enableX;
			_gyrState.enableY = enableY;
			_gyrState.enableZ = enableZ;
			uint8_t value;
			if (pd == G_POWER_SLEEP)
			{ // will be overwritten acc. to Tab. 22
				enableX = 0;
				enableY = 0;
				enableZ = 0;
			}
			value = 0b1 & enableY;
			value |= (0b1 & enableX) << 1;
			value |= (0b1 & enableZ) << 2;
			value |= (0b1 & pd) << 3;
			value |= (0b1111 & odr) << 4;
			return writeReg(CTRL_REG1_G, value);
		}
		bool configure(HighPassMode hpm, gyr_high_pass hpcf)
		{
			// Tab. 23
			_gyrState.highpassmode = hpm;
			_gyrState.highpasscutoff = hpcf;
			uint8_t value;
			value = 0b1111 & hpcf;
			value |= (0b11 & hpm) << 4;
			return writeReg(CTRL_REG2_G, value);
		}
		bool configure(gyr_scale scale = G_SCALE_500dps, gyr_bdu bdu = G_CONTINUOUS_UPDATE, bool bigEndian = false, gyr_selftest_mode selftest = G_TEST_OFF, bool spi_interface_mode = 0)
		{
			// bdu : Block data update def=0 (0: continuous update; 1: output registers not updated until MSB and LSB have been read)
			// spi: 0=4 wire interface, 1=3 wire interface
			_gyrState.scale = scale;
			_gyrState.selftest = selftest;
			_gyrState.bdu = bdu;
			_gyrState.spi_interface_mode = spi_interface_mode;
			_configState.bigendian = bigEndian;

			uint8_t value = 0;
			value = 1 & spi_interface_mode;
			value |= (0b11 & selftest) << 1;
			value |= (0b11 & scale) << (3 + 1); // 0bit at position 4
			value |= (0b1 & bigEndian) << 6;
			value |= (0b1 & bdu) << 7;
			return writeReg(CTRL_REG4_G, value);
			// writeReg(GYR,CTRL_REG4_G, 0b0 0 11 0 00 0); // Continuos update, 2000 dps full scale
		}

		GyrState _gyrState;
		rotation_units unit = DEGPERSEC;
		struct timespec lastReadingTime;
	};	//Class Gyr

	class Mag : IMU
	{
	private:
		Mag() : IMU()
		{
			_magState.scale = M_SCALE_2Gs;
			_magState.odr = M_ODR_100Hz;
		}

	public:
		static Mag& Get_Instance()
		{
			static Mag mag_instance;
			return mag_instance;
		}

		Mag(const Mag&) = delete;
		Mag operator=(const Mag&) = delete;

		bool enable()
		{
			if (IMU::enableIMU())
			{
				// Enable the magnetometer
				configure(_magState.highpass, _magState.filter, _magState.sensormode, _magState.power);
				configure(_magState.scale);
				configure(_magState.odr, _magState.resolution, _configState.temperature_sensor_activated);
				return true;
			}
			return false;
		}
		bool disable()
		{
			return IMU::disableIMU();
		}

		bool read(int16_t* output)
		{
			return IMU::readRaw (OUT_X_L_M, output, false); 
		}

	protected:

		void setDatarate(mag_odr datarate)
		{
			_magState.odr = datarate;
			uint8_t value = readReg(CTRL_REG5_XM);
			// Then mask out the mag ODR bits:
			value &= 0xFF ^ (0b111 << 2); // &= 0b11100011
			// Then shift in our new ODR bits:
			value |= (datarate << 2);
			// And write the new register value back into CTRL_REG5_XM:
			writeReg(CTRL_REG5_XM, value);
		}

		float gain()
		{
			switch (_magState.scale)
			{
			case M_SCALE_2Gs:
				return 0.08f;
			case M_SCALE_4Gs:
				return 0.16f;
			case M_SCALE_8Gs:
				return 0.32f;
			case M_SCALE_12Gs:
				return 0.48f;
			default:
				throw (2);
			}
		}

		bool selectDevice()
		{
			return IMU::selectDevice(m_i2c_file, MAG_ADDRESS);
		}

	private:

		bool configure(mag_odr datarate, mag_resolution mag_resolution = M_HIGH_RES, bool temperature = true, bool latch_interrupt_on_int1_src = 0, bool latch_interrupt_on_int2_src = 0)
		{
			// Configuration according to Tab. 83
			// Bitmasking just in case
			_magState.odr = datarate;
			_magState.resolution = mag_resolution;
			_configState.temperature_sensor_activated = temperature;

			uint8_t value;
			value = 0b1 & latch_interrupt_on_int1_src;
			value |= (0b1 & latch_interrupt_on_int2_src) << 1;
			value |= (0b111 & datarate) << 2;
			value |= (0b11 & mag_resolution) << 5;
			value |= (0b1 & temperature) << 7;
			return writeReg(CTRL_REG5_XM, value);
			// writeReg(CTRL_REG5_XM, 0b1 11 100 00);   // Temp enable, high res, Mag data rate = 50Hz
		}

		bool configure(mag_scale scale)
		{
			// Configuration according to Tab. 85
			// Bitmasking just in case
			_magState.scale = scale;
			uint8_t value;
			value = (0b11 & scale) << 5;
			return writeReg(CTRL_REG6_XM, value);
		}

		bool configure(HighPassMode highpass, mag_filter_acceleration filter = M_FILTER_INTERNAL_BYPASSED,
			mag_sensor_mode sensormode = M_SENSOR_CONTINUOUS_CONVERSION, mag_power_mode powermode = M_POWER_HIGH)
		{
			// Configuration according to Tab. 88
			// Bitmasking just in case
			_magState.sensormode = sensormode;
			_magState.power = powermode;
			_magState.filter = filter;
			_magState.highpass = highpass;
			uint8_t value;
			if (powermode == M_POWER_LOW && _magState.odr != M_ODR_3p125Hz) // this is automatically set by sensor
				setMessage("Mag datarate has been temporarily changed due to power saving mode.");
			value = (0b11 & sensormode);
			value |= (0b1 & powermode) << 2;
			value |= (0b1 & filter) << 5; // gap for 2 zero bits
			value |= (0b11 & highpass) << 6;
			return writeReg(CTRL_REG7_XM, value);
		}


		MagState _magState;


	};	// Class Mag

	class TP : IMU
	{
	public:
		TP() : IMU()
		{
			_configState.temperature_sensor_activated = false;
		}
		bool enable()
		{
			return IMU::enableIMU();
		}
		bool disable()
		{
			return IMU::disableIMU();
		}

		bool enableTemperatureSensor()
		{
			throw (0);
//			return configureMag(_magState.odr, _magState.resolution, true);
		}
		bool disableTemperatureSensor()
		{
			throw (0);
//			return configureMag(_magState.odr, _magState.resolution, false);
		}

		void readTandP(float& T, float& P, bool reuse_device = false, bool only_T = false, bool reuse_T = false)
		{
//#ifdef _WIN32
			return;
//#endif
/*			//# Print temperature & pressure & chip data
			int oversampling = _configState.oversampling;
			if (!reuse_device)
				selectDevice(TP);

			//# Read whole calibration EEPROM data [only 1x]
			const int BUFFER_SIZE = 22;
			static uint8_t cal[BUFFER_SIZE];
			static bool calibrated = false;
			// BMP180_COMMAND_TEMPERATURE (0x2E) to the register BMP180_REG_CONTROL (0xF4)
			// BMP180_COMMAND_PRESSURE (0xF4) to the register BMP180_REG_CONTROL (0xF4)
			if (!calibrated)
			{
				calibrated = 0 < i2c_smbus_read_i2c_block_data(m_i2c_file, 0xAA, BUFFER_SIZE, cal);
				// qDebug() << "BMP190 sensor calibration data read.";
			}

			// cal = read_i2c_block_data(addr, 0xAA, BUFFER_SIZE);
			//# Convert byte data to word values
			int16_t ac1 = get_short(cal, 0);
			int16_t ac2 = get_short(cal, 2);
			int16_t ac3 = get_short(cal, 4);
			uint16_t ac4 = get_ushort(cal, 6);
			uint16_t ac5 = get_ushort(cal, 8);
			uint16_t ac6 = get_ushort(cal, 10);
			int16_t b1 = get_short(cal, 12);
			int16_t b2 = get_short(cal, 14);
			// int16_t mb = get_short(cal, 16); // not used. is =-32768
			int16_t mc = get_short(cal, 18);
			int16_t md = get_short(cal, 20);

			uint8_t msb, lsb, xsb;
			int32_t b5, x1, x2, t, ut;
			if (!reuse_T)
			{
				// "Starting temperature conversion..."
				// request data
				i2c_smbus_write_byte_data(m_i2c_file, BMP180_CTRL, BMP180_COMMAND_TEMPERATURE);
				// wait while data is getting ready
				usleep(getWaitingTimeTemperature());

				// read temperature results
				//        (msb, lsb) = bus.read_i2c_block_data(addr, 0xF6, 2)
				msb = i2c_smbus_read_byte_data(m_i2c_file, BMP180_REG_PRE) & 0xFF;
				lsb = i2c_smbus_read_byte_data(m_i2c_file, BMP180_REG_PRE + 1) & 0xFF;
				if (msb == 0 && lsb == 0)
				{ // to avoid exception div/0 if read goes wrong
					setMessage("Error: BMP180 temperature read unsuccessful");
					return;
				}
				ut = (msb << 8) + lsb;
				//"Calculating temperature..."
				x1 = ((ut - ac6) * ac5) >> 15;
				x2 = round((mc << 11) / (x1 + md));
				b5 = (x1 + x2);
				t = (b5 + 8) >> 4;
				T = t / 10.0; //# [C]
				m_last_temperature_reading = T;
				if (only_T)
					return;
			}

			//"Starting pressure conversion..."
			// request data
			i2c_smbus_write_byte_data(m_i2c_file, BMP180_CTRL, BMP180_COMMAND_PRESSURE + (oversampling << 6));
			// wait while data is getting ready
			usleep(getWaitingTimePressure(this->_configState.oversampling));
			//(msb, lsb, xsb) = bus.read_i2c_block_data(addr, BMP180_REG_PRE, 3)// from python, works there!
			// i2c_smbus_read_i2c_block_data(m_i2c_file,BMP180_REG_PRE,3,data);
			msb = i2c_smbus_read_byte_data(m_i2c_file, BMP180_REG_PRE) & 0xFF;
			lsb = i2c_smbus_read_byte_data(m_i2c_file, BMP180_REG_PRE + 1) & 0xFF;
			xsb = i2c_smbus_read_byte_data(m_i2c_file, BMP180_REG_PRE + 2) & 0xFF;

			int32_t up = ((msb << 16) + (lsb << 8) + xsb) >> (8 - oversampling);

			// print "Calculating pressure..."
			if (reuse_T)
				b5 = ((static_cast<int> (T * 10)) << 4) - 8;
			int32_t b6 = b5 - 4000;
			int32_t b62 = (b6 * b6) >> 12;
			x1 = (b2 * b62) >> 11;
			x2 = (ac2 * b6) >> 11;
			int32_t x3 = x1 + x2;
			int32_t b3 = (((ac1 * 4 + x3) << oversampling) + 2) >> 2;

			x1 = (ac3 * b6) >> 13;
			x2 = (b1 * b62) >> 16;
			x3 = ((x1 + x2) + 2) >> 2;
			int32_t b4 = (ac4 * (x3 + 32768)) >> 15;
			uint32_t b7 = ((unsigned long)up - b3) * (50000 >> oversampling);

			int32_t p;
			if (b7 < 0x80000000)
				p = (b7 * 2) / b4;
			else
				p = (b7 / b4) * 2;
			x1 = (p >> 8) * (p >> 8);
			x1 = (x1 * 3038) >> 16;
			x2 = (-7357 * p) >> 16;
			p = p + ((x1 + x2 + 3791) >> 4);

			P = p / 100.0; // [hPa]
			//        qDebug()<<" " <<b5;
			//        throw("END");
*/
		}

		void readP(float& P, bool reuse_device = false)
		{
			// void readTandP(float & T, float & P,bool reuse_device, bool only_T=false, bool reuse_T=false);
			throw (0);
			//			readTandP(m_last_temperature_reading, P, reuse_device, false, true);
		}


		void readT(float& T, bool reuse_device = false)
		{
//			float discard;
			throw (0);
			//			readTandP(T, discard, reuse_device, false);
		}

		void readTlsm(float& T) // reads T from LSM9DS0
		{
			throw (0);
//			if (_configState.temperature_sensor_activated)
			//{
			//	uint8_t buffer[2]; // We'll read two bytes from the temperature sensor into temp
			//	selectDevice(TP); // Register lies with magnetometer
			//	if (!readBlock(READ_MULTIPLE_BYTES_FLAG | OUT_TEMP_L_XM, 2, buffer))
			//	{
			//		setMessage("Error:readTlsm::readBlock returned 0 bytes.");
			//		return;
			//	}
			//	int8_t lo = buffer[0];
			//	int16_t hi = buffer[1];
			//	T = static_cast<float> (((hi << 8) | lo) >> 3) + 25.;
			//	setMessage(std::string("OUT_TEMP_L_XM:") + std::to_string((hi << 8) | lo) + " "
			//		+ std::to_string((hi << 8) | lo));
			//	// T = (float)temp / 8.0 + 25.; // celsius // 25 is undocumented
			//	// float temperature_f = temperature_c * 1.8 + 32.;
			//}
		}

		float gain() { return -1.0; }

		bool selectDevice ()
		{
//			int addr;
//			addr = TP_ADDRESS;
			return IMU::selectDevice(m_i2c_file, TP_ADDRESS);
//			if (ioctl(m_i2c_file, I2C_SLAVE, addr) < 0)
//			{
//				setMessage("Failed to select I2C device.");
//				return false;
//			}
//			return true;
		}

		inline int getWaitingTimeTemperature ()
		{
			// Waiting time in us for reading temperature values
			return 4500; // time.sleep(0.0045) was in python
		}

		int getWaitingTimePressure (pressure_oversampling oversampling)
		{
			/*
			 * Waiting times in us for reading pressure values
			 */
			const int extra_time = 0; // To check wether I dnt wait enough
			switch (oversampling)
				{
					case 0:
						return extra_time + 4500;
					case 1:
						return extra_time + 7500;
					case 2:
						return extra_time + 13500;
					case 3:
						return extra_time + 25500;
				}
			return -1;
		}


	private:

	static double getAltitude (float pressure)
	{
		// uses barometric formula to calculate altitude in [m] from [mBar]
		// pressure at sea level 	const float airPressureAtSeaLevel = 1013.25;
		return 44330. * (1 - pow (pressure / airPressureAtSeaLevel, 1. / 5.255)); 	// international barometric formula
	}

		float m_last_temperature_reading = 0.0;

	};	// Class TP


} // namespace BerryIMU
#endif // BERRYIMU_H

