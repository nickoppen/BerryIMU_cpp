#ifndef BERRYIMU_DEFINITIONS_H
#define BERRYIMU_DEFINITIONS_H

namespace BerryIMU
{
	//------------------------------------------------
	// Pressure & Temp sensor BMP180
	//------------------------------------------------
	const int BMP180_CTRL = 0xF4;    // AC register
	const int BMP180_REG_PRE = 0xF6; // * Pressure register
	const int BMP180_COMMAND_TEMPERATURE = 0x2E;
	const int BMP180_COMMAND_PRESSURE = 0x34;
	//-----------------------------------------------------------------------
	// Constants to interact with LSM9DS0
	//-----------------------------------------------------------------------
	// I2C addresses
	const int MAG_ADDRESS = 0x1E; // use i2c detection tool to determine
	const int ACC_ADDRESS = 0x1E;
	const int GYR_ADDRESS = 0x6A;
	const int TP_ADDRESS = 0x77;

	const int READ_MULTIPLE_BYTES_FLAG = 0x80; // See Sec.6.1.1 datasheet

	//------------------------------------------------
	/**LSM9DS0GyroRegisters**/

	const int WHO_AM_I_G = 0x0F;
	const int CTRL_REG1_G = 0x20;
	const int CTRL_REG2_G = 0x21;
	const int CTRL_REG3_G = 0x22;
	const int CTRL_REG4_G = 0x23;
	const int CTRL_REG5_G = 0x24;
	const int REFERENCE_G = 0x25;
	const int STATUS_REG_G = 0x27;
	const int OUT_X_L_G = 0x28;
	const int OUT_X_H_G = 0x29;
	const int OUT_Y_L_G = 0x2A;
	const int OUT_Y_H_G = 0x2B;
	const int OUT_Z_L_G = 0x2C;
	const int OUT_Z_H_G = 0x2D;
	const int FIFO_CTRL_REG_G = 0x2E;
	const int FIFO_SRC_REG_G = 0x2F;
	const int INT1_CFG_G = 0x30;
	const int INT1_SRC_G = 0x31;
	const int INT1_THS_XH_G = 0x32;
	const int INT1_THS_XL_G = 0x33;
	const int INT1_THS_YH_G = 0x34;
	const int INT1_THS_YL_G = 0x35;
	const int INT1_THS_ZH_G = 0x36;
	const int INT1_THS_ZL_G = 0x37;
	const int INT1_DURATION_G = 0x38;

	//------------------------------------------------
	// LSM9DS0Accel/Magneto(XM)Registers//

	const int OUT_TEMP_L_XM = 0x05;
	const int OUT_TEMP_H_XM = 0x06;
	const int STATUS_REG_M = 0x07;
	const int OUT_X_L_M = 0x08;
	const int OUT_X_H_M = 0x09;
	const int OUT_Y_L_M = 0x0A;
	const int OUT_Y_H_M = 0x0B;
	const int OUT_Z_L_M = 0x0C;
	const int OUT_Z_H_M = 0x0D;
	const int WHO_AM_I_XM = 0x0F;
	const int INT_CTRL_REG_M = 0x12;
	const int INT_SRC_REG_M = 0x13;
	const int INT_THS_L_M = 0x14;
	const int INT_THS_H_M = 0x15;
	const int OFFSET_X_L_M = 0x16;
	const int OFFSET_X_H_M = 0x17;
	const int OFFSET_Y_L_M = 0x18;
	const int OFFSET_Y_H_M = 0x19;
	const int OFFSET_Z_L_M = 0x1A;
	const int OFFSET_Z_H_M = 0x1B;
	const int REFERENCE_X = 0x1C;
	const int REFERENCE_Y = 0x1D;
	const int REFERENCE_Z = 0x1E;
	const int CTRL_REG0_XM = 0x1F;
	const int CTRL_REG1_XM = 0x20;
	const int CTRL_REG2_XM = 0x21;
	const int CTRL_REG3_XM = 0x22;
	const int CTRL_REG4_XM = 0x23;
	const int CTRL_REG5_XM = 0x24;
	const int CTRL_REG6_XM = 0x25;
	const int CTRL_REG7_XM = 0x26;
	const int STATUS_REG_A = 0x27;
	const int OUT_X_L_A = 0x28;
	const int OUT_X_H_A = 0x29;
	const int OUT_Y_L_A = 0x2A;
	const int OUT_Y_H_A = 0x2B;
	const int OUT_Z_L_A = 0x2C;
	const int OUT_Z_H_A = 0x2D;
	const int FIFO_CTRL_REG = 0x2E;
	const int FIFO_SRC_REG = 0x2F;
	const int INT_GEN_1_REG = 0x30;
	const int INT_GEN_1_SRC = 0x31;
	const int INT_GEN_1_THS = 0x32;
	const int INT_GEN_1_DURATION = 0x33;
	const int INT_GEN_2_REG = 0x34;
	const int INT_GEN_2_SRC = 0x35;
	const int INT_GEN_2_THS = 0x36;
	const int INT_GEN_2_DURATION = 0x37;
	const int CLICK_CFG = 0x38;
	const int CLICK_SRC = 0x39;
	const int CLICK_THS = 0x3A;
	const int TIME_LIMIT = 0x3B;
	const int TIME_LATENCY = 0x3C;
	const int TIME_WINDOW = 0x3D;

	const float airPressureAtSeaLevel = 1013.25;

	// ----------------------------------------------------
	// The following constants are understood by the sensor
	// ----------------------------------------------------
	enum acc_selftest_mode
	{
		A_TEST_OFF = 0,
		A_TEST_POSITIVE_SIGN = 1,
		A_TEST_NEGATIVE_SIGN = 2,
		A_TEST_THIS_IS_NOT_ALLOWED = 3
	}; // for acc
	enum gyr_selftest_mode
	{
		G_TEST_OFF = 0,
		G_TEST_XPositiveYZNegative = 1,
		G_TEST_XNegativeYZPositive = 3
	}; // For gyr

	// see: Table 75. Acceleration anti-alias filter bandwidth
	enum acc_aa_bandwidth
	{
		A_BANDWIDTH_773Hz = 0,
		A_BANDWIDTH_194Hz = 1,
		A_BANDWIDTH_362Hz = 2,
		A_BANDWIDTH_50Hz = 3
	};
	enum acc_scale
	{
		A_SCALE_2g = 0, // maximal range of accelerometer ( in multiples of g=9.81m/s^2 )
		A_SCALE_4g = 1,
		A_SCALE_6g = 2,
		A_SCALE_8g = 3,
		A_SCALE_16g = 4
	};

	// acc_odr defines all possible output data rates of the accelerometer:
	enum acc_odr
	{
		A_POWER_DOWN = 0, // Power-down mode (0x0)
		A_ODR_3p125Hz,    // 3.125 Hz	(0x1)
		A_ODR_6p25Hz,     // 6.25 Hz (0x2)
		A_ODR_12p5Hz,     // 12.5 Hz (0x3)
		A_ODR_25Hz,       // 25 Hz (0x4)
		A_ODR_50Hz,       // 50 Hz (0x5)
		A_ODR_100Hz,      // 100 Hz (0x6)
		A_ODR_200Hz,      // 200 Hz (0x7)
		A_ODR_400Hz,      // 400 Hz (0x8)
		A_ODR_800Hz,      // 800 Hz (9)
		A_ODR_1600Hz      // 1600 Hz (0xA)
	};

	enum mag_odr
	{
		M_ODR_3p125Hz = 0,
		M_ODR_6p25Hz,
		M_ODR_12p5Hz = 2, // table 84
		M_ODR_25Hz,
		M_ODR_50Hz = 4,
		M_ODR_100Hz
	};
	enum mag_scale
	{
		M_SCALE_2Gs = 0,
		M_SCALE_4Gs,
		M_SCALE_8Gs,
		M_SCALE_12Gs
	};
	enum mag_resolution
	{
		M_LOW_RES = 0,
		M_HIGH_RES = 0b11
	};
	enum mag_filter_acceleration
	{
		M_FILTER_INTERNAL_BYPASSED = 0,
		M_FILTER_FROM_INTERNAL_TO_OUTPUT_AND_FIFO = 1
	};
	enum mag_sensor_mode
	{
		M_SENSOR_CONTINUOUS_CONVERSION = 0,
		M_SENSOR_SINGLE_CONVERSION = 1,
		M_SENSOR_POWER_DOWN = 2,
		M_SENSOR_POWER_DOWN_B = 3
	}; // Tab 91. diff between A&B unclear
	enum mag_power_mode
	{
		M_POWER_HIGH = 0,
		M_POWER_LOW = 1
	}; // Tab. 89

	enum gyr_scale
	{
		G_SCALE_250dps = 0,
		G_SCALE_500dps = 1,
		G_SCALE_1000dps = 2,
		G_SCALE_2000dps = 3
	};

	// See Section 8.2
	enum gyro_odr		       // 4 bit
	{			       // ODR (Hz) --- Cutoff
		G_ODR_95_BW_125 = 0x0, //   95         12.5
		G_ODR_95_BW_25 = 0x1,  //   95          25
				      // 0x2 and 0x3 define the same data rate and bandwidth as 0x1
		G_ODR_190_BW_125 = 0x4, //   190        12.5
		G_ODR_190_BW_25 = 0x5,  //   190         25
		G_ODR_190_BW_50 = 0x6,  //   190         50
		G_ODR_190_BW_70 = 0x7,  //   190         70
		G_ODR_380_BW_20 = 0x8,  //   380         20
		G_ODR_380_BW_25 = 0x9,  //   380         25
		G_ODR_380_BW_50 = 0xA,  //   380         50
		G_ODR_380_BW_100 = 0xB, //   380         100
		G_ODR_760_BW_30 = 0xC,  //   760         30
		G_ODR_760_BW_35 = 0xD,  //   760         35
		G_ODR_760_BW_50 = 0xE,  //   760         50
		G_ODR_760_BW_100 = 0xF, //   760         100
	};

	// Max 7%. or 2^(-k) thereof. see tab??
	enum gyr_high_pass
	{
		G_HIGH_MAX,
		G_HIGH_1_2,
		G_HIGH_1_4,
		G_HIGH_1_8,
		G_HIGH_1_16,
		G_HIGH_1_32,
		G_HIGH_1_64,
		G_HIGH_1_128,
		G_HIGH_1_256,
		G_HIGH_1_512,
		G_HIGH_1_1024
	};

	// bdu : Block data update for acceleration and magnetic data. Default value: 0
	//       (0: continuous update; 1: output registers not updated until MSB and LSB have been read)
	enum acc_bdu
	{
		A_CONTINUOUS_UPDATE = 0,
		A_UPDATE_AFTER_MSB_LSB_READ = 1
	};
	enum gyr_bdu
	{
		G_CONTINUOUS_UPDATE = 0,
		G_UPDATE_AFTER_MSB_LSB_READ = 1
	};

	enum gyr_power_mode
	{
		G_POWER_DOWN = 0,
		G_POWER_NORMAL = 1,
		G_POWER_SLEEP = 0b11
	}; // Sleep disables all axes

	enum pressure_oversampling
	{
		T_ULTRA_LOW_POWER = 0,
		T_STANDARD,
		T_HIGH_RESOLUTION,
		T_ULTRA_HIGH_RESOLUTION
	};

	// Should be redundant
	enum sensor_type
	{
		ACC,
		GYR,
		MAG,
		TP
	};

	enum rotation_units
	{
		RPM,		// Revolutions per minute
		RADPERSEC,	// Radians per second
		DEGPERSEC	// Degrees per second
	};

	enum acceleration_units
	{
		CMPSPS,		// centimeters per second per second
		MPSPS,		// meters per second per second
		FTSPS		// feet per second per second
	};


	// DONT UNDERSTAND YET:
	enum HighPassMode
	{
		NormalResetting = 0,
		ReferenceSignalForFiltering,
		Normal = 2,
		AutoResetOnInterrupt = 3
	};
	// ----------------------------------------------------

	// Types that define the state of each sensor
	struct AccState
	{ // Table 71 and 75 for registers CTRL_REG1_XM and CTRL_REG2_XM
		acc_scale scale = A_SCALE_2g;
		acc_odr odr = A_ODR_1600Hz;
		acc_aa_bandwidth aa_bw = A_BANDWIDTH_773Hz;
		acc_bdu bdu = A_CONTINUOUS_UPDATE;
		acc_selftest_mode selftest = A_TEST_OFF;
		bool spi_interface_mode = false;
		bool enableX = 1, enableY = 1, enableZ = 1;
	};

	struct MagState
	{
		mag_scale scale = M_SCALE_4Gs;
		mag_odr odr = M_ODR_100Hz;
		mag_resolution resolution = M_HIGH_RES;
		mag_power_mode power = M_POWER_HIGH;
		mag_sensor_mode sensormode = M_SENSOR_CONTINUOUS_CONVERSION;
		mag_filter_acceleration filter = M_FILTER_INTERNAL_BYPASSED;
		HighPassMode highpass = NormalResetting;
	};

	struct GyrState
	{
		gyr_scale scale = G_SCALE_2000dps;
		gyro_odr odr = G_ODR_760_BW_30;
		gyr_power_mode power = G_POWER_NORMAL;

		gyr_high_pass highpasscutoff = G_HIGH_MAX;
		HighPassMode highpassmode = NormalResetting;

		gyr_selftest_mode selftest = G_TEST_OFF;
		gyr_bdu bdu = G_CONTINUOUS_UPDATE;
		bool spi_interface_mode = 0;
		bool enableX = 1, enableY = 1, enableZ = 1;
	};

	struct configuration
	{
		bool bigendian = false;
		bool temperature_sensor_activated = true;
		pressure_oversampling oversampling = T_ULTRA_LOW_POWER; // notice that low power is fastest.
	};

	// return two bytes from data as a signed 16-bit value
//	int16_t get_short(uint8_t* data, int index) { return ((data[index] << 8) + data[index + 1]); }
//	uint16_t get_ushort(uint8_t* data, int index) { return ((data[index] << 8) + data[index + 1]); }

}

#endif
