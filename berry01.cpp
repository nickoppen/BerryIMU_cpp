#include <iostream>
#include "berryimu.h"

using namespace BerryIMU;
using namespace std;

bool continuePolling;	// signal the call back to request that the polling loop close
bool retrieveAccRaw(int16_t* rawData, int xyzValues)
{
	uint8_t i, j;
	uint8_t data1, data2, data3;

	cout << endl << "Got data:" << endl;
	for (i = 0; i < xyzValues; i++)
	{
		j = i * 3;
		data1 = rawData[j];
		data2 = rawData[j+1];
		data3 = rawData[j + 2];
		cout << (unsigned int)i << ":" << (unsigned int)data1 << " " << (unsigned int)data2 << " " << (unsigned int)data3 << endl;
	}

	return continuePolling;
}

int main(int argc, char *argv[])
{

	Acc & acc = Acc::Get_Instance();
	Gyr & gyr = Gyr::Get_Instance();
//	Mag * mag;

	try
	{
//		mag = new Mag();

		bool status = acc.enable();
		if (!status)
		{
			cout << "Berry Acc failed to enable" << endl;
			exit(0);
		}
		continuePolling = true;
		acc.enableFIFO();
		acc.setDatarate(A_ODR_3p125Hz);
		acc.initiateDataReadyWithCallback(retrieveAccRaw);
		sleep(200);
		continuePolling = false;
		acc.disableFIFO();

		//status = gyr.enable();
		//if (!status)
		//{
		//	cout << "Berry Gyr failed to enable" << endl;
		//	exit(0);
		//}

		//status = mag->enable();
		//if (!status)
		//{
		//	cout << "Berry Mag failed to enable" << endl;
		//	exit(0);
		//}

//		double accData[3];
//		double gyrData[3];
//		double magData[3];
//		long int seconds;
//		float xRot, yRot, zRot, sumXRot, sumYRot, sumZRot;
//		float xAcc, yAcc, zAcc, sumXAcc, sumYAcc, sumZAcc;
//		cout << endl << "xDeg" << "\t" << "yDeg" << "\t" << "zDeg" << "\t" << "xRPM" << "\t" << "yRPM" << "\t" << "zRPM" << "\t" << "xRad" << "\t" << "yRad" << "\t" << "zRad" << endl;
		//cout << endl << "xAccRaw" << "\t" << "yAccRaw" << "\t" << "zAccRaw" << "\t" << "xDeg" << "\t" << "yDeg" << "\t" << "zDeg" << endl;// "\t" << "xRad" << "\t" << "yRad" << "\t" << "zRad" << endl;
		//int iterations = 250;
		//int readingsPerAverage;
		//while (iterations--)
		//{
		//	readingsPerAverage = 100;
		//	sumXAcc = sumYAcc = sumZAcc = 0;
		//	sumXRot = sumYRot = sumZRot = 0;
		//	while (readingsPerAverage--)
		//	{
		//		xAcc = yAcc = zAcc = 0;
		//		acc->rawAcceleration(xAcc, yAcc, zAcc);
		//		sumXAcc += xAcc;
		//		sumYAcc += yAcc;
		//		sumZAcc += zAcc;

		//		xRot = yRot = zRot = 0;
		//		gyr->rotation(xRot, yRot, zRot, DEGPERSEC);
		//		sumXRot += xRot;
		//		sumYRot += yRot;
		//		sumZRot += zRot;
		//	}
		//	cout << to_string(xAcc) << "\t" << to_string(yAcc) << "\t" << to_string(zAcc) << "\t";
		//	cout << to_string(xRot) << "\t" << to_string(yRot) << "\t" << to_string(zRot) << endl;
		//	sleep(1);

		//}

//		delete(mag);
	}
	catch (int e)
	{
		switch (e)
		{
			case 0:
				cout << "Not Implemeted Yet: " << e << endl;
				break;
			case 1:
				cout << "Unknown device type: " << e << endl;
				break;
			case 2:
				cout << "Scale not defined: " << e << endl;
				break;
			default:
				cout << "Look for an unhandled throw!" << endl;
		}
	}

	return 0;
}

