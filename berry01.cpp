#include <iostream>
#include "berryimu.h"

using namespace BerryIMU;
using namespace std;

int main(int argc, char *argv[])
{

	Acc * acc;
	Gyr * gyr;
	Mag * mag;

	try
	{
		acc = new Acc();
		gyr = new Gyr();
		mag = new Mag();

		bool status = acc->enable();
		if (!status)
		{
			cout << "Berry Acc failed to enable" << endl;
			exit(0);
		}

		status = gyr->enable();
		if (!status)
		{
			cout << "Berry Gyr failed to enable" << endl;
			exit(0);
		}

		status = mag->enable();
		if (!status)
		{
			cout << "Berry Mag failed to enable" << endl;
			exit(0);
		}

//		double accData[3];
//		double gyrData[3];
//		double magData[3];
//		long int seconds;
		float xRot, yRot, zRot;
//		float xAcc, yAcc, zAcc;
//		cout << endl << "xDeg" << "\t" << "yDeg" << "\t" << "zDeg" << "\t" << "xRPM" << "\t" << "yRPM" << "\t" << "zRPM" << "\t" << "xRad" << "\t" << "yRad" << "\t" << "zRad" << endl;
		cout << endl << "xRaw" << "\t" << "yRaw" << "\t" << "zRaw" << endl;// << "\t" << "xRPM" << "\t" << "yRPM" << "\t" << "zRPM" << "\t" << "xRad" << "\t" << "yRad" << "\t" << "zRad" << endl;
		int iterations = 300;
		while (iterations--)
		{
//			acc->read(accData);
//			gyr->read(gyrData);
//			mag->read(magData);
//			cout << "Acc:" << accData[0] << ":" << accData[1] << ":" << accData[2] << "\tMag:" <<  magData[0] << "\tGyr:" << gyrData[0]  << endl;
//			gyrData[0] = gyrData[1] = gyrData[2] = 0.0;
			xRot = yRot = zRot = 0;
			gyr->rawRotation(xRot, yRot, zRot);
			cout << to_string(xRot) << "\t" << to_string(yRot) << "\t" << to_string(zRot) << endl;
//			gyr->rotation(xRot, yRot, zRot, DEGPERSEC);
//			cout << to_string(xRot) << "\t" << to_string(yRot) << "\t" << to_string(zRot) << "\t";
//			gyr->rotation(xRot, yRot, zRot, RPM);
//			cout << to_string(xRot) << "\t" << to_string(yRot) << "\t" << to_string(zRot) << "\t";
//			gyr->rotation(xRot, yRot, zRot, RADPERSEC);
//			cout << to_string(xRot) << "\t" << to_string(yRot) << "\t" << to_string(zRot) << endl;

//			acc->acceleration(xAcc, yAcc, zAcc);
//			cout << "Gyr:" << to_string(xRot) << "\t" <<  to_string(yRot) << "\t" <<  to_string(zRot) << "\tRAD/sec\tAcc:" <<  to_string(xAcc) << "\t" <<  to_string(yAcc) << "\t" <<  to_string(zAcc) << endl;
		}

		delete(acc);
		delete(gyr);
		delete(mag);
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
