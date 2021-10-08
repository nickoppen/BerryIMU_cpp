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
		if (status)
		{
			cout << "Berry Acc is enabled" << endl;
		}
		else
		{
			cout << "Berry Acc failed to enable" << endl;
		}

		status = gyr->enable();
		if (status)
		{
			cout << "Berry Gyr is enabled" << endl;
		}
		else
		{
			cout << "Berry Gyr failed to enable" << endl;
		}

		status = mag->enable();
		if (status)
		{
			cout << "Berry Mag is enabled" << endl;
		}
		else
		{
			cout << "Berry Mag failed to enable" << endl;
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
