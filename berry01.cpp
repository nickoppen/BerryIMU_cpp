#include <iostream>
#include "berryimu.h"

using namespace BerryIMU;
using namespace std;

int main(int argc, char *argv[])
{

	IMU * b0;

	try
	{
		b0 = new IMU();

		bool b0Status = b0->enableIMU();
		if (b0Status)
		{
			cout << "Berry IMU is enabled" << endl;
		}
		else
		{
			cout << "Berry IMU failed to enable" << endl;
		}

		delete(b0);
	}
	catch (int e)
	{
		std::cout << "Failed to create instance of BerryIMU object: " << e << "\n";
	}

	return 0;
}
