#include<iostream>
#include"iarmInterface.h"



int main()
{
	float x, y, z, roll, pitch, yaw;
	IARMInterface iarm;
	iarm.init();
	iarm.stockUnfold();
	iarm.stockLiftDown();
	iarm.stockMoveHomePosition();

	while (true) {
		if (iarm.check() == IARMInterface::CHECK_RESULT_FINISH) {
			printf("x y z\n");
			scanf("%f %f %f", &x, &y, &z);
			if (x == 1000 || y == 1000 || z == 1000) break;
			iarm.moveTo(x, y, z);
		}
	}
	return 0;
}