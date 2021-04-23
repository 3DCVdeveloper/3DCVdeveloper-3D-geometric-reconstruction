#include <iostream>
#include "orbbec.h"
#include "timestamp.h"

using namespace std;
using cv::imshow;
using cv::Mat;
using cv::waitKey;

int main(int argc, char **argv)
{
	Orbbec CamStream;
	timeStamp Stamper;
	
	while (1)
	{
		Mat color, depth;
		CamStream.getFrames(color, depth);
		// Stamper.run(color, depth); // 将数据存为TUM-RGBD格式

		imshow("Color Image", color);
		imshow("Depth Image", depth * 10);

		if (waitKey(1) == 'q')
			break;
	}
}
