#include "gimbal_control.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace roborts_detection;
using namespace std;
using namespace cv;

int main()
{
    fstream fin("/home/yangming/Coding/rm/rm2021_adv/mill/config.txt");

    GimbalContrl infancy;
    infancy.Init(0,0,0,0,0,15,0.1);
    
    float pitch,yaw;
    float x,y,z;
    
    fin >> x >> y >> z;

    Point3f p(x, y, z);
    infancy.Transform(p,yaw,pitch);

    cout << p << endl;
    printf("yaw: %f, pitch: %f\n", yaw, pitch);
    return 0;
}