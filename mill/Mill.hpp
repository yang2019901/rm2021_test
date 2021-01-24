#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include "util.hpp"
#include "configurations.hpp"
#include "serial.hpp"
#include "mill-test/MillHiter.hpp"

using namespace std;
using namespace cv;
using namespace millhiter;
// mill id为4
// MillBase is no more than a socket(AKA, a modem). It successes universal API "Update()", in which mill-hitting is fulfilled.
class MillBase : public ModuleBase{
public:
    MillBase(SerialManager *serial) : ModuleBase(4), _serial(serial) {};
protected:
    SerialManager *_serial;
};

// On one hand, GeneralMillHiter is designed to be a socket with API "Update()" for external calling so it belongs to MillBase and successes MillBase. 
// On the other hand, it is a special socket with mill-hitting abilities so it belong to MillHiter class and success MillHiter.
class GeneralMillHiter : public MillBase, public MillHiter
{
public:
    GeneralMillHiter(SerialManager * serial) : MillBase(serial), MillHiter(BLUE)
    {
        if (2 - ElectronicControlParams::teamInfo == ARMOR_BLUE)
            this->setColor(BLUE);
        else if (2 - ElectronicControlParams::teamInfo == ARMOR_RED)
            this->setColor(RED);
        else 
            printf("the color paras of \'MillHiter()\' is wrong\n");
        return ;
    };
    void Update(ImageData &frame, float dtTime)
    {
        // 2 - ElectronicControlParams::teamInfo is the color of the enemy
        Mat img = frame.image;
        if (!this->init(img))
            return;
        Point2f prePos, postPos;
        if (!this->targetLock(img, prePos))
            return;
        circle(img, prePos, 3, Scalar(0, 255, 255), -1);
        if (this->predictConstSpeed(prePos, postPos, dtTime))
            circle(img, postPos, 3, Scalar(0, 255, 255), -1);
        DEBUG_DISPLAY(img);
    }
};

class InfancyMillHiter : public GeneralMillHiter
{
public:
    InfancyMillHiter(SerialManager *serial) : GeneralMillHiter(serial) {}
};