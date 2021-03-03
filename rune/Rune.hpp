#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include "../misc/util.hpp"
#include "../misc/configurations.hpp"
#include "../misc/serial.hpp"
#include "runehiter/RuneHiter.hpp"

using namespace std;
using namespace cv;
using namespace runehiter;
// rune id为4
// RuneBase is no more than a socket(AKA, a modem). It successes universal API "Update()", in which rune-hitting is fulfilled.
class RuneBase : public ModuleBase{
public:
    RuneBase(SerialManager *serial) : ModuleBase(4), _serial(serial) {};
protected:
    SerialManager *_serial;
};

// On one hand, GeneralRuneHiter is designed to be a socket with API "Update()" for external calling so it belongs to RuneBase and successes RuneBase. 
// On the other hand, it is a special socket with rune-hitting abilities so it belong to RuneHiter class and success RuneHiter.
class GeneralRuneHiter : public RuneBase, public RuneHiter
{
public:
    GeneralRuneHiter(SerialManager * serial) : RuneBase(serial), RuneHiter(BLUE)
    {
        if (2 - ElectronicControlParams::teamInfo == ARMOR_BLUE)
            this->setColor(BLUE);
        else if (2 - ElectronicControlParams::teamInfo == ARMOR_RED)
            this->setColor(RED);
        else 
            printf("the color paras of \'RuneHiter()\' is wrong\n");
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

class InfancyRuneHiter : public GeneralRuneHiter
{
public:
    InfancyRuneHiter(SerialManager *serial) : GeneralRuneHiter(serial) {}
};