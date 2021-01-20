#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include "util.hpp"
#include "configurations.hpp"
#include "serial.hpp"
#include "DfcDetector.hpp"
#include "MillHiter.hpp"

using namespace std;
using namespace cv;
using namespace millhiter;
//Dfc id为4
class DfcHiter : public ModuleBase{
public:
    DfcHiter(SerialManager *_serial,DfcBaseDetetor* _dfcBaseDetetor):ModuleBase(4)
    {
        serial = _serial;
        dfcBaseDetector = _dfcBaseDetetor;
    }

protected:
    SerialManager *serial;
    DfcBaseDetetor *dfcBaseDetector;    
};

class InfancyDfcHiter : public DfcHiter
{
public:

    
    InfancyDfcHiter(SerialManager *_serial,DfcBaseDetetor *_dfcBaseDetetor) : DfcHiter(_serial,_dfcBaseDetetor), _hiter(BLUE)
    {
        // 2 - ElectronicControlParams::teamInfo is the color of the enemy
        if (2 - ElectronicControlParams::teamInfo == ARMOR_BLUE)
            this->_hiter.setColor(BLUE);
        else if (2 - ElectronicControlParams::teamInfo == ARMOR_RED)
            this->_hiter.setColor(RED);
        else 
            printf("the color paras of InfancyDfcHiter is wrong\n");
        return ;
    }

    void Update(ImageData &frame,float dtTime)
    {   
        // dfcBaseDetector->DetectDfcArmors(frame.image);
        // 功能类 MillHiter 的接口
        Mat img = frame.image;
        if (!this->_hiter.init(img))
            return ;
        Point2f prePos, postPos;
        if (!this->_hiter.targetLock(img,prePos))
            return;
        circle(img, prePos, 3, Scalar(0,255,255), -1);
        if (_hiter.predictIn(prePos, postPos, dtTime, CONSTMODE))
            circle(img, postPos, 3, Scalar(0, 255, 255), -1);

        imshow("marked frame", img);
        if (waitKey(0) == 'q')
            return;
    }

protected:
    MillHiter _hiter;
};