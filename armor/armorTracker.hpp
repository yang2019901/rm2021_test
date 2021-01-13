/*
 * 这是 TJU Robomasters 上位机源码，未经管理层允许严禁传播给其他人（包括队内以及队外）
 *
 *   This file defines base class of armor tracker, and its implements. Tracker is 
 * designed to control ptz to follow the aiming target.
 *    2018.6.12 Architect the armortrackerBase
 */

#pragma once
#include <opencv2/opencv.hpp>
#include "util.hpp"
#include "configurations.hpp"
#include "camview.hpp"
#include "predictor.hpp"
#include "armorDetector.hpp"
#include "dnnManager.hpp"
#include "trackerMedianFlow.hpp"

using namespace cv;
using namespace std;

//装甲追踪基类
class ArmorTrackerBase
{
public:
    // Configuration variables
    int InfancyMaxTrackingFrame = 60;
    int HeroMaxTrackingFrame = 30;
    int EngineerMaxTrackingFrame = 10;
    double ArmorEvaluationDistanceWeight = 0.1;
    double DetectionROIScale = 2.5;
    double TrackerBoundRectScale = 1.5;
    int TrackerResetTime = 10;
    double TrackerPositionChangeThresh = 0.14;
    double TrackerSizeChangeThresh = 0.5;

    // 云台到最佳射击角度的偏转角度，默认这个角度为到装甲的角度加上重力补偿角度的值
    Point2f shootOffAngle;
    // 云台到装甲的偏转角度，转过该角度云台正好对准装甲
    Point2f ptzOffAngle;
    // 追踪得到的装甲距离云台的距离
    float lastArmorDistance ;
    // 装甲的世界坐标
    Point3f targetWorldPosition;
    // 上次检测得到的装甲结果
    ArmorResult lastArmor;
    // track state: 0 lost target; 1 last detected ; 2+ last tracked
    int trackState = 0;
    // 跟踪的目标兵种
    ArmorDetailType trackTargetType;

    ArmorTrackerBase(CameraView *_camview,ArmorBaseDetector *_detector,DnnManager *_dnn) //构造函数
    {
        camview = _camview;
        detector = _detector;
        dnnManager = _dnn;

        SET_CONFIG_INT_VARIABLE(InfancyMaxTrackingFrame,60)
        SET_CONFIG_INT_VARIABLE(HeroMaxTrackingFrame,30)
        SET_CONFIG_INT_VARIABLE(EngineerMaxTrackingFrame,10)
        SET_CONFIG_DOUBLE_VARIABLE(ArmorEvaluationDistanceWeight,0.1)
        SET_CONFIG_DOUBLE_VARIABLE(DetectionROIScale,2.5)
        SET_CONFIG_DOUBLE_VARIABLE(TrackerBoundRectScale,1.5)
        SET_CONFIG_INT_VARIABLE(TrackerResetTime,10)
        SET_CONFIG_DOUBLE_VARIABLE(TrackerPositionChangeThresh,0.14)
        SET_CONFIG_DOUBLE_VARIABLE(TrackerSizeChangeThresh,0.5)
    }

    // called every frame, and returns the deltaAngle for ptz control
    // the base class only consider shootOffAngle as the ptz control deltaAngle
    virtual Point2f UpdateFrame(ImageData frame,float deltaTime)
    {
        // disable detectors' debug image
        // detector->DisallowDebug();
//LOG(A)
        // track too much or lost target, detect the whole frame and rechoose a target
        //重新开始检测全图
        if (!trackState || (trackState > InfancyMaxTrackingFrame && trackTargetType == ARMOR_INFAN) ||
                (trackState > HeroMaxTrackingFrame && trackTargetType == ARMOR_HERO) ||
                (trackState > EngineerMaxTrackingFrame && trackTargetType == ARMOR_ENGIN) ||
                (trackState > InfancyMaxTrackingFrame && trackTargetType == ARMOR_TYPE_UNKNOWN))
        {
            detector->DetectArmors(frame.image);
//LOG(B)
            if (detector->result.size() > 0)
            {
                ArmorDetailType armRes[20]; 
                float confidence[20];       
                this->GetArmorTypes(armRes,confidence);
                ChooseTarget(frame,armRes,confidence);
                this->InitTracker(frame);
                trackState = 1;
            }
            else if (trackerAlgorithmState) //即使没有检测到目标，仍然尝试追踪
            {
                // still try tracking
                if (UpdateTracker(frame))
                    ApplyTracker(frame);
                else trackState = 0;
            }
            else
                // not found any possible target
                trackState = 0;
        }
        else // small-range detect or just track
        {
            // 根据上一次计算出的世界坐标 给出在屏幕上的投影位置
            float dis;
            Point2f lptzAngle = ProjectWorldPositionToPTZAngle(frame,targetWorldPosition,dis);  
            Point2d scrPt = camview->PTZAngleToScreenPoint(lptzAngle * Deg2Rad,dis);    //相机坐标系到图像坐标系
//LOG(C)
            // first try detection长度
            Rect r1 = lastArmor.leftLight.rr.boundingRect(), r2 = lastArmor.rightLight.rr.boundingRect();  
//LOG(1)
            //int left = min(r1.x, r2.x), top = min(r1.y, r2.y), right = max(r1.x + r1.width, r2.x + r2.width), bottom = max(r1.y + r1.height, r2.y + r2.height);
            //Rect roiRect(left,top,right-left,bottom-top);
            int width = max(r1.x + r1.width, r2.x + r2.width) - min(r1.x, r2.x), height = max(r1.y + r1.height, r2.y + r2.height) - min(r1.y, r2.y);
            width *= DetectionROIScale ; height *= DetectionROIScale;
            Rect roiRect(scrPt.x - width * 0.5f,scrPt.y - height * 0.5f,width,height);  
//LOG(2)
            //roiRect.x -= roiRect.width * (DetectionROIScale - 1) * 0.5f;
            //roiRect.y -= roiRect.height * (DetectionROIScale - 1) * 0.5f;
            //roiRect.width *= DetectionROIScale; roiRect.height *= DetectionROIScale;
            MakeRectSafe(frame.image,roiRect);  
            Point2f startPoint(roiRect.x,roiRect.y);
            Mat roiRegion = frame.image(roiRect).clone();
//LOG(4)
            detector->DetectArmors(roiRegion);
//LOG(D)
            // check detection result
            if (detector->result.size() > 0)
            {
                trackState++;
                // transform the result in roi to frame image
                FOREACH(i,detector->result.size())
                {
                    detector->result[i].center += startPoint;
                    detector->result[i].leftLight.center += startPoint;
                    detector->result[i].leftLight.rr.center += startPoint;
                    detector->result[i].rightLight.rr.center += startPoint;
                    detector->result[i].rightLight.center += startPoint;
                }
                ChooseTarget(frame);
                if (trackerAlgorithmState > TrackerResetTime || !UpdateTracker(frame))
                    InitTracker(frame);
            }//else trackState = 0;
            else
            {
                if (UpdateTracker(frame))
                {
                    // successfully tracked the armor
                    trackState += 2; // 追踪的结果往往不可靠
                    ApplyTracker(frame);
                }
                else // tracking failed, lost target
                    trackState = 0;
            }
            if (DEBUG_MODE && trackState)
            {
                // circle(frame.image,scrPt,3,Scalar(0,0,255),2);
                rectangle(frame.image,roiRect,Scalar(0,0,255),2);
                //cout << "world position calculated PTZ off angle " << (lptzAngle * Rad2Deg) << endl;
            }
        }
//LOG(E)
        // 如果检测成功，计算装甲的世界坐标
        if (trackState)
        {
            Point2f worldAngle = (ptzOffAngle + frame.ptzAngle) * Deg2Rad;
            targetWorldPosition.y = sin(worldAngle.y) * lastArmorDistance ;
            float d = cos(worldAngle.y) * lastArmorDistance;
            targetWorldPosition.x = d * cos(worldAngle.x);
            targetWorldPosition.z = d * sin(worldAngle.x);
            targetWorldPosition.x += frame.worldPosition.x;
            targetWorldPosition.z += frame.worldPosition.y;
        }

//LOG(F)
        if (DEBUG_MODE)
        {
            if (trackState)
                circle(frame.image,lastArmor.center, 6 ,Scalar(255,0,255),2);
            if (trackerAlgorithmState)
                rectangle(frame.image,trackerBound,Scalar(0,255,0),1);
            //DEBUG_DISPLAY(frame.image)
        }
//LOG(G)
        return shootOffAngle;
    }

    // choose an target from detector result 
    // default stratage is to find the nearest armor to hit
    virtual void ChooseTarget(ImageData &frame_data,ArmorDetailType types[],float confidence[])
    {
        Point2f bestPTZ,curPTZ;
        float bestDis,curDis,bestScore,curScore;
        int bestIndex = -1;
        FOREACH(i,detector->result.size())
        {
            curPTZ = CalcArmorPTZAngle(detector->result[i],curDis);
            curScore = EvaluateArmorPosition(curPTZ,curDis,types[i],confidence[i]);
            if (bestIndex == -1 || curScore > bestScore)
            {
                bestIndex = i;
                bestDis = curDis;
                bestScore = curScore;
                bestPTZ = curPTZ;
            }
        }
        // choose target 'bestIndex'
        lastArmor = detector->result[bestIndex];
        ptzOffAngle = bestPTZ;
        shootOffAngle = ptzOffAngle + CalculateGravityAngle(frame_data.ptzAngle.y + ptzOffAngle.y,frame_data.shootSpeed,bestDis);
        lastArmorDistance = bestDis;
        this->trackTargetType = types[bestIndex];
    }

    void ChooseTarget(ImageData &frame_data)
    {
        ArmorDetailType t_types[20];
        float t_conf[20];
        FOREACH(i,detector->result.size()){
            t_types[i] = trackTargetType;
            t_conf[i] = 0.5f;
        }
        ChooseTarget(frame_data,t_types,t_conf);
    }

protected:
    CameraView *camview;
    ArmorBaseDetector *detector;
    DnnManager *dnnManager;
    Ptr<TrackerMedianFlow> tracker;
    int trackerAlgorithmState = 0; // 0 means uninitialized  ; otherwise means initialized and the time tracked
    Rect2d trackerBound;

    void InitTracker(ImageData &frame)
    {
        tracker = TrackerMedianFlow::create();
        trackerBound.width = abs(lastArmor.leftLight.center.x - lastArmor.rightLight.center.x) * TrackerBoundRectScale;
        trackerBound.height = (lastArmor.leftLight.rr.size.height + lastArmor.rightLight.rr.size.height) * TrackerBoundRectScale ;
        trackerBound.x = lastArmor.center.x - trackerBound.width / 2;
        trackerBound.y = lastArmor.center.y - trackerBound.height / 2;
        tracker->init(frame.image,trackerBound);
        trackerAlgorithmState = 1;
    }

    bool UpdateTracker(ImageData &frame)
    {
        Rect2d org = trackerBound;
        // check position sudden change or size sudden change
        if (
                (!tracker->update(frame.image,trackerBound)) || // algorithm tracking failed
                (Length(Point2d(org.x + org.width / 2 - trackerBound.x - trackerBound.width/2, // position suddenly change
                           org.y + org.height / 2 - trackerBound.y - trackerBound.height/2)) / (org.width + org.height) > TrackerPositionChangeThresh) ||
                (trackerBound.width * trackerBound.height / org.width / org.height < TrackerSizeChangeThresh)  // size suddenly change
           )
        {
            trackerAlgorithmState = 0;
            return false;
        }
        trackerAlgorithmState ++;
        return true;
    }

    void ApplyTracker(ImageData &frame)
    {
        Point2f delta (trackerBound.x +trackerBound.width /2 - lastArmor.center.x,
                       trackerBound.y + trackerBound.height/2 - lastArmor.center.y );
        lastArmor.center += delta;
        lastArmor.leftLight.center += delta;
        lastArmor.rightLight.center += delta;
        lastArmor.leftLight.rr.center += delta;
        lastArmor.rightLight.rr.center += delta;

        ptzOffAngle = CalcArmorPTZAngle(lastArmor,lastArmorDistance);
        shootOffAngle = ptzOffAngle + CalculateGravityAngle(
            frame.ptzAngle.y+ptzOffAngle.y,frame.shootSpeed,lastArmorDistance);
    }

    Point2f ProjectWorldPositionToPTZAngle(ImageData &frameData,Point3f worldPosition,float &distance)
    {
        worldPosition.x -= frameData.worldPosition.x;
        worldPosition.z -= frameData.worldPosition.y;
        distance = Length(worldPosition);
        Point2f absoluteAngle;
        absoluteAngle.y = atan(worldPosition.y / sqrt(worldPosition.x * worldPosition.x + worldPosition.z * worldPosition.z));
        absoluteAngle.x = atan2(worldPosition.z ,worldPosition.x);
        return absoluteAngle * Rad2Deg - frameData.ptzAngle;
    }

    void GetArmorTypes(ArmorDetailType res[],float confidence[])
    {
        vector<Mat> imgs;
        FOREACH(i,detector->result.size())
        {
            // initialize res and confidence
            res[i] = ARMOR_TYPE_UNKNOWN; //armor_type 
            confidence[i] = 0;           //confidence
            Mat resized;                 //resize img to 28*28
            resize(detector->ArmorNumberAreaGray(detector->result[i]),resized,Size(28,28));
            imgs.push_back(resized);     //push to vector
        }
        dnnManager->ClassifyArmors(imgs,res,confidence);
        // if(DEBUG_MODE)
        // {
        //     FOREACH(i,imgs.size())
        //     cout<<"armor_id:"<<res[i]<<"    armor_confidence:"<<confidence[i]<<endl;
        // }
        FOREACH(i,imgs.size())
        {
            switch(res[i])
            {
            case 3:case 4:case 5: res[i] = ARMOR_INFAN; break;
            case 1:res[i] = ARMOR_HERO; break;
            case 2:res[i] = ARMOR_ENGIN; break;
            default: res[i] = ARMOR_TYPE_UNKNOWN; break;
            }
        }
    }

    // returns ptz off angle of an armor and calculates the probdis by the way;
    virtual Point2f CalcArmorPTZAngle(ArmorResult armor,float &probDis)
    {
        // estimate the distance of the armor        
        const float height_weight = 20 , width_weight = 2, whole_scale = 100;
        probDis = (armor.leftLight.rr.size.height + armor.rightLight.rr.size.height) / height_weight +
            (armor.leftLight.rr.size.width + armor.rightLight.rr.size.width) / width_weight;    //装甲板距离的一个量化指标
        // cout<<armor.leftLight.rr.size.height<<" "<<armor.rightLight.rr.size.height<<endl;
        cout<<armor.leftLight.rr.size.width<<" "<<armor.rightLight.rr.size.width<<endl;
        probDis *= whole_scale;
        return camview->ScreenPointToPTZAngle(armor.center,probDis,1);
    }
    // evaluate armor worth shooting by considering ptz offset angle and estimated distance
    virtual float EvaluateArmorPosition(Point2f ptzAngle,float probDis,ArmorDetailType type,float confidence)
    {
        float typeScore = 0;
        switch(type)
        {
        case ARMOR_INFAN: typeScore = 250; break;
        case ARMOR_HERO: typeScore = 200; break;
        case ARMOR_ENGIN: typeScore = 50;break;
        }
        typeScore *= confidence;
        return typeScore -(probDis * ArmorEvaluationDistanceWeight + Length(ptzAngle));
    }
    // virtual calculateDistance()
    // {
    //     vector<Point3f> obj=vector<Point3f>{
    // cv::Point3f(-HALF_LENGTH, -HALF_LENGTH, 0),	//tl
    // cv::Point3f(HALF_LENGTH, -HALF_LENGTH, 0),	//tr
    // cv::Point3f(HALF_LENGTH, HALF_LENGTH, 0),	//br
    // cv::Point3f(-HALF_LENGTH, HALF_LENGTH, 0)	//bl
    // };
    // }
};




/*
 * 该Tracker在基类的基础上，通过预测算法预测目标移动轨迹，根据子弹飞行时间给出一定的提前量
 * 通过PID 控制算法，提高云台对目标角度的响应速度，并通过积分项消除运动时产生的静差
 */
class PredictPIDArmorTracker : public ArmorTrackerBase
{
public:
    double PredictionTimeScale = 1;
    double PredictionTimeBias = 0;

    PredictPIDArmorTracker(CameraView *_camview,ArmorBaseDetector *_detector,DnnManager *_dnn,Predictor *_prd)
        : ArmorTrackerBase(_camview,_detector,_dnn)
    {
        predictor = _prd;
        yawPID.SetLimitParams(60,10);
        pitchPID.SetLimitParams(30,5);
        SET_CONFIG_DOUBLE_VARIABLE(PredictionTimeScale,1);
        SET_CONFIG_DOUBLE_VARIABLE(PredictionTimeBias,0);

        SET_CONFIG_DOUBLE_VARIABLE(yawPID.kp,1);
        SET_CONFIG_DOUBLE_VARIABLE(yawPID.ki,0);
        SET_CONFIG_DOUBLE_VARIABLE(yawPID.kd,0);
        SET_CONFIG_DOUBLE_VARIABLE(pitchPID.kp,1);
        SET_CONFIG_DOUBLE_VARIABLE(pitchPID.ki,0);
        SET_CONFIG_DOUBLE_VARIABLE(pitchPID.kd,0);
    }
    // 重写UpdateFrame函数，增加预测器和PID控制
    Point2f UpdateFrame(ImageData frame,float deltaTime)
    {
        // 调用基类的函数更新
        ArmorTrackerBase::UpdateFrame(frame,deltaTime);
        if (trackState)
        {
            float dis ;
            predictor->AddPredictPoint(PredictionPathPoint(targetWorldPosition,ptzOffAngle,frame.worldPosition,deltaTime));
            Point3f predicted = predictor->Predict(lastArmorDistance * PredictionTimeScale / frame.shootSpeed * 0.001f + PredictionTimeBias);
            //Point3f predicted = targetWorldPosition;
            Point2f ptz = ProjectWorldPositionToPTZAngle(frame,predicted,dis);
            shootOffAngle += ptz - ptzOffAngle;
            ptzOffAngle = ptz;

            if (DEBUG_MODE)
            {
                Point2d scrPt = camview->PTZAngleToScreenPoint(ptz * Deg2Rad,dis);
                circle(frame.image,scrPt, 3 ,Scalar(255,255,255),1);
            }
            // the target angle is shootoffangle
            return Point2f(
                yawPID.calc(shootOffAngle.x),
                pitchPID.calc(shootOffAngle.y)
            );
        }
        else
        {
            predictor->ClearHistory();
            return Point2f();
        }
    }
protected:
    Predictor *predictor;
    PID yawPID,pitchPID;
};
