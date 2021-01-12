/***
 * When I wrote this, only God and I understood what I was doing
 * Now, God only knows
 *                    _ooOoo_
 *                   o8888888o
 *                   88" . "88
 *                   (| -_- |)
 *                    O\ = /O
 *                ____/`---'\____
 *              .   ' \\| |// `.
 *               / \\||| : |||// \
 *             / _||||| -:- |||||- \
 *               | | \\\ - /// | |
 *             | \_| ''\---/'' | |
 *              \ .-\__ `-` ___/-. /
 *           ___`. .' /--.--\ `. . __
 *        ."" '< `.___\_<|>_/___.' >'"".
 *       | | : `- \`.;`\ _ /`;.`/ - ` : | |
 *         \ \ `-. \_ __\ /__ _/ .-` / /
 * ======`-.____`-.___\_____/___.-`____.-'======
 *                    `=---='
 *.............................................
 *          佛祖保佑             永无BUG
 *
 ***/


#include "custom_definations.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "armorDetector.hpp"
#include "serial.hpp"
#include <thread>
#include "camview.hpp"         
#include <unistd.h>
#include "armorTracker.hpp"
#include "armorHiter.hpp"
#include "dnnManager.hpp"
#include "dfcDetector.hpp"
using namespace cv;
using namespace std;


void ProcessFullFunction(ImageData &frame);
void ProcessAlgorithmFunction(ImageData &frame);
void ArmorDetectDebug(ImageData &frame);
void DfcDetectDebug(ImageData &frame);
double lastt = getTickCount(), curt = lastt,checkpoint_time = lastt,dtTime = 0;
int fps  = 0,frames = 0;

void capture_init(VideoCapture &capture)
{
    //设置帧缓存,capture.set返回1则设置成功，返回0则设置失败
    cout<<"帧缓存设置："<<capture.set(CV_CAP_PROP_BUFFERSIZE,3)<<endl; 
    //设置分辨率
    switch (ConfigurationVariables::GetInt("resolutionType",0))
    {
    case 0:
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
        capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
        break;
    case 1 :
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
        capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    case 2:
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
        capture.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    default:
        break;
    }  
    cout<<"图片压缩格式设置："<<capture.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'))<<endl; 
    cout<<"手动曝光设置:"<<capture.set(CV_CAP_PROP_AUTO_EXPOSURE,0.25)<<endl;
    cout<<"曝光度设置："<<capture.set(CV_CAP_PROP_EXPOSURE, ConfigurationVariables::GetInt("exposure",30)*0.0002)<<endl;
    cout<<"曝光度："<<capture.get(CV_CAP_PROP_EXPOSURE)<<endl;
    //0-1 映射到qv4l2的1-5000
}

// returns delta time
void UpdateFPS()
{
    frames++;
    curt = (double)getTickCount();
    //getTickFrequency 返回时钟频率
    //如果当前时钟减去上次输出时钟计数器为运行频率，则统计当前frames
    if (curt - checkpoint_time >= getTickFrequency()) 
    {
        checkpoint_time = curt;
        fps = frames;
        frames = 0;
        cout <<"fps:"<< fps << endl;
    }
    dtTime = (curt - lastt) / getTickFrequency();
    lastt = curt;  
}

void DisplayFPS(Mat &m)
{
    char fpsstr[30];
    sprintf(fpsstr,"fps:%d",fps);
    putText(m,fpsstr,Point(20,20),FONT_HERSHEY_SIMPLEX,1.0,Scalar(0,0,0),2);

}

#if defined DEVICE_TX
    VideoCapture rmcap("/dev/video0");
#elif defined DEVICE_MANIFOLD
    VideoCapture rmcap("/dev/video0");
#elif defined DEVICE_PC
    VideoCapture rmcap("/dev/video0");
    // VideoCapture rmcap("../doc/videos/blue.avi");
#endif

bool threadContinueFlag = true;
char waitedKey = 0;
bool onpause = false;
Lock<ImageData> frameData;//the framedata includes the data of every image
// modules definations
SerialManager *serial_ptr = NULL;
CameraView *camview_ptr = NULL;
ArmorBaseDetector *armor_detector_ptr = NULL;
ArmorTrackerBase *armor_tracker_ptr = NULL;
ArmorHiter *armor_hiter_ptr = NULL;
DnnManager *dnn_manager_ptr = NULL;

EllipseDfcDetector *dfc_detector_ptr = NULL;

Point2f res;

// 图像收集线程
void ImageCollectThread()
{
    SerialPort serialPort(ConfigurationVariables::GetInt("Port",1));
    SerialManager serialManager(&serialPort);
    serial_ptr = &serialManager;

    int frameIndex = 0;
    Mat img;
    while(threadContinueFlag)
    {
        try{
            // Get Image
            rmcap >> img;
            if (ConfigurationVariables::resolutionType == 0)
                resize(img,img,Size(960,720));
            // 更新当前帧的电控参数
            serialManager.UpdateReadData();
            frameData.Lock();   //frameData 加锁，防止其他线程在更改变量时调用 将串口下位机发送的指令赋值给framedata
            //{
                frameData.variable.ptzSpeed = ElectronicControlParams::PTZSpeed;
                frameData.variable.ptzAngle = ElectronicControlParams::PTZAngle;
                frameData.variable.worldPosition = ElectronicControlParams::worldPosition;
                frameData.variable.shootSpeed = ElectronicControlParams::shotSpeed;
                // 告知处理线程，图像准备完成
                frameData.variable.image = img;
                frameData.variable.index = frameIndex++;
                // DEBUG_DISPLAY(frameData.variable.image);
            //}
            frameData.Unlock(); //数据更改完成，解锁

        }
        catch(...)
        {
            cout << "Error in Collect." << endl;
        }
    }
}
//图像处理线程
void ImageProcessThread()
{
    // 初始化各模块
    char camparams[100] ;
    // //format output to camparams
    //注意如果更改了源码保存位置一定修改FILEDIR路径
    sprintf(camparams,FILEDIR(camparams_%d.xml),ConfigurationVariables::resolutionType);
    CameraView camview(camparams);// load the parameters of the camera
    camview_ptr = &camview;

    PolyMatchArmorDetector armor_detector;
    armor_detector_ptr = (ArmorBaseDetector*)(&armor_detector);
    armor_detector_ptr->SetTargetArmor(2 - ElectronicControlParams::teamInfo);

    LinearPredictor linear_predictor; 

    DnnManager dnnManager(FILEDIR(model/armor.cfg), FILEDIR(model/armor.weights)); // file names
    dnn_manager_ptr = &dnnManager;

    PredictPIDArmorTracker predict_pid_tracker(camview_ptr,armor_detector_ptr,dnn_manager_ptr,&linear_predictor);
    armor_tracker_ptr = &predict_pid_tracker;

    // 等待串口类被初始化
    while(!serial_ptr);
    
#if defined ROBOT_SENTINEL
    SentinelArmorHiter sentinelHiter(serial_ptr,armor_tracker_ptr);
    armor_hiter_ptr = &sentinelHiter;
#elif defined ROBOT_INFANCY
    InfancyArmorHiter infancyHiter(serial_ptr,armor_tracker_ptr);
    armor_hiter_ptr = &infancyHiter;

    EllipseDfcDetector dfcDetector;
    dfc_detector_ptr = &dfcDetector;
    dfc_detector_ptr->SetTargetArmor(2 - ElectronicControlParams::teamInfo); //设置敌方装甲板颜色
 #elif defined ROBOT_HERO
    InfancyArmorHiter infancyHiter(serial_ptr,armor_tracker_ptr);
    armor_hiter_ptr = &infancyHiter;

    EllipseDfcDetector dfcDetector;
    dfc_detector_ptr = &dfcDetector;
    dfc_detector_ptr->SetTargetArmor(2 - ElectronicControlParams::teamInfo); //设置敌方装甲板颜色  
#endif

    //初始化serialManager中的模块列表
    //压入2个模块 装甲识别 2； 大风车 1；
    serial_ptr->RegisterModule(armor_hiter_ptr);
    // serial_ptr->RegisterModule(dfc_hiter_ptr);

    int lastIndex = 0;
    ImageData processFrame;
    while(threadContinueFlag)
    {
        if (waitedKey == 'p')
        {
            waitedKey = 0;
            onpause = !onpause;
        }
        if (onpause) continue;
        try
        {
            // 当前图像获取线程还不能提供最新图像
            if (lastIndex >= frameData.variable.index) continue;
//LOG(A)
            // 拷贝获取线程得到的图像，防止内存错误
            frameData.Lock();
                frameData.variable.copyTo(processFrame);
            frameData.Unlock();
//LOG(B)
            lastIndex = processFrame.index;
            if (processFrame.image.empty()) break;
            UpdateFPS();
            // 处理该帧数据
            if (ConfigurationVariables::MainEntry < 4)
                ProcessFullFunction(processFrame);
            else 
                ProcessAlgorithmFunction(processFrame);

//LOG(C)
            DisplayFPS(processFrame.image);
            if(DEBUG_MODE)
                DEBUG_DISPLAY(processFrame.image);
            waitedKey = 0;

        }
        catch(...)
        {
            cout << "Error in process." << endl;
        }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
        // 串口发送当前帧数据
        serial_ptr->FlushData();
//LOG(D)
        // 每秒更新一次configuration
        if(frames == 0 && ConfigurationVariables::KeepUpdateConfiguration)
        {
            // reloading configuration variables
            ConfigurationVariables::ReadConfiguration(false);
            if (!ConfigurationVariables::Loaded) cout << "Error when update configuration variables." << endl;
        }
    }
}
//图像显示线程
void ImageDisplayThread()
{
    while(threadContinueFlag)
    {
        if (ConfigurationVariables::DebugMode)
        {
            try{
                if (!onpause || DebugDisplayManager::imgs.size())
                    DebugDisplayManager::DisplayAll();
                int c = waitKey(1);
                if (c != -1) waitedKey = (char)c;
                if (waitedKey == 'x') threadContinueFlag = false;
            }catch(...){
                cout << "Error in Display." << endl;
            }
        }
	else
	    sleep(1);
    }
}
//消息发送线程 调试用
void InfoSendThread()
{
    while(threadContinueFlag)
    {
    if(res.x != 0.0)
    {
    Sleep(50);
    serial_ptr->SendPTZAbsoluteAngle(res.x,res.y);
    if(DEBUG_MODE)
        cout<<"res.x:"<<res.x<<" res.y:"<<res.y<<endl;
    }
    }
}

int main() {
//     // init configuration
    ConfigurationVariables::ReadConfiguration(true);//读取Configuration
    #ifdef ROBOT_INFANCY
        ConfigurationVariables::ReadConfiguration(true,FILEDIR(config/config_infantry.ini));
    #elif defined ROBOT_HERO
        ConfigurationVariables::ReadConfiguration(true,FILEDIR(config/config_hero.ini));
        cout<<FILEDIR(config/config_hero.ini)<<endl;
    #elif defined ROBOT_SENTINEL
        ConfigurationVariables::ReadConfiguration(true,FILEDIR(config/config_sentinel.ini));
    #endif
    if (!ConfigurationVariables::Loaded)
        cout << "Load Configuration Failed. Using default values." << endl;
    else
        cout << "Configuration Loaded Successfully." << endl;
    ElectronicControlParams::teamInfo = ConfigurationVariables::GetInt("StartArmorType",2);

//     // init camera capture
    capture_init(rmcap);

//     // calibration process differs
    if (ConfigurationVariables::MainEntry == 10)//相机标定程序
    {
        char camparams[100] ;
        sprintf(camparams,FILEDIR(camparams_%d.xml),ConfigurationVariables::resolutionType);
        CameraView::CalibrateCameraProcess(camparams,rmcap); // write the parameters of the camera into the camparas.xml
        return 0;
    }

    thread proc_thread(ImageProcessThread);
    thread display_thread(ImageDisplayThread);
    thread collect_thread(ImageCollectThread);
    thread send_thread(InfoSendThread);

    collect_thread.join();
    proc_thread.join();
    display_thread.join();
    send_thread.join();

    return 0;
}



void ProcessFullFunction(ImageData &frame)
{
    // 调试模块的模式下 强制打开模块
    switch(ConfigurationVariables::MainEntry)
    {
    case 1: serial_ptr->EnableModule(2);break;
    case 2: serial_ptr->EnableModule(1);break;
    }
    serial_ptr->UpdateCurModule(frame,dtTime);
}

void ProcessAlgorithmFunction(ImageData &frame)
{
    switch(ConfigurationVariables::MainEntry){
        case 4:  // armor detect
            ArmorDetectDebug(frame);
            break;
        case 5: //dfc detect
            DfcDetectDebug(frame);
            break;
    }
}

void ArmorDetectDebug(ImageData &frame)
{
    res = armor_tracker_ptr->UpdateFrame(frame,dtTime) * 0.3 + frame.ptzAngle;
    // armor_detector_ptr->DetectArmors(frame.image);
    // if(armor_detector_ptr->result.size()>0)
    // {
    //     ArmorDetailType armRes[20];
    //     float confidence[20];
    //     vector<Mat> imgs;

    //     FOREACH(i,armor_detector_ptr->result.size())
    //     {
    //         // initialize res and confidence
    //         armRes[i] = ARMOR_TYPE_UNKNOWN; //armor_type 
    //         confidence[i] = 0;           //confidence
    //         Mat resized;                 //resize img to 28*28
    //         resize(armor_detector_ptr->ArmorNumberAreaGray(armor_detector_ptr->result[i]),resized,Size(28,28));
    //         imgs.push_back(resized);     //push to vector                
    //     }
    //     dnn_manager_ptr->ClassifyArmors(imgs,armRes,confidence);

    //     if(DEBUG_MODE)
    //     {
    //         FOREACH(i,imgs.size())
    //         {
    //         cout<<"armor_id:"<<armRes[i]<<"    armor_confidence:"<<confidence[i]<<endl; //查看dnn预测的装甲板id
    //         }
    //     }

    //     FOREACH(i,imgs.size())
    //     {
    //         switch(armRes[i])
    //         {
    //         case 3:case 4:case 5: armRes[i] = ARMOR_INFAN; break;
    //         case 1:armRes[i] = ARMOR_HERO; break;
    //         case 2:armRes[i] = ARMOR_ENGIN; break;
    //         default: armRes[i] = ARMOR_TYPE_UNKNOWN; break;
    //         }
    //     }
    // }
}

void DfcDetectDebug(ImageData &frame)
{
    dfc_detector_ptr->DetectDfcArmors(frame.image);
}