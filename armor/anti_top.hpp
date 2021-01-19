/*
#pragma once

#include <cmath>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "util.hpp"
#include "configurations.hpp"
#include "armorDetector.hpp"
#include <cmath>
#include <queue>

using namespace std;
using namespace cv;

typedef double systime

double get_pointlen(const Point2f& a){
    return sqrt(a.x*a.x+a.y*a.y);
}


static systime get_systime(){
    timeval time_base;
    gettimeofday(&time_base,nullptr);
    return time_base.tv_sec*1000+time_base.tv_usec/1000;
}

void get_systime(systime& t){
    static systime time_base=get_systime();
    timeval tv;
    gettimeofday(&tv,nullptr);
    t=tv.tv_sec*1000+tv.tv_usec/1000-time_base;
}

class anti_top : public ArmorTrackerBase{
private:
    ArmorResult last_box;
    ArmorResult target_box;
    systime last_time,now_time;
public:
    anti_top(ArmorResult last_Armor){
        last_box=last_Armor;
        last_time=get_systime();
    }

    ~anti_top(){

    }

    bool is_top(){
        const int LOST=5;
        const int TOP;
        const double detla_h;
        const double detla_w;
        static queue <ArmorResult> armor_seq;
        int lost_frame_cnt=0;
        static int top=0;
        if(target_box==ArmorResult()){
            lost_frame_cnt++;
            target_box=last_box;
            if(lost_frame_cnt>LOST){
                top=0;
                track=0;
                return false;
            }
        }
        else{
            armor_seq.push(target_box);
            if(armor_seq.size()<2){
                return false;
            }
            else{
                if((last_box.rr.height-target_box.rr.height)>detla_h){
                    top=0;
                    while(!armor_seq.empty()){
                        armor_seq.pop();
                    }
                    return false;
                }
                if((last_box.center-target_box.center)>detla_w){
                    top++;
                    if(top<TOP){
                        return false;
                    }   
                }
            }
            
        }
        last_box=target_box;
        if(armor_seq.size()>=5){
            arm0r_seq.pop();
        }
        return true;
    }

    anti_top_run(){
        
    }
};
*/