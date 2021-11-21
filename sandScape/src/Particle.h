//
//  Particale.h
//  sandScape
//
//  Created by Keru Wang on 13/6/2021.
//

#pragma once

#include "ofMain.h"

class Particle {
public:
    
    
    void setup(ofVec3f startPos, ofVec3f c);
    void move(ofVec3f dir);
    void draw(float scale);
    void stayOnScreen(float w, float h);
    bool isOffScreen(float w, float h);
    bool isDead();
    void updateDuration(float speed);
    
    ofVec3f pos;
    ofVec3f speed;
    ofVec3f color;
    float size;
    int duration;
    
};
