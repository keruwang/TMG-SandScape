//
//  waterDrainage.h
//  sandScape
//
//  Created by Keru Wang on 12/11/2021.
//

#pragma once

#include "ofMain.h"
#define NUM 8
#define SIZE 80

class WaterDrainage {
public:
    WaterDrainage* child;
    WaterDrainage* parent;
    ofVec2f ind; // store the index of the nearest vertex --- might not be necessary
    float pathLength; // store the path length
    ofVec2f startPos; // store the position of the starting point of the vector. The end point will be the position of its child
    ofVec2f endPos;
    ofVec2f slop; // the interpolated slop
    bool visited = false;
    void init();
    void findPath(int indX, int indY, ofVboMesh mainMesh, WaterDrainage* vectorList[SIZE][SIZE], ofVec2f grad[SIZE][SIZE]); // see if there can be a path connected
    ofVec2f lerpGrad(ofVec2f slop0, ofVec2f slop1, ofVec2f pos0, ofVec2f pos1); // interpolate the gradient
    ofVec2f classification(ofVec2f initSlop);
    ofVec2f constructLinearEquation(ofVec2f slop, ofVec2f point);
    ofVec2f solveLinearEquation(ofVec2f line, ofVec2f bound, float quadrant);
};
