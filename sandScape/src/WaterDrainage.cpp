//
//  waterDrainage.cpp
//  sandScape
//
//  Created by Keru Wang on 12/11/2021.
//

#include "WaterDrainage.h"
#include "ofApp.h"

void WaterDrainage::init() {
    child = NULL;
    parent = NULL;
    pathLength = 1; // store the path length
    visited = false;
}

void WaterDrainage::findPath(int indX, int indY, ofVboMesh mainMesh, WaterDrainage* vectorList[SIZE][SIZE], ofVec2f grad[SIZE][SIZE]) {
    if(visited) return;
    slop = classification(grad[indX][indY]);
    ind.set(indX,indY);
    if(parent != nullptr) startPos = parent->endPos;
    else startPos.set(mainMesh.getVertex(indX * SIZE + indY).x,mainMesh.getVertex(indX * SIZE + indY).y); // set the start pos to its nearest vertex
    if(slop.x == 1 || slop.y == 1 || slop.x == -1 || slop.y == -1) {  // if classification result fits the standarded 4 directions
        int x = slop.x + indX;
        int y = slop.y + indY;
        if(x < SIZE && y < SIZE && x >= 0 && y >= 0) {
            ofVec2f childSlop = classification(grad[x][y]);
            if(slop.dot(childSlop) > 0) {
    //                if(mainMesh.getVertex(x * SIZE + y).z < mainMesh.getVertex(ind.x * SIZE + ind.y).z) {
                    child = vectorList[x][y];
                    vectorList[x][y]->parent = this;
                    endPos.set(mainMesh.getVertex(x * SIZE + y).x,mainMesh.getVertex(x * SIZE + y).y); // this part is slightly wrong for the case where startPos is not on a vertex
                    visited = true;
                    findPath(x,y,mainMesh,vectorList,grad);
//                    }
            }
        }
    } else {
        ofVec2f equation = constructLinearEquation(slop, startPos);
        int x, y, quadrant;
        // the indX, indY below are used as floored vertex index of the end points
        if(slop.x > 0 && slop.y < 0) { // slop pointing to the first quadrant
            x = indX + 1;
            y = indY - 1;
            quadrant = 1;
        } else if(slop.x < 0 && slop.y < 0) { // slop pointing to the second quadrant
            x = indX - 1;
            y = indY - 1;
            quadrant = 2;
        } else if(slop.x < 0 && slop.y > 0) { // slop pointing to the third quadrant
            x = indX - 1;
            y = indY + 1;
            quadrant = 3;
        } else { // slop pointing to the fourth quadrant
            x = indX + 1;
            y = indY + 1;
            quadrant = 4;
        }
        if(x < SIZE && y < SIZE && x >= 0 && y >= 0) { // if the newly connected vertex's position bound is in the range
            int newX = -1;
            int newY = -1;
//            if(mainMesh.getVertex(x * SIZE + y).z < mainMesh.getVertex(ind.x * SIZE + ind.y).z) { // form a connection only when it is from higher place to lower place
                ofVec2f bound(mainMesh.getVertex(x * SIZE + y).x,mainMesh.getVertex(x * SIZE + y).y); // what is the two lines we are trying to intersect with
                endPos = solveLinearEquation(equation, bound, quadrant); // get the intersection result
                if(endPos.y == bound.y) { // intersect with horizontal line
                    newY = y;
                    if(endPos.x > mainMesh.getVertex(ind.x * SIZE + ind.y).x) {
                        newX = ind.x;
                    } else {
                        newX = ind.x - 1;
                    }
                } else { // intersect with vertical line
                    newX = x;
                    if(endPos.y > mainMesh.getVertex(ind.x * SIZE + ind.y).y) {
                        newY = ind.y;
                    } else {
                        newY = ind.y - 1;
                    }
                }
                if(newX >= 0 && newY >= 0 && newX < SIZE && newY < SIZE) {
                    child = vectorList[newX][newY];
                    vectorList[newX][newY]->parent = this;
                    visited = true;
                    findPath(newX, newY, mainMesh, vectorList, grad);
                }
//            }
        }
    }
    if(child) child->pathLength += pathLength;
}

ofVec2f WaterDrainage::lerpGrad(ofVec2f slop0, ofVec2f slop1, ofVec2f pos0, ofVec2f pos1) {
    float ratio = ofDist(startPos.x,startPos.y,pos0.x,pos0.y)/ofDist(pos0.x, pos0.y, pos1.x, pos1.y);
    ofVec2f slop(ratio * slop1 + (1 - ratio) * slop0);
    return slop.normalize();
}

ofVec2f WaterDrainage::classification(ofVec2f initSlop) {
    float angleX = initSlop.angle(ofVec2f(0,1));
    float angleY = initSlop.angle(ofVec2f(1,0));
    float unit = 360 / NUM;
    if(angleX < 0 && angleY < 0) angleX = 180 - angleX; // third quarant
    else if(angleX < 0 && angleY > 0) angleX += 180; // second quarant
    else if(angleX > 0 && angleY < 0) angleX = 360 - angleX; // fourth quarant
    float diff = angleX - unit * floor(angleX / unit);
    if (diff > unit/2) angleX = unit * ceil(angleX / unit);
    else angleX = unit * floor(angleX / unit);
    slop = ofVec2f(0,1).getRotated(angleX);
    cout<<slop<<endl;
    return slop.normalize();
//    return ofVec2f(1,0);
}

ofVec2f WaterDrainage::constructLinearEquation(ofVec2f slop, ofVec2f point) {
    float a = slop.x;
    float b = slop.y;
    float x = point.x;
    float y = point.y;
    float t = 1000;
    if(b != 0) t = a/b;
    return ofVec2f(t, -t * x + y);
}

ofVec2f WaterDrainage::solveLinearEquation(ofVec2f line, ofVec2f bound, float quadrant) { // y = ax + b, a = line.x, b = line.y
    float a = line.x;
    float b = line.y;
    float x0 = bound.x;
    float y0 = bound.y;
    ofVec2f sol1 = ofVec2f(x0, a * x0 + b); // intersect with the vertical line
    ofVec2f sol2 = ofVec2f((y0 - b)/a, y0); // intersect with the horizontal line, y = ax + b => x = (y - b)/a
    if(quadrant == 1) { // in the first/second quadrant
        if(sol1.y > y0) return sol1;
    } else if(quadrant == 2) {
        if(sol1.y > y0) return sol1;
    } else if(quadrant == 3) {
        if(sol1.y < y0) return sol1;
    } else {
        if(sol1.y < y0) return sol1;
    }
    return sol2;
}
