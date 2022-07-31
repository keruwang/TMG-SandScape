//
//  Particale.cpp
//  sandScape
//
//  Created by Keru Wang on 13/6/2021.
//

#include "Particle.h"
#include "ofApp.h"

void Particle::setup(ofVec3f startPos, ofVec3f c){
    pos = startPos;
    speed.set(0,0,0);
//    color.set(ofRandom(0, 100),ofRandom(100, 200),ofRandom(200, 255));
    color.set(c);
    duration = ofRandom(0,500);
    size = 1.;
}

void Particle::move(ofVec3f dir){
//    pos += dir;
    speed += dir;
    pos += 0.5 * speed.dot(speed) * speed.normalize();
}

void Particle::draw(float scale){
    ofPushStyle();
    ofSetColor(color.x, color.y, color.z);
//    ofDrawCircle(125 + pos.x * 5, 125 + pos.y * 5, 0,scale * size);
//    ofDrawRectangle(pos.x,pos.y,pos.z,scale * size,scale * size * duration / 10);
    ofDrawCircle(pos.x,pos.y,pos.z,scale * size);
//    ofDrawRectangle(pos.x, pos.y, pos.z, scale * size, 10 * scale * size);
    ofPopStyle();
}

void Particle::updateDuration(float speed){
    duration += speed;
}

void Particle::stayOnScreen(float w, float h){
    if( pos.x < 0 ) pos.x += w;
    if( pos.x >= w ) pos.x -= w;
    if( pos.y < 0 ) pos.y += h;
    if( pos.y >= h ) pos.y -= h;
}

bool Particle::isOffScreen(float w, float h){
    if( pos.x < 0 || pos.x >= w || pos.y < 0 || pos.y >= h ){
        return true;
    }
    
    return false;
}

bool Particle::isDead(){
    return duration >= 100;
}