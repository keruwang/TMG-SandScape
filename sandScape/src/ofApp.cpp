#include "ofApp.h"

float u_mode = 0;
int season = 0;
int waterMode = 0;
float snowHeight = 0;
float meshX = SIZE;
float meshY = SIZE;
float meshZ = SIZE/5;
float cloudAmount = 5;
float specular = 1.;

//--------------------------------------------------------------
void ofApp::setup(){
    DISPLAY_WIDTH = ofGetScreenWidth();
    DISPLAY_HEIGHT = ofGetScreenHeight();
    // initsialize the standard vectors
    up.set(0,-1);
    down.set(0,1);
    left.set(-1,0);
    right.set(1,0);
    // window display setup
    ofSetWindowShape(5000, 1000);
    ofSetWindowPosition(0, 0);
    //    ofSetLogLevel(OF_LOG_VERBOSE);
    
    // enable depth->video image calibration
    kinect.setRegistration(true);
    kinect.init();
    //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
    
    kinect.open();        // opens first available kinect
    //kinect.open(1);    // open a kinect by id, starting with 0 (sorted by serial # lexicographically))
    //kinect.open("A00362A08602047A");    // open a kinect using it's unique serial #
    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
    grayImage.allocate(kinect.width, kinect.height);
    grayImageSmall.allocate(SIZE, SIZE);
    grayOverall.allocate(200, 200);
    grayOverall.set(1);
    grayImage_avg.allocate(200, 200);
    grayImage_avg.set(1);
    
    
    // get current time
    startTimeout = ofGetElapsedTimef();
    // load the shaders
    shader.load("shader");
    sunShader.load("sunShader");
//    cloudShader.load("cloudShader");
    rainShader.load("rainShader");
    
    // set the initial values to use for our perlinNoise
    perlinRange = 2;
    perlinHeight = meshZ;
    mainCam.setPosition(0, 0, 80); // set initial position for oureasyCam 3D viewer
    mainCam.setOrientation(ofVec3f(0,0,0));
    
    // create the mesh
    for (int y = 0; y < meshY; y ++){
        for (int x = 0; x < meshX; x ++){
            mainMesh.addVertex(ofPoint(x - meshX / 2, y - meshY / 2, 0));
            mainMesh.addTexCoord(ofVec2f(x,y));
        }
    }
    
    
    for (int x = 0; x < meshX - 1; x ++){
        for (int y = 0; y < meshY - 1; y ++){
            mainMesh.addIndex(x + y * meshX);                // 0
            mainMesh.addIndex((x + 1) + y * meshX);            // 1
            mainMesh.addIndex(x + (y + 1) * meshX);            // 10
            
            mainMesh.addIndex((x + 1) + y * meshX);            // 1
            mainMesh.addIndex((x + 1) + (y + 1) * meshX);        // 11
            mainMesh.addIndex(x + (y + 1) * meshX);            // 10
        }
    }
    
    // distort the z value of each point in the mesh with perlinNoise
    int i = 0;
    for (int y = 0; y < meshY; y ++){
        for (int x = 0; x < meshX; x ++){
            ofVec3f newPosition = mainMesh.getVertex(i);
            newPosition.z = ofNoise(ofMap(x, 0, meshX, 0, perlinRange),  ofMap(y, 0, meshY, 0, perlinRange) ) * perlinHeight;
            mainMesh.setVertex(i, newPosition);
            i ++;
        }
    }
    
    gravity.set(0.,0.,-1.);
    wind.set(1.,1.,0.);
    
    // get vertex and face of the mesh
    V.resize(mainMesh.getNumVertices(),3);
    F.resize(mainMesh.getNumIndices() / 3,3);
    N.resize(mainMesh.getNumVertices(),3);
    normals = new float[3 * mainMesh.getNumVertices()];
    directions = new float[3 * mainMesh.getNumVertices()];
    randOffset = new float[mainMesh.getNumVertices()];
    //    curvatures = new float[mainMesh.getNumIndices() / 3];
    slope = new ofVec3f[mainMesh.getNumVertices()];
    //    windField = new ofVec3f[mainMesh.getNumVertices()];
    updateMeshInfo(V, F, N, normals);
    // store the barycenter for the drawing of the wind field
    //    igl::barycenter(V, F, BC);
    // construct the grad field using curvature value
    //    constructGradField(V,F,gradU,gradV, curU, curV);
    // construct the "wind field" with harmonic paraneterization
    //    constructWindField(V, F, windU, windV);
    for (int i = 0; i < mainMesh.getNumVertices(); i ++) {
        ofVec3f temp;
        temp.set(N.row(i)[0],N.row(i)[1],N.row(i)[2]);
        slope[i].set(gravity - (temp * gravity)[2] * temp);
        directions[3 * i] = slope[i].x;
        directions[3 * i + 1] = slope[i].y;
        directions[3 * i + 2] = slope[i].z;
    }
    gridNum = meshX/gridSize;
    for (int i = 0; i < gridNum; i ++) {
        for (int j = 0; j < gridNum; j ++) {
            int index = i * gridSize * meshX + j * gridSize;
            grad[i][j].set(slope[index].x,slope[index].y);
            if(grad[i][j].length() > 0.0)
                grad[i][j] = grad[i][j].getNormalized();
            else grad[i][j].set(ofNoise(10 * i),ofNoise(10 * j));
        }
    }
    
    for (int i = 0; i < mainMesh.getNumVertices(); i ++) {
        randOffset[i] = (int)ofRandom(8);
    }
    
    gui.setup();
    gui.add(nearThreshold.setup("nearThreshold", nearThreshold_, 0, 255));
    gui.add(farThreshold.setup("farThreshold", farThreshold_, 0, 255));
    gui.add(xLeftBoundary.setup("xLeftBoundary", xLeftBoundary_, 0, grayImage.width));
    gui.add(xRightBoundary.setup("xRightBoundary", xRightBoundary_, 0, grayImage.width));
    gui.add(yTopBoundary.setup("yTopBoundary", yTopBoundary_, 0, grayImage.height));
    gui.add(yBottomBoundary.setup("yBottomBoundary", yBottomBoundary_, 0, grayImage.height));
    gui.add(scale.setup("scale", scale_, 0, 5));
    gui.add(topViewX.setup("topViewX", topViewX_, 0, 5000));
    gui.add(topViewY.setup("topViewY", topViewY_, 0, 1000));
    gui.add(smoothRound.setup("smoothRound", smoothRound_, 0, 25));
    gui.add(smoothWeight.setup("smoothWeight", smoothWeight_, 0, 10));
    gui.add(transitionSpeed.setup("transitionSpeed", transitionSpeed_, 0, 1));
    gui.add(shadowContrast.setup("shadowContrast", specular_, 0, 100));
    gui.add(grid.setup("gridSize", gridSize_, 1, 4));
    gui.add(dotSize.setup("dotSize", dotSize_, 1, 5));
    gui.add(lineWidth.setup("lineWidth", lineWidth_, 4, 14));
    gui.add(rotX.setup("rotX", rotX_, -180, 180));
    gui.add(posX.setup("posX", posX_, -30, 30));
    gui.add(posY.setup("posY", posY_, -30, 30));
    gui.add(posZ.setup("posZ", posZ_, -30, 30));

    
    ofSetBackgroundColor(0, 0, 0);
    
    fbo_water.allocate(kinect.width,kinect.height);
    fbo_water.begin();
    ofClear(0,0,0,0);
    fbo_water.end();
    fbo_particle.allocate(kinect.width,kinect.height);
    fbo_particle.begin();
    ofClear(255,255,255,0);
    fbo_particle.end();
    fbo_topView.allocate(meshX * 10,meshY * 10);
    fbo_topView.begin();
    ofClear(255,255,255,0);
    fbo_topView.end();
    fbo_sideView.allocate(meshX * 20,meshY * 20);
    fbo_sideView.begin();
    ofClear(255,255,255,0);
    fbo_sideView.end();
    fbo_grad.allocate(meshX * 10,meshY * 10);
    fbo_grad.begin();
    ofClear(0,0,0,0);
    fbo_grad.end();
    fbo_rainBack.allocate(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    fbo_rainBack.begin();
    ofClear(0,0,0,0);
    fbo_rainBack.end();
    fbo_rainMid.allocate(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    fbo_rainMid.begin();
    ofClear(0,0,0,0);
    fbo_rainMid.end();
    fbo_rainFront.allocate(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    fbo_rainFront.begin();
    ofClear(0,0,0,0);
    fbo_rainFront.end();
    
    sun.setRadius(5);
    rainPlane.set(DISPLAY_WIDTH,DISPLAY_HEIGHT);
    rainPlane.setPosition(DISPLAY_WIDTH/2, DISPLAY_HEIGHT/2,0);
    rainPlane.setResolution(2,2);
    
    for(int i = 0; i < SIZE; i ++) {
        for(int j = 0; j < SIZE; j ++) {
            vectorList[i][j] = new WaterDrainage();
        }
    }
    cloudVideo.load("cloud.mp4");
    cloudVideo.setLoopState(OF_LOOP_NORMAL);
}

//--------------------------------------------------------------
void ofApp::update(){
    ofSetWindowShape(5000, 1000);
    ofSetWindowPosition(0, 0);
    
    if(kinect.isConnected()) {
        kinect.update();
        if (kinect.isFrameNew()) {
            grayImage.setFromPixels(kinect.getDepthPixels());
            // crop out the valid part
            pre_pix = grayImage.getPixels().getData();
            cropped_w = abs(xRightBoundary - xLeftBoundary);
            cropped_h = abs(yBottomBoundary - yTopBoundary);
            cropped = new unsigned char[cropped_w * cropped_h];
            for(int x = 0; x < cropped_w; x++) {
                for(int y = 0; y < cropped_h; y++) {
                    int index_large = (y + yTopBoundary) * grayImage.getWidth() + (x + xLeftBoundary);
                    int index_small = y * cropped_w + x;
                    cropped[index_small] = pre_pix[index_large];
                    
                }
            }
            croppedImg.setFromPixels(cropped,cropped_w, cropped_h);
            grayImageSmall.scaleIntoMe(croppedImg);
            free(cropped);
            // add effects to smooth signal and reduce noise
            grayImageSmall.blur(3);
            pix = grayImageSmall.getPixels().getData();

            // draw image
            int diffThreshold = nearThreshold - farThreshold;

            // iterate through pixel data
            for (int w = 0; w < grayImageSmall.getWidth(); w++) {
                for (int h = 0; h < grayImageSmall.getHeight(); h++) {
                    // average previous pixels with current reading
                    int index = h * grayImageSmall.getWidth() + w;
                    double currentDepth = pix[index];
                    double prevDepth = prevPix[index];
                    prevPix[index] = prevDepth * .9 + currentDepth * .1;
                    double  depth = prevPix[index];
                    //                        double depth = currentDepth;

                    bool isInDepthBoundary = (currentDepth < nearThreshold) && (currentDepth > farThreshold);

                    // set Timeout zone
                    //                        int lowerBoundary = 210;
                    //                        int upperBoundary = 400;
                    //
                    //                        bool isInActiveZone = currentDepth < upperBoundary && currentDepth > lowerBoundary;

                    //                if (isInBoundary && isInActiveZone) {
                    //                    isTimeout = false;
                    //                    startTimeout = ofGetElapsedTimef();
                    //                }
                    //                if (ofGetElapsedTimef() == startTimeout + timeout) {
                    //                    isTimeout = true;
                    //                }
                    
                    // set pixels of colorImg based on depth ratio
                    if (isInDepthBoundary) {//} && !isTimeout) { // ***
                        double diffToThreshold = depth - farThreshold;
                        double ratioInDepth = diffToThreshold/diffThreshold;
                        if (ratioInDepth > 0.95) {
                            ratioInDepth = 0.95;
                        } else if (ratioInDepth < 0) {
                            ratioInDepth = 0;
                        }
                        
                        ofVec3f tmpVec = mainMesh.getVertex(h * (grayImageSmall.width) + grayImageSmall.getWidth() - w);
                        tmpVec.z = sqrt(ratioInDepth) * meshZ;
                        mainMesh.setVertex((h)*(grayImageSmall.width)+(grayImageSmall.getWidth() - w), tmpVec);
                    } else {
                        
                        ofVec3f tmpVec = mainMesh.getVertex(h * grayImageSmall.width + (grayImageSmall.getWidth() - w));
                        if(currentDepth < nearThreshold) tmpVec.z = 0;
                        else if(currentDepth > farThreshold) tmpVec.z = meshZ;
                        
                        mainMesh.setVertex( h * grayImageSmall.width + grayImageSmall.getWidth() - w , tmpVec);
                    }
                    
                }
            }
        }
        // smooth mesh along y
        for (int z = 0; z < smoothRound; z ++) {
            for (int x = 0; x < meshX; x ++) {
                for (int y = 1; y < meshY - 1; y ++) {
                    int index = x * meshX + y;
                    ofVec3f a = mainMesh.getVertex(index - 1);
                    ofVec3f b = mainMesh.getVertex(index);
                    ofVec3f c = mainMesh.getVertex(index + 1);
                    mainMesh.setVertex(index, (a + c + smoothWeight * b)/(2 + smoothWeight));
                }
            }
        }
        // smooth mesh along x
        for (int z = 0; z < smoothRound; z ++) {
            for (int y = 0; y < meshY; y ++) {
                for (int x = 1; x < meshX - 1; x ++) {
                    int index = x * meshX + y;
                    ofVec3f a = mainMesh.getVertex(index - meshY);
                    ofVec3f b = mainMesh.getVertex(index);
                    ofVec3f c = mainMesh.getVertex(index + meshY);
                    mainMesh.setVertex(index, (a + c + smoothWeight * b)/(2 + smoothWeight));
                }
            }
        }
    } else {
//        // distort the z value of each point in the procedural mesh with perlinNoise
//        int i = 0;
//        for (int y = 0; y < meshY; y ++){
//            for (int x = 0; x < meshX; x ++){
//                ofVec3f newPosition = mainMesh.getVertex(i);
//                newPosition.z = ofNoise(ofMap(x + ofGetElapsedTimef(), 0, meshX, 0, perlinRange),  ofMap(y + ofGetElapsedTimef(), 0, meshY, 0, perlinRange) ) * perlinHeight;
//                mainMesh.setVertex(i, newPosition);
//                i ++;
//            }
//        }
    }
    // update the reconstructed mesh
    updateMeshInfo(V, F, N, normals);
    for (int i = 0; i < mainMesh.getNumVertices(); i ++) {
        ofVec3f temp;
        temp.set(N.row(i)[0],N.row(i)[1],N.row(i)[2]);
        slope[i].set(gravity - (temp * gravity)[2] * temp);
        directions[3 * i] = slope[i].x;
        directions[3 * i + 1] = slope[i].y;
        directions[3 * i + 2] = slope[i].z;
    }
    // construct the gradient field for waterdrainage display
    for (int i = 0; i < gridNum; i ++) {
        for (int j = 0; j < gridNum; j ++) {
            int index = i * gridSize * meshX + j * gridSize;
            grad[i][j].set(slope[index].x,slope[index].y);
            if(grad[i][j].length() > 0.)
                grad[i][j] = grad[i][j].getNormalized();
            else grad[i][j].set(ofNoise(10 * i),ofNoise(10 * j));
        }
    }
    // close the boundary of the landscape
    for (int x = 1; x < meshX - 1; x ++) {
        int index1 = x * meshX;
        int index2 = x * meshX + meshY - 1;
        ofVec3f ver1 = mainMesh.getVertex(index1);
        ofVec3f ver2 = mainMesh.getVertex(index2);
        ver1.z = 0;
        ver2.z = 0;
        mainMesh.setVertex(index1, ver1);
        mainMesh.setVertex(index2, ver2);
    }
    
    for (int y = 1; y < meshY - 1; y ++) {
        int index1 = y;
        int index2 = (meshX - 1) * meshX + y;
        ofVec3f ver1 = mainMesh.getVertex(index1);
        ofVec3f ver2 = mainMesh.getVertex(index2);
        ver1.z = 0;
        ver2.z = 0;
        mainMesh.setVertex(index1, ver1);
        mainMesh.setVertex(index2, ver2);
    }
    
    // update values from GUI
    float cloudChangeSpeed = transitionSpeed;
    cloudAmount = ofClamp(2.5 + 5 * sin(transitionSpeed * ofGetElapsedTimef()),0,5); // period = 2 * PI / cloud
    specular = shadowContrast;
    waterVectorDis = grid;
    // move the sun in shadow shading 3D display
    sun.setPosition(50 * cos(ofGetElapsedTimef()),0,50 * sin(ofGetElapsedTimef()));
    
    //---------------------------------- season change -----------------------------------
    //    if (season == 0) {
    //        if (snowHeight >= 0) {
    //            snowHeight -= 0.006 * (snowHeight);
    //            for(int i = 0; i < 5; i ++) {
    //                Particle newParticle;
    //                int rand = ofRandom(mainMesh.getNumVertices());
    //                ofVec3f pos = mainMesh.getVertex(rand);
    //                if (pos.z > meshZ - snowHeight + 1) {
    //                    ofVec3f white;
    //                    white.set(255, 255, 255);
    //                    newParticle.setup(pos, white);
    //                    particles.push_back(newParticle);
    //                }
    //            }
    //
    //            for (int i = 0; i < particles.size(); i ++) {
    //                particles[i].updateDuration(0.5);
    //                if (particles[i].isDead()) {
    //                    particles.erase(particles.begin() + i);
    //                }
    //                ofVec3f moveDir;
    //                for (int j = 0; j < mainMesh.getNumVertices(); j ++) {
    //                    if (particles[i].pos.distance(mainMesh.getVertex(j)) < 1.2) {
    //                        moveDir.set(slope[j]);
    //                        break;
    //                    }
    //                }
    //                particles[i].move(moveDir);
    //            }
    //        }
    //
    //        if (flowers.size() < 400) {
    //            Particle newParticle;
    //            int rand = ofRandom(mainMesh.getNumVertices());
    //            ofVec3f pos = mainMesh.getVertex(rand);
    //               if(pos.z < 7) {
    //                   ofVec3f color;
    //                   color.set(ofRandom(240, 255), ofRandom(150, 200),ofRandom(220, 230));
    //                   newParticle.setup(pos + 5 * ofNoise(pos), color);
    //                   flowers.push_back(newParticle);
    //               }
    //            rand = ofRandom(mainMesh.getNumVertices());
    //            pos = mainMesh.getVertex(rand);
    //               if(pos.z < 4) {
    //                   ofVec3f color;
    //                   color.set(ofRandom(240, 255), ofRandom(150, 200),ofRandom(220, 230));
    //                   newParticle.setup(pos + 5 * ofNoise(pos), color);
    //                   flowers.push_back(newParticle);
    //               }
    //         }
    //        for( int i = 0; i < flowers.size(); i ++) {
    //            flowers[i].updateDuration(0.5);
    //            if(flowers[i].isDead()) {
    //                flowers.erase(flowers.begin() + i);
    //            }
    //        }
    //    } else if(season == 1) {
    //        snowHeight = 0;
    //        particles.clear();
    //        fbo_water.begin();
    //        ofClear(255,255,255, 0);
    //        fbo_water.end();
    //        if (flowers.size() < 600){
    //         Particle newParticle;
    //         int rand = ofRandom(mainMesh.getNumVertices());
    //         ofVec3f pos = mainMesh.getVertex(rand);
    //            while(pos.z < 7) {
    //                rand = ofRandom(mainMesh.getNumVertices());
    //                pos = mainMesh.getVertex(rand);
    //            }
    //         ofVec3f color;
    //         color.set(ofRandom(240, 255), ofRandom(150, 200),ofRandom(220, 230));
    //         newParticle.setup(pos, color);
    //         flowers.push_back(newParticle);
    //         }
    //        for( int i = 0; i < flowers.size(); i ++) {
    //            flowers[i].updateDuration(1);
    //            if(flowers[i].isDead()) {
    //                flowers.erase(flowers.begin() + i);
    //            }
    //            ofVec3f moveDir;
    //            for( int j = 0; j < mainMesh.getNumVertices(); j ++) {
    //                if(flowers[i].pos.distance(mainMesh.getVertex(j)) < 1.2) {
    //                    moveDir.set(slope[j]);
    //                    break;
    //                }
    //            }
    //            ofVec3f w;
    //            w.set(1. + ofNoise(flowers[i].pos.x,flowers[i].pos.y,ofGetElapsedTimef()),1.+ ofNoise(flowers[i].pos.x,flowers[i].pos.y,ofGetElapsedTimef()),0.);
    //            flowers[i].move((moveDir + 0.1 * w));
    //        }
    //    } else {
    //        if(snowHeight < 7) snowHeight += 0.002*(7 - snowHeight);
    //        flowers.clear();
    //    }
    //---------------------------------- draw water drainage vectors based on 8 classification -----------------------------------
    
    fbo_grad.begin();
    ofPushStyle();
    ofSetColor(0, 0, 0);
    ofDrawRectangle(0, 0, meshX * 10, meshY * 10);
 
    int temp = waterVectorDis;
    if(ofGetFrameNum() % 70 == 0) {
        for (int i = 0; i < gridNum; i += temp) {
            for (int j = 0; j < gridNum; j += temp) {
                visitedList[i][j].set(0,-1,-1);
            }
        }
        for (int i = 0; i < gridNum; i += temp) {
            for (int j = 0; j < gridNum; j += temp) {
                findPath(i,j);
            }
        }
    }
    
    for (int i = 0; i < gridNum; i +=temp) {
        for (int j = 0; j < gridNum; j +=temp) {
            int indOff = i * meshX * gridSize + j * gridSize;
            if(visitedList[i][j].y == -1) {  // it is the root: parent = -1
                visitedList[i][j].x = 0;
                ofVec3f currentNode = visitedList[i][j];
                int travelStep = 0;
                int ind1 = i * meshX * gridSize + j * gridSize;
                int ind2;
                while(currentNode.z != -1) { // it still has child
                    ind2 = currentNode.z;
                    int ind2X = floor(ind2 * gridSize / meshX);
                    int ind2Y = ind2 - meshX * ind2X;
                    float offSet = meshX * 5;
                    float x1 = offSet + mainMesh.getVertex(ind1).x * 10;
                    float y1 = offSet + mainMesh.getVertex(ind1).y * 10;
                    float x2 = offSet + mainMesh.getVertex(ind2).x * 10;
                    float y2 = offSet + mainMesh.getVertex(ind2).y * 10;
                    float stiff =  mainMesh.getVertex(ind1).z -  mainMesh.getVertex(ind2).z;
                    int ind = ofClamp((int)(stiff * 5 / (gridSize * gridSize)) ,0, 10);
                    if(waterMode < 2) ofSetColor(waterColor[ind][0],waterColor[ind][1],waterColor[ind][2]);
                    else ofSetColor(255,255,255);
                    float dx = x2 - x1;
                    float dy = y2 - y1;
                    float d = ofDist(x1, y1, x2, y2);
                    int circleNum = lineWidth * gridSize;
                    for(int k = 0; k < circleNum; k ++) {
                        int dn = 2 * (circleNum - 1);
                        float x = x1 + 2 * k * dx/dn;
                        float y = y1 + 2 * k * dy/dn;
                        ofDrawCircle(x + 1.8 * (-1 + 3. * ofNoise(y/30)),y + 3. * (-1 + 3. * ofNoise(x/30)), d/(dn - 1));
                    }
                    float time = ofClamp(0.5 * ind, 1, 3) * ofGetElapsedTimef();
                    float ratio = (time - floor(time));
                    if((travelStep + (int)randOffset[indOff]) % (3 + (int)transitionSpeed) == (int)time % (3 + (int)transitionSpeed)) {
                        ofPushStyle();
                        if(waterMode == 1 || waterMode > 1) ofSetColor(0, 100, 255);
                        float x = x1 + ratio * dx;
                        float y = y1 + ratio * dy;
                        ofDrawCircle(x + 1.8 * (-1 + 3. * ofNoise(y/30)),y + 1.8 * (-1 + 3. * ofNoise(x/30)), dotSize);
                        float xb = x1 + (ratio-0.15) * dx;
                        float yb = y1 + (ratio-0.15) * dy;
                        float xa = x1 + (ratio+0.15) * dx;
                        float ya = y1 + (ratio+0.15) * dy;
                        ofDrawCircle(xb + 1.5 * (-1 + 3. * ofNoise(yb/30)),yb + 1.5 * (-1 + 3. * ofNoise(xb/30)), dotSize - 0.5);
                        ofDrawCircle(xa + 1.5 * (-1 + 3. * ofNoise(ya/30)),ya + 1.5 * (-1 + 3. * ofNoise(xa/30)), dotSize - 0.5);
                        ofPopStyle();
                    }
                    currentNode = visitedList[ind2X][ind2Y];
                    visitedList[ind2X][ind2Y].x = 0;
                    ind1 = ind2;
                    travelStep ++;
                }
            }
        }
    }
    ofPopStyle();
    fbo_grad.end();
    
    //---------------------------------------------------------- rain ----------------------------------------------------------
    float rainAmount = ofClamp(2.5 + 7 * sin(transitionSpeed * ofGetElapsedTimef()),0,5);
    if(rainAmount < 3) {
        for(int i = 0; i < ofRandom((5 - rainAmount) * 8); i ++) {
            if(rain.size() < (5 - rainAmount) * 100) {
                Particle newParticle;
                float rainRange = (5 - rainAmount) * DISPLAY_WIDTH/ 6;
                float rand = DISPLAY_WIDTH/2 + ofRandom(-rainRange/2, rainRange/2);
                ofVec3f pos(rand, DISPLAY_HEIGHT - ofRandom(100) - DISPLAY_HEIGHT/8, 0);
                ofVec3f color(200,200,255);
                newParticle.setup(pos, color);
                rain.push_back(newParticle);
            }
        }
    }
    
    float duration = cloudVideo.getDuration();
    float period = 2 * PI / transitionSpeed;
    float speed = duration / period;
    cloudVideo.setSpeed(speed);
    cloudVideo.update();
    if(!resetVideo && cloudAmount == 5) {
        cloudVideo.setFrame(0);
        resetVideo = true;
    }
    if(cloudAmount != 5) resetVideo = false;
}

//--------------------------------------------------------------
void ofApp::draw(){
    // water flow
    //    fbo_water.begin();
    //    ofPushStyle();
    //    for( int i = 0; i < particles.size(); i ++){
    //        particles[i].draw(4);
    //    }
    //    ofPopStyle();
    //    ofPushStyle();
    //    ofPopStyle();
    //    fbo_water.end();
    
    // flowers
    fbo_particle.begin();
    ofPushStyle();
    ofSetColor(0, 0, 0);
    ofDrawRectangle(0, 0, kinect.width, kinect.height);
    ofPopStyle();
    ofPushStyle();
    for( int i = 0; i < flowers.size(); i ++){
        flowers[i].draw(2.);
    }
    ofPopStyle();
    fbo_particle.end();
    
    // render top view
    fbo_topView.begin();
    ofPushStyle();
    ofSetColor(0,0,0);
    ofDrawRectangle(0, 0, kinect.width, kinect.height);
    ofPopStyle();
    ofTranslate(10*meshX/2, 10*meshY/2);
    ofScale(10,-10,1);
    int normalLoc = shader.getAttributeLocation("normal");
    mainMesh.getVbo().setAttributeData(normalLoc, normals, 3, 3 * mainMesh.getNumVertices(), GL_STATIC_DRAW);
    int dirLoc = shader.getAttributeLocation("direction");
    mainMesh.getVbo().setAttributeData(dirLoc, normals, 3, 3 * mainMesh.getNumVertices(), GL_STATIC_DRAW);
    shader.begin();
    shader.setUniformTexture("waterTexture", fbo_grad.getTexture(0), 1 );
    shader.setUniformTexture("particleTexture", fbo_particle.getTexture(0), 2 );
    mainMesh.drawFaces();
    shader.end();
    fbo_topView.end();
    fbo_topView.getTexture(0).draw(topViewX, topViewY, scale * 10 * meshX,scale * 10 * meshY);

    // render side view
    fbo_sideView.begin();
    ofPushStyle();
    
    ofClear(0,0,0,0);
    mainCam.begin();
    shader.begin();
    ofEnableDepthTest();
    ofVec3f trans(posX,posY,posZ);
    ofTranslate(trans);
    ofRotateZDeg(rotX);
    mainMesh.drawFaces();
    shader.end();
    //    renderVectorField(V, F, gradV, curV, windU, slope, 0); // for debugging the slope vector field
    if(u_mode == 3){
        sunShader.begin();
        sun.draw();
        sunShader.end();
    }
    ofDisableDepthTest();

    mainCam.end();
    ofPopStyle();
    fbo_sideView.end();

    // render the rain on of-screen frames: render the raindrops on 3 planes to create the illusion of 3D space
    // rain at the back: slower speed and smaller size
    fbo_rainBack.begin();
    for(int j = 0; j < 5; j ++) {
        for(int i = 0; i < floor(rain.size()/3); i ++) {
            if(rain[i].pos.y < DISPLAY_HEIGHT/2 ) {
                rain.erase(rain.begin() + i);
            }
            ofVec3f g(-0.32,-0.8,0);
            rain[i].move(g);
        }
        ofPushStyle();
        ofSetColor(255, 255, 255, 10);
        ofDrawRectangle(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
        for( int i = 0; i < floor(rain.size()/3); i ++){
            rain[i].draw(0.6);
        }
        ofPopStyle();
    }
    fbo_rainBack.end();
    // rain at the middle
    fbo_rainMid.begin();
    for(int j = 0; j < 10; j ++) {
        for(int i = floor(rain.size()/3); i < floor(2 * rain.size()/3); i ++) {
            if(rain[i].pos.y < DISPLAY_HEIGHT/4 + i) {
                rain.erase(rain.begin() + i);
            }
            ofVec3f g(-0.4,-1,0);
            rain[i].move(g);
        }
        ofPushStyle();
        ofSetColor(255, 255, 255, 10);
        ofDrawRectangle(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
        for( int i = floor(rain.size()/3); i < floor(2 * rain.size()/3); i ++){
            rain[i].draw(1.2);
        }
        ofPopStyle();
    }
    fbo_rainMid.end();
    // rain at the front: faster speed and larger size
    fbo_rainFront.begin();
    for(int j = 0; j < 15; j ++) {
        for(int i = floor(2 * rain.size()/3); i < rain.size(); i ++) {
            if(rain[i].pos.y < 0) {
                rain.erase(rain.begin() + i);
            }
            ofVec3f g(-0.6,-1.5,0);
            rain[i].move(g);
        }
        ofPushStyle();
        ofSetColor(255, 255, 255, 10);
        ofDrawRectangle(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
        for( int i = floor(2 * rain.size()/3); i < rain.size(); i ++){
            rain[i].draw(1.8);
        }
        ofPopStyle();
    }
    fbo_rainFront.end();
    
    // if in rain->elevation transition mode
    if(u_mode == 1) {
        // rain at the back
        rainShader.begin();
        rainPlane.mapTexCoordsFromTexture(fbo_rainBack.getTexture(0));
        rainShader.setUniformTexture("rainTexture", fbo_rainBack.getTexture(0),1);
        rainShader.setUniformTexture("meshTexture",fbo_sideView.getTexture(0),2);
        rainPlane.draw();
        rainShader.end();
        // cloud
        cloudVideo.draw(-DISPLAY_WIDTH * 0.1, -50, DISPLAY_WIDTH * 1.2, DISPLAY_HEIGHT/3);
        // the landscape
        fbo_sideView.getTexture(0).draw(0, 100, 1680, 1300);
        // rain at the middle
        rainShader.begin();
        rainPlane.mapTexCoordsFromTexture(fbo_rainFront.getTexture(0));
        rainShader.setUniformTexture("rainTexture", fbo_rainMid.getTexture(0),1);
        rainShader.setUniformTexture("meshTexture",fbo_sideView.getTexture(0),2);
        rainPlane.draw();
        rainShader.end();
        // rain at the front
        rainShader.begin();
        rainPlane.mapTexCoordsFromTexture(fbo_rainFront.getTexture(0));
        rainShader.setUniformTexture("rainTexture", fbo_rainFront.getTexture(0),1);
        rainShader.setUniformTexture("meshTexture",fbo_sideView.getTexture(0),2);
        rainPlane.draw();
        rainShader.end();
    } else fbo_sideView.getTexture(0).draw(0, 100, 1680, 1300);
    
    if(!bHide){
        gui.draw();
    }
}

//--------------------------------------------------------------
void ofApp::exit() {
    kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();
}

//--------------------- do 8 classification -----------------------------------------
void ofApp::findPath(int indX, int indY) {
    int temp = waterVectorDis;
    // visitedList vec3
    // digit 1: visited or not; digit 2: parent, digit 3: child
    if(visitedList[indX][indY].x == 0) {
        int ind1 = indX * gridSize * meshX + indY * gridSize;
        visitedList[indX][indY].x ++; // marked as visited
        ofVec2f gradient1 = grad[indX][indY];
        int indXX = indX;
        int indYY = indY;
        if(gradient1.dot(up) > 0.5) indXX -=temp;
        else if(gradient1.dot(down) > 0.5) indXX +=temp;
        if(gradient1.dot(right) > 0.5) indYY +=temp;
        else if(gradient1.dot(left) > 0.5) indYY -=temp;
        if(indXX >=0 && indXX < gridNum && indYY >=0 && indYY < gridNum && (indXX != indX || indYY != indY)) {
            int ind2 = indXX * gridSize * meshX + indYY * gridSize;
            ofVec2f gradient2 = grad[indXX][indYY];
            ofVec3f slope1 = slope[ind1];
            ofVec3f slope2 = slope[ind2];
            float z1 = mainMesh.getVertex(ind1).z;
            float z2 = mainMesh.getVertex(ind2).z;
            if(gradient1.dot(gradient2) > 0) {
                if (z1 > z2) { // from high to low
                    visitedList[indX][indY].z = ind2; // current node has child ind2
                    visitedList[indXX][indYY].y = ind1; // current node has parent ind1
                    findPath(indXX,indYY);
                } else if(z1 < z2){ // from low to high
                    visitedList[indX][indY].y = ind2; // current node has parent ind2
                    visitedList[indXX][indYY].z = ind1; // current node has child ind1
                    findPath(indXX,indYY);
                }
            }
        }
    } else return;
}

//--------------------------------------------------------------
void ofApp::updateMeshInfo(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &N, float* normals) {
    for(int i = 0 ; i < mainMesh.getNumVertices(); i ++) {
        ofVec3f v = mainMesh.getVertex(i);
        V.row(i) << v.x,v.y,v.z;
    }
    
    for(int i = 0, j = 0 ; i < mainMesh.getNumIndices(); i += 3, j ++) {
        F.row(j) << mainMesh.getIndex(i), mainMesh.getIndex(i + 1), mainMesh.getIndex(i + 2);
    }
    
    igl::per_vertex_normals(V, F, N);
    for(int i = 0, j = 0 ; i < mainMesh.getNumVertices(); i ++, j += 3) {
        for(int k = 0; k < 3; k ++) {
            normals[j + k] = (float)N.row(i)[k];
        }
    }
    //    std::cout<<"update mesh info!!\n";
    //    std::cout<< "vertices:\n";
    //    std::cout<<vertices;
    //    std::cout<< "\n";
    //    std::cout<< "faces:\n";
    //    std::cout<<faces;
    //    std::cout<< "\n";
}

//--------------------------------------------------------------
void ofApp::constructGradField(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &gradU, Eigen::MatrixXd &gradV, Eigen::VectorXd &curU, Eigen::VectorXd &curV) {
    std::cout<<"construct grad field!!\n";
    igl::principal_curvature(V, F, gradU, gradV, curU, curV);
    for(int i = 0; i < mainMesh.getNumVertices()/3; i ++ ) {
        curvatures[i] = (float)MAX(abs(curU[i]), abs(curV[i]));
    }
}

//--------------------------------------------------------------
void ofApp::constructWindField(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &windU, Eigen::MatrixXd &windV) {
    std::cout<<"construct wind field!!\n";
    // get the harmonic weight
    Eigen::VectorXi b;
    igl::boundary_loop(F, b);
    Eigen::MatrixXd bc;
    igl::map_vertices_to_circle(V,b,bc);
    Eigen::MatrixXd uv;
    igl::harmonic(V, F, b, bc, 1, uv);
    Eigen::SparseMatrix<double> G;
    igl::grad(V,F,G);
    Eigen::MatrixXd grad;
    grad = G * uv;
    
    float unit = grad.rows()/3;
    Eigen::MatrixXd gradX_U = grad.block(0,0,unit,1);
    Eigen::MatrixXd gradX_V = grad.block(0,1,unit,1);
    Eigen::MatrixXd gradY_U = grad.block(unit,0,unit,1);
    Eigen::MatrixXd gradY_V = grad.block(unit,1,unit,1);
    Eigen::MatrixXd gradZ_U = grad.block(2*unit,0,unit,1);
    Eigen::MatrixXd gradZ_V = grad.block(2*unit,1,unit,1);
    
    windU.resize(unit,3);
    windU << gradX_U, gradY_U, gradZ_U;
    windV.resize(unit,3);
    windV << gradX_V, gradY_V, gradZ_V;
}

//--------------------------------------------------------------
void ofApp::renderVectorField(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &grad, Eigen::VectorXd &cur, Eigen::MatrixXd &wind, ofVec3f* slope, float vectorFieldMode) {
    if(vectorFieldMode == 0) { // draw slope field
        for (int i = 0; i < mainMesh.getNumVertices(); i ++) {
            //            ofDrawLine(V.row(i)[0], V.row(i)[1], V.row(i)[2],V.row(i)[0] + slope[i].x,V.row(i)[1] + slope[i].y,V.row(i)[2] + slope[i].z);
            ofVec3f arrowTailPoint(V.row(i)[0], V.row(i)[1], V.row(i)[2]);
            ofVec3f arrowHeadPoint(V.row(i)[0] + slope[i].x,V.row(i)[1] + slope[i].y,V.row(i)[2] + slope[i].z);
            ofDrawArrow(arrowTailPoint, arrowHeadPoint, 0.1);
            ofSetColor(255,0,0);
            // draw normal field
            //            ofDrawLine(V.row(i)[0], V.row(i)[1], V.row(i)[2],V.row(i)[0] + N.row(i)[0],V.row(i)[1] + N.row(i)[1],V.row(i)[2] + N.row(i)[2]);
            //            ofSetColor(0,0,255);
            // draw gravity field
            //            ofDrawLine(V.row(i)[0], V.row(i)[1], V.row(i)[2],V.row(i)[0] + gravity.x,V.row(i)[1] + gravity.y,V.row(i)[2] + gravity.z);
        }
        //        int scale = 3;
        //        for (int i = 0; i < grad.rows(); i ++) {
        // draw gradiant field
        //            ofDrawLine(V.row(i)[0], V.row(i)[1], V.row(i)[2], V.row(i)[0] + scale * cur[i] * grad.row(i)[0], V.row(i)[1] + scale * cur[i] * grad.row(i)[1], V.row(i)[2] + scale * cur[i] * grad.row(i)[2]);
        //        }
    } else { // draw wind field created using harmonic weight
        double avg = 10 * igl::avg_edge_length(V, F);
        for (int i = 0; i < wind.rows(); i ++) {
            ofDrawLine(BC.row(i)[0], BC.row(i)[1], BC.row(i)[2], BC.row(i)[0] + avg * wind.row(i)[0], BC.row(i)[1] + avg * wind.row(i)[1], BC.row(i)[2] + avg * wind.row(i)[2]);
        }
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch(key) {
        case '1':
            u_mode = 0;
            break;
        case '2':
            u_mode = 1;
            break;
        case '3':
            u_mode = 2;
            break;
        case '4':
            u_mode = 3;
            break;
        case 'f':
            ofToggleFullscreen();
            break;
        case 'r':
            realTime = !realTime;
            break;
        case 'a':
            season = (season + 1) % 3;
            break;
        case 'u':
            updateMesh = true;
            break;
        case 's':
            gui.saveToFile("settings.xml");
            break;
        case 'l':
            gui.loadFromFile("settings.xml");
            break;
        case 'h':
            bHide = !bHide;
            break;
        case 't':
            displayTopView = !displayTopView;
            break;
        case 'c':
            displayRain = !displayRain;
            break;
        case 'd':
            waterMode = (waterMode + 1) % 5;
            break;
            
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    
}
