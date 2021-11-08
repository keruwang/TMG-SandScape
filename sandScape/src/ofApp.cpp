#include "ofApp.h"
#define DISPLAY_WIDTH 1680
#define DISPLAY_HEIGHT 1050

float u_mode = 0;
int season = 0;
int waterMode = 0;
float snowHeight = 0;
float meshX = SIZE;
float meshY = SIZE;
float meshZ = SIZE/5;
float cloudAmount = 5;
float specular = 1.;
float shadowRate = 1.;
float lineDarkness = 0.3;
float mAlpha = 1.;
float wAlpha = 0.;
float mBrightness = 1.;
float mSaturation = 1.;
//--------------------------------------------------------------
void ofApp::setup(){
    // initsialize the standard vectors
    up.set(0,-1);
    down.set(0,1);
    left.set(-1,0);
    right.set(1,0);
    // window display setup
    ofSetWindowShape(5000, 1000);
    ofSetWindowPosition(-1, -1);
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
    cloudShader.load("cloudShader");
    rainShader.load("rainShader");
    
    // set the initial values to use for our perlinNoise
    perlinRange = 1.5;
    perlinHeight = meshZ;
    mainCam.setPosition(0, 0, 80); // set initial position for oureasyCam 3D viewer
    mainCam.setOrientation(ofVec3f(0,0,0));
    
    // create the mesh
    for (int y = 0; y < meshY; y ++){
        for (int x = 0; x < meshX; x ++){
            mainMesh.addVertex(ofPoint(x - meshX / 2,y - meshY / 2,0));
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
    indList = new int[mainMesh.getNumVertices()];
    //    curvatures = new float[mainMesh.getNumIndices() / 3];
    slop = new ofVec3f[mainMesh.getNumVertices()];
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
        slop[i].set(gravity - (temp * gravity)[2] * temp);
        directions[3 * i] = slop[i].x;
        directions[3 * i + 1] = slop[i].y;
        directions[3 * i + 2] = slop[i].z;
    }
    gridNum = meshX/gridSize;
    for (int i = 0; i < gridNum; i ++) {
        for (int j = 0; j < gridNum; j ++) {
            int index = i * gridSize * meshX + j * gridSize;
            grad[i][j].set(slop[index].x,slop[index].y);
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
    
    gui.add(blur.setup("blur", blur_, 0, 25));
    gui.add(scale.setup("scale", scale_, 0, 5));
    gui.add(topViewX.setup("topViewX", topViewX_, 0, 5000));
    gui.add(topViewY.setup("topViewY", topViewY_, 0, 1000));
    
    gui.add(smoothRound.setup("smoothRound", smoothRound_, 0, 25));
    gui.add(smoothWeight.setup("smoothWeight", smoothWeight_, 0, 10));
    gui.add(cloud.setup("cloud", cloud_, 0.5, 5));
    gui.add(water.setup("water", water_, 0, 1));
    gui.add(contourLineDarkness.setup("contourLineDarkness", contourLineDarkness_, 0, 1));
    gui.add(shadowContrast.setup("shadowContrast", specular_, 0, 100));
    gui.add(shadowRatio.setup("shadowRatio", shadow_, 0, 1));
    gui.add(waterAlpha.setup("waterAlpha", waterAlpha_, 0, 1));
    gui.add(meshAlpha.setup("meshAlpha", meshAlpha_, 0, 1));
    gui.add(meshBrightness.setup("meshBrightness", meshBrightness_, 0, 1));
    gui.add(meshSaturation.setup("meshSaturation", meshSaturation_, 0, 1));
    gui.add(grid.setup("gridSize", gridSize_, 1, 4));
    gui.add(dotSize.setup("dotSize", dotSize_, 1, 5));
    gui.add(lineWidth.setup("lineWidth", lineWidth_, 4, 14));
    
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
    fbo_grad.allocate(meshX * 10,meshY * 10);
    fbo_grad.begin();
    ofClear(0,0,0,0);
    fbo_grad.end();
    
    sun.setRadius(5);
    // set up for the 3D rain particle system
    //    world.setup();
    //    world.setCamera(&mainCam);
    //
    //    bulletMesh = shared_ptr< ofxBulletTriMeshShape >( new ofxBulletTriMeshShape() );
    //    bulletMesh->create( world.world, mainMesh, ofVec3f(0,0,0), 0.f, ofVec3f(-10000, -10000, -10000), ofVec3f(10000,10000,10000) );
    //    bulletMesh->add();
    //    bulletMesh->enableKinematic();
    //    bulletMesh->setActivationState( DISABLE_DEACTIVATION );
    
    //    ofSetFrameRate(2);
}

//--------------------------------------------------------------
void ofApp::update(){
    // cout << ofGetFrameRate() << endl;
    
//        if ((updateMesh || realTime)) {
    if (false) {
        kinect.update();
        
        if (kinect.isFrameNew()) {
            grayImage.setFromPixels(kinect.getDepthPixels());
            // crop out the valid part
            unsigned char* pre_pix = grayImage.getPixels().getData();
            unsigned char* cropped;
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
            
            // add effects to smooth signal and reduce noise
            grayImageSmall.blur(3);
            //                grayImageSmall.blurGaussian(blur);
            //                grayImageSmall.blurHeavily();
            //                grayImageSmall.dilate();
            
            //                for (int i = 0; i < nearThreshold - farThreshold; i ++) {
            //                    grayThreshNear = grayImageSmall;
            //                    grayThreshFar = grayImageSmall;
            //                    grayThreshNear.threshold(farThreshold + 1 + 1, true);
            //                    grayThreshFar.threshold(farThreshold + 1);
            //                    cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImageSmall.getCvImage(), NULL);
            //
            //                    cvAddWeighted(grayImage_avg.getCvImage(), .51, grayImageSmall.getCvImage(), .49, 0.0, grayImage_avg.getCvImage());
            //                    cvSmooth(grayImage_avg.getCvImage(), grayImage_avg.getCvImage(), CV_GAUSSIAN);
            //
            ////                    contourFinder.findContours(grayImage_avg, 200, (340*240)/3, 10, true);
            //                }
            //
            //
            //                cvAnd(grayImageSmall.getCvImage(), grayOverall.getCvImage(), grayOverall.getCvImage(), NULL);
            
            unsigned char * pix = grayImageSmall.getPixels().getData();
            
            // draw image
            int diffThreshold = nearThreshold - farThreshold;
            
            // iterate through pixel data
            for (int w = 0; w < grayImageSmall.getWidth(); w++) {
                for (int h = 0; h < grayImageSmall.getHeight(); h++) {
                    // average previous pixels with current reading
                    int index = h * grayImageSmall.getWidth() + w;
                    double currentDepth = pix[index];
                    double prevDepth = prevPix[index];
                    prevPix[index] = prevDepth* .9 + currentDepth * .1;
                    double  depth = prevPix[index];
                    //                        double depth = currentDepth;
                    
                    
                    
                    // boolean operations if inside sandbox boundaries and depth boundaries
                    //                        bool isInBoundary = w < xRightBoundary/10 && w > xLeftBoundary/10 && h < yBottomBoundary/10 && h > yTopBoundary/10;
                    bool isInBoundary = true;
                    //                bool isInDepthBoundary = true;
                    bool isInDepthBoundary = currentDepth < nearThreshold && currentDepth > farThreshold;
                    
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
                        //                            cout << tmpVec.z << " ";
                        //                            if (ofGetMouseX() == w && ofGetMouseY() == h) {
                        //                                cout << tmpVec.z << endl;
                        //                            }
                        mainMesh.setVertex((h)*(grayImageSmall.width)+(grayImageSmall.getWidth() - w), tmpVec);
                        
                        //cout << "TEST" << endl;
                    } else {
                        
                        ofVec3f tmpVec = mainMesh.getVertex(h * grayImageSmall.width + (grayImageSmall.getWidth() - w));
                        //                            tmpVec.z = -0.1;
                        
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
        // update the reconstructed mesh
        updateMeshInfo(V, F, N, normals);
        for (int i = 0; i < mainMesh.getNumVertices(); i ++) {
            ofVec3f temp;
            temp.set(N.row(i)[0],N.row(i)[1],N.row(i)[2]);
            slop[i].set(gravity - (temp * gravity)[2] * temp);
            directions[3 * i] = slop[i].x;
            directions[3 * i + 1] = slop[i].y;
            directions[3 * i + 2] = slop[i].z;
        }
        // construct the gradient field for waterdrainage display
        for (int i = 0; i < gridNum; i ++) {
            for (int j = 0; j < gridNum; j ++) {
                int index = i * gridSize * meshX + j * gridSize;
                grad[i][j].set(slop[index].x,slop[index].y);
                if(grad[i][j].length() > 0.)
                    grad[i][j] = grad[i][j].getNormalized();
                else grad[i][j].set(ofNoise(10 * i),ofNoise(10 * j));
            }
        }
        //        bulletMesh->updateMesh( world.world, mainMesh );
        //        updateMesh = false;
    }
    
    cloudAmount = cloud;
    lineDarkness = contourLineDarkness;
    specular = shadowContrast;
    shadowRate = shadowRatio;
    mAlpha = meshAlpha;
    wAlpha = waterAlpha;
    mBrightness = meshBrightness;
    mSaturation = meshSaturation;
    gridSizez = grid;
    
    
    //------------ draw water drainage as jittering lines with particles -----------------------------------
    //    fbo_grad.begin();
    //    ofClear(0,0,0,0);
    //    fbo_grad.end();
    //    particles.clear();
    //    for(int i = 0; i < meshX; i+= 5) {
    //        for (int j = 0; j < meshY; j += 5) {
    //            int ind = i * meshX + j;
    //            Particle newParticle;
    //            ofVec3f pos = mainMesh.getVertex(ind);
    //            //            if(pos.z > 0.1 * meshZ) {
    //            ofVec3f norm(N.row(ind)[0],N.row(ind)[1],N.row(ind)[2]);
    //            float dot = abs(norm.dot(gravity));
    //            //                if(dot > 0.85 && dot < 0.99) {
    //            ofVec3f white(255, 255, 255);
    //            newParticle.setup(pos, white);
    //            particles.push_back(newParticle);
    //        }
    //    }
    //
    //    fbo_grad.begin();
    //    ofPushStyle();
    //    for (int i = 0; i < particles.size(); i ++) {
    //        particles[i].updateDuration(0.);
    //        if (particles[i].isDead()) {
    //            particles.erase(particles.begin() + i);
    //        }
    //        for(int k = 0; k < 5; k ++) {
    //            ofVec3f moveDir;
    //            ofVec2f pxy(particles[i].pos.x,particles[i].pos.y);
    //            if(pxy.x > -25 && pxy.x < 25 && pxy.y > -25 && pxy.y < 25 ) {
    //                int indP = (int)(round(particles[i].pos.x) + 25) * meshX + (int)(round(particles[i].pos.y)+25);
    //                moveDir.set(3 * slop[indP]);
    //                particles[i].move(moveDir);
    //                ofVec2f cxy(particles[i].pos.x,particles[i].pos.y);
    //                if(pxy.x > -25 && pxy.x < 25 && pxy.y > -25 && pxy.y < 25 ) {
    //                    int indC = (int)(round(particles[i].pos.x) + 25) * meshX + (int)(round(particles[i].pos.y)+25);
    //                    particles[i].pos = mainMesh.getVertex(indC);
    //                ofSetLineWidth(3);
    //                ofSetColor(k * 15, 0, 0);
    //                ofDrawLine(250 + pxy.x * 10, 250 + pxy.y * 10, 250 + cxy.x * 10, 250 + cxy.y * 10);
    //                }
    //                if(pxy.distance(cxy) < 0.01) {
    //                    particles.erase(particles.begin() + i);
    //                     break;
    //                }
    //                //                particles[i].draw(1.);
    //            } else {
    //                particles.erase(particles.begin() + i);
    //                break;
    //            }
    //        }
    //    }
    //    int num = particles.size();
    //    for (int i = 0; i < num; i ++) {
    //        Particle newParticle;
    //        ofVec3f white(255, 255, 255);
    //        ofVec3f pos(particles[i].pos.x + 2,particles[i].pos.y + 2,particles[i].pos.z);
    //        newParticle.setup(pos, white);
    //        particles.push_back(newParticle);
    //        ofDrawLine(250 + pos.x * 10, 250 + pos.y * 10, 250 + particles[i].pos.x * 10, 250 + particles[i].pos.y * 10);
    //    }
    //    for (int i = 0; i < particles.size(); i ++) {
    //        particles[i].updateDuration(0.);
    //        if (particles[i].isDead()) {
    //            particles.erase(particles.begin() + i);
    //        }
    //        for(int k = 0; k < 10; k ++) {
    //            ofVec3f moveDir;
    //            ofVec2f pxy(particles[i].pos.x,particles[i].pos.y);
    //            if(pxy.x > -25 && pxy.x < 25 && pxy.y > -25 && pxy.y < 25 ) {
    //                int indP = (int)(round(particles[i].pos.x) + 25) * meshX + (int)(round(particles[i].pos.y)+25);
    //                moveDir.set(3 * slop[indP]);
    //                particles[i].move(moveDir);
    //                ofVec2f cxy(particles[i].pos.x,particles[i].pos.y);
    //                if(pxy.x > -25 && pxy.x < 25 && pxy.y > -25 && pxy.y < 25 ) {
    //                    int indC = (int)(round(particles[i].pos.x) + 25) * meshX + (int)(round(particles[i].pos.y)+25);
    //                    particles[i].pos = mainMesh.getVertex(indC);
    //                ofSetLineWidth(3);
    //                ofSetColor(75 + k * 15, 0, 0);
    //                ofDrawLine(250 + pxy.x * 10, 250 + pxy.y * 10, 250 + cxy.x * 10, 250 + cxy.y * 10);
    //                }
    //                if(pxy.distance(cxy) < 0.05) {
    //                    particles.erase(particles.begin() + i);
    //                     break;
    //                }
    //                //                particles[i].draw(1.);
    //            } else {
    //                particles.erase(particles.begin() + i);
    //                break;
    //            }
    //        }
    //    }
    //    ofPopStyle();
    //    fbo_grad.end();
    
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
    //                        moveDir.set(slop[j]);
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
    //                    moveDir.set(slop[j]);
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
    sun.setPosition(40 * cos(ofGetElapsedTimef()),0,40 * sin(ofGetElapsedTimef()));
    
    //---------------------------------- display 3D rain particle system -----------------------------------
    //    if(displayRain) {
    //        if( bulletSpheres.size() < 200 ) {
    //            shared_ptr< ofxBulletSphere > sphere( new ofxBulletSphere() );
    //            float trad = fabs(sin( ofGetElapsedTimef() ) * 5);
    //            sphere->create( world.world, ofVec3f( ofRandom(-12.5,12.5), ofRandom(-12.5,12.5), 24.5), 10., 0.15 );
    //            sphere->add();
    //            bulletSpheres.push_back( sphere );
    //        }
    //
    //        for( int i = 0; i < bulletSpheres.size(); i++ ) {
    //            ofVec3f spos = bulletSpheres[i]->getPosition();
    //            if( spos.z < 0 ) {
    //                bulletSpheres.erase( bulletSpheres.begin() + i );
    //                break;
    //            }
    //        }
    //        world.update( ofGetLastFrameTime(), 12 );
    //    }
    
    //---------------------------------- draw water drainage vectors based on classification -----------------------------------
   

        fbo_grad.begin();
        ofPushStyle();
        ofSetColor(0, 0, 0);
        ofDrawRectangle(0, 0, meshX * 10, meshY * 10);
        // don't find path, just draw the vectors
        //        int time = (int)(ofGetFrameNum()/20);
        //        for (int i = 0; i < gridNum; i ++) {
        //            for (int j = 0; j < gridNum; j ++) {
        //                ofVec2f gradient = grad[i][j];
        //                int ind = i * gridSize * meshX + j * gridSize;
        //                float x = 250 + mainMesh.getVertex(ind).x * 10;
        //                float y = 250 + mainMesh.getVertex(ind).y * 10;
        //                float z = meshZ - mainMesh.getVertex(ind).z;
        //                int xx = 0;
        //                int yy = 0;
        //                if (gradient.dot(up) >  water || gradient.dot(down) > water) {
        //                    yy = 1;
        //                }
        //                if (gradient.dot(right) >  water || gradient.dot(left) > water) {
        //                    xx = 1;
        //                }
        //                ofPushStyle();
        //                ofSetColor(0,0,200);
        //                float temp = time - 10 * (int)(time/10);
        //                if( temp > z - .5 && temp < z + .5) ofSetColor(100,0,0);
        //                if(z > 7) ofSetColor(100,0,0);
        //                ofSetLineWidth(6);
        //                ofDrawLine(x, y, x + xx * 5, y + yy * 5);
        //                ofPopStyle();
        //            }
        //        }
        // find path based on the vectors
//    if(ofGetFrameNum() % 180 == 0) {
    int temp = gridSizez;
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
//    }
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
//                        ofSetLineWidth(6);
//                        int ind = min(travelStep,10);
                        float stiff =  mainMesh.getVertex(ind1).z -  mainMesh.getVertex(ind2).z;
                        int ind = min((int)(stiff * 5 / (gridSize * gridSize)) ,10);
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
                            ofDrawCircle(x + 1.8 * (-1 + cloudAmount * ofNoise(y/10)),y + cloudAmount * (-1 + 4 * ofNoise(x/10)), d/(dn - 1));
                        }
                        float time = ofClamp(0.5 * ind, 1, 3) * ofGetElapsedTimef();
                        float ratio = (time - floor(time));
                        if((travelStep + (int)randOffset[indOff]) % (3 + (int)cloud) == (int)time % (3 + (int)cloud)) {
                            ofPushStyle();
                            if(waterMode == 1 || waterMode > 1) ofSetColor(0, 100, 255);
                            float x = x1 + ratio * dx;
                            float y = y1 + ratio * dy;
                            ofDrawCircle(x + 1.8 * (-1 + cloudAmount * ofNoise(y/10)),y + 1.8 * (-1 + cloudAmount * ofNoise(x/10)), dotSize);
                            float xb = x1 + (ratio-0.15) * dx;
                            float yb = y1 + (ratio-0.15) * dy;
                            float xa = x1 + (ratio+0.15) * dx;
                            float ya = y1 + (ratio+0.15) * dy;
                            ofDrawCircle(xb + 1.5 * (-1 + cloudAmount * ofNoise(yb/10)),yb + 1.5 * (-1 + cloudAmount * ofNoise(xb/10)), dotSize - 0.5);
                            ofDrawCircle(xa + 1.5 * (-1 + cloudAmount * ofNoise(ya/10)),ya + 1.5 * (-1 + cloudAmount * ofNoise(xa/10)), dotSize - 0.5);
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
        for (int x = 0; x < 1; x ++) {
            for (int i = 0; i < gridNum; i +=temp) {
                for (int j = 0; j < gridNum; j +=temp) {
        //            cout<<visitedList[i][j]<<endl;
                    int indOff = i * meshX * gridSize + j * gridSize;
                    int ind2p = visitedList[i][j].z;
                    int ind2Xp = floor(ind2p * gridSize / meshX);
                    int ind2Yp = ind2p - meshX * ind2Xp;
                    if(visitedList[i][j].x == 1 && visitedList[ind2Xp][ind2Yp].x != 1) {  // it is the root: parent = -1
                        visitedList[i][j].x = 0;
                        ofVec3f currentNode = visitedList[i][j];
                        int travelStep = 0;
                        int ind1 = i * meshX * gridSize + j * gridSize;
                        int ind2;
                        while(currentNode.z != -1) { // it still has child
                            ind2 = currentNode.z;
                            int ind2X = floor(ind2 * gridSize / meshX);
                            int ind2Y = ind2 - meshX * ind2X;
                            float x1 = 10 * (meshX/2) + mainMesh.getVertex(ind1).x * 10;
                            float y1 = 10 * (meshX/2) + mainMesh.getVertex(ind1).y * 10;
                            float x2 = 10 * (meshY/2) + mainMesh.getVertex(ind2).x * 10;
                            float y2 = 10 * (meshY/2) + mainMesh.getVertex(ind2).y * 10;
    //                        ofSetLineWidth(6);
//                            int ind = min(travelStep,10);
                            float stiff =  mainMesh.getVertex(ind1).z -  mainMesh.getVertex(ind2).z;
                            int ind = min((int)(stiff * 5 / (gridSize * gridSize)) ,10);
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
                                ofDrawCircle(x + 1.8 * (-1 + cloudAmount * ofNoise(y/dn)),y + 1.8 * (-1 + cloudAmount * ofNoise(x/dn)), d/dn);
                            }
                            
    //                        if((travelStep + (int)randOffset[indOff]) % 8 == (int)time % 8) {
    //                            ofPushStyle();
    //                            ofDrawCircle(x1 + ratio * dx, y1 + ratio * dy, 2);
    //                            ofPopStyle();
    //                        }
                           
                            currentNode = visitedList[ind2X][ind2Y];
                            visitedList[ind2X][ind2Y].x = 0;
                            ind1 = ind2;
                            travelStep ++;
                        }
                    }
                }
            }
        }
      
        
        ofPopStyle();
        fbo_grad.end();

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
    
    fbo_topView.begin();
    ofPushStyle();
    ofSetColor(0,0,0);
    ofDrawRectangle(0, 0, kinect.width, kinect.height);
    ofPopStyle();
    ofTranslate(10*meshX/2, 10*meshY/2);
    ofScale(-10,10,1);
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
    mainCam.begin();
    shader.begin();
    ofEnableDepthTest();
    mainMesh.drawFaces();
    shader.end();
    //        renderVectorField(V, F, gradV, curV, windU, slop, vectorFieldMode);
    if(u_mode == 3){
        sunShader.begin();
        sun.draw();
        sunShader.end();
    }
    ofDisableDepthTest();
    if(u_mode == 1) {
        cloudShader.begin();
        ofDrawRectangle(-25, -25, 25, 50, 50);
        cloudShader.end();
        if(displayRain){
            rainShader.begin();
            for( int i = 0; i < bulletSpheres.size(); i++ ) {
                bulletSpheres[i]->draw();
            }
            rainShader.end();
            
        }
    }
    
    mainCam.end();
    
    if(!bHide){
        gui.draw();
    }
    //    croppedImg.draw(ofGetMouseX(),ofGetMouseY(),cropped_w,cropped_h);
}

//--------------------------------------------------------------
void ofApp::exit() {
    kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();
}

//--------------------------------------------------------------
//void ofApp::findPath(int indX, int indY) {
//    // visitedList vec3
//    // digit 1: visited or not; digit 2: parent, digit 3: child
//    int time = (int)(ofGetFrameNum()/80);
//    if(visitedList[indX][indY].x == 0) {
//        int ind1 = indX * gridSize * meshX + indY * gridSize;
//        visitedList[indX][indY].x ++; // marked as visited
//        visitedList[indX][indY].y ++; // give it a length
//        ofVec2f gradient1 = grad[indX][indY];
//        int indXX = indX;
//        int indYY = indY;
//        if(gradient1.dot(up) > water) indYY --;
//        else if(gradient1.dot(down) > water) indYY ++;
//        if(gradient1.dot(right) > water) indXX ++;
//        else if(gradient1.dot(left) > water) indXX --;
//        if(indXX >=0 && indXX < gridNum && indYY >=0 && indYY < gridNum && (indXX != indX || indYY != indY)) {
//            int ind2 = indXX * gridSize * meshX + indYY * gridSize;
//            ofVec2f gradient2 = grad[indXX][indYY];
//            ofVec3f slop1 = slop[ind1];
//            ofVec3f slop2 = slop[ind2];
//            float z1 = mainMesh.getVertex(ind1).z;
//            float z2 = mainMesh.getVertex(ind2).z;
//            if(gradient1.dot(gradient2) > 0) { // && visitedList[indX][indY].y < randOffset[ind1] && visitedList[indXX][indYY].y < randOffset[ind1]
//                if (z1 > z2) {
//                    float x1 = 250 + mainMesh.getVertex(ind1).x * 10 + gridSize;
//                    float y1 = 250 + mainMesh.getVertex(ind1).y * 10 + gridSize;
//                    float x2 = 250 + mainMesh.getVertex(ind2).x * 10 + gridSize;
//                    float y2 = 250 + mainMesh.getVertex(ind2).y * 10 + gridSize;
//                    ofPushStyle();
//                    visitedList[indXX][indYY].y += visitedList[indX][indY].y; // ((int)(1.5 * (z1 - z2))) *
//                    int ind = (int)(visitedList[indXX][indYY].y-1) % 8;
//                    ofSetColor(ind * 50,0,0);
////                    ofSetColor(waterColor[ind][0],waterColor[ind][1],waterColor[ind][2]);
//                    //                    if (time % 8 == ind) ofSetColor(0,0,200);
//                    ofSetLineWidth(10);
//                    ofDrawLine(x1, y1, x2, y2);
//                    ofPopStyle();
//                    findPath(indXX,indYY);
//                }
//                else if(z2 > z1) {
//                    float x1 = 250 + mainMesh.getVertex(ind1).x * 10 + gridSize;
//                    float y1 = 250 + mainMesh.getVertex(ind1).y * 10 + gridSize;
//                    float x2 = 250 + mainMesh.getVertex(ind2).x * 10 + gridSize;
//                    float y2 = 250 + mainMesh.getVertex(ind2).y * 10 + gridSize;
//                    ofPushStyle();
//                    //                    visitedList[indX][indY].y += visitedList[indXX][indYY].y; // ((int)(1.5 * (z2 - z1))) *
//                    visitedList[indXX][indYY].y += visitedList[indX][indY].y; // ((int)(1.5 * (z1 - z2))) *
//                    //                    ofSetColor(80 + 40 * abs(z1 - z2),120 +  40 * abs(z1 - z2),250 - 30 * abs(z1 - z2));
//                    int ind = (int)(visitedList[indXX][indYY].y - 1) % 8;
//                    ofSetColor(0,0,ind * 50);
////                    ofSetColor(waterColor[ind][0],waterColor[ind][1],1.5 * waterColor[ind][2]);
//                    ofSetLineWidth(10);
//                    ofDrawLine(x1, y1, x2, y2);
//                    ofPopStyle();
//                    findPath(indXX,indYY);
//
//                }
////                else {
////                    float x1 = 250 + mainMesh.getVertex(ind1).x * 10 + gridSize;
////                    float y1 = 250 + mainMesh.getVertex(ind1).y * 10 + gridSize;
////                    float x2 = 250 + mainMesh.getVertex(ind2).x * 10 + gridSize;
////                    float y2 = 250 + mainMesh.getVertex(ind2).y * 10 + gridSize;
////                    ofPushStyle();
////                    visitedList[indX][indY].y += visitedList[indXX][indYY].y; // ((int)(1.5 * (z2 - z1))) *
////                    //                    ofSetColor(80 + 40 * abs(z1 - z2),120 +  40 * abs(z1 - z2),250 - 30 * abs(z1 - z2));
////                    int ind = (int)(max(visitedList[indX][indY].y,visitedList[indXX][indYY].y)-1) % 8;
////                    ofSetColor(0.9 * waterColor[ind][0],0.9 * waterColor[ind][1],0.9 * waterColor[ind][2]);
////                    ofSetLineWidth(10);
////                    ofDrawLine(x1, y1, x2, y2);
////                    ofPopStyle();
////                    findPath(indXX,indYY);
////
////                }
//            }
//        }
//    } else return;
//}

//--------------------- do 8 classification -----------------------------------------
void ofApp::findPath(int indX, int indY) {
    int temp = gridSizez;
    // visitedList vec3
    // digit 1: visited or not; digit 2: parent, digit 3: child
    if(visitedList[indX][indY].x == 0) {
        int ind1 = indX * gridSize * meshX + indY * gridSize;
        visitedList[indX][indY].x ++; // marked as visited
        ofVec2f gradient1 = grad[indX][indY];
        int indXX = indX;
        int indYY = indY;
        if(gradient1.dot(up) > water) indYY -=temp;
        else if(gradient1.dot(down) > water) indYY +=temp;
        if(gradient1.dot(right) > water) indXX +=temp;
        else if(gradient1.dot(left) > water) indXX -=temp;
        if(indXX >=0 && indXX < gridNum && indYY >=0 && indYY < gridNum && (indXX != indX || indYY != indY)) {
            int ind2 = indXX * gridSize * meshX + indYY * gridSize;
            ofVec2f gradient2 = grad[indXX][indYY];
            ofVec3f slop1 = slop[ind1];
            ofVec3f slop2 = slop[ind2];
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

//--------------------- do 16 classification -----------------------------------------
//void ofApp::findPath(int indX, int indY) {
//    int temp = gridSizez;
//    // visitedList vec3
//    // digit 1: visited or not; digit 2: parent, digit 3: child
//    if(visitedList[indX][indY].x == 0) {
//        int ind1 = indX * gridSize * meshX + indY * gridSize;
//        visitedList[indX][indY].x ++; // marked as visited
//        ofVec2f gradient1 = grad[indX][indY];
//        int indXX = indX;
//        int indYY = indY;
//        if(gradient1.dot(up) > water) indYY -=temp;
//        else if(gradient1.dot(down) > water) indYY +=temp;
//        if(gradient1.dot(right) > water) indXX +=temp;
//        else if(gradient1.dot(left) > water) indXX -=temp;
//        if(indXX >=0 && indXX < gridNum && indYY >=0 && indYY < gridNum && (indXX != indX || indYY != indY)) {
//            int ind2 = indXX * gridSize * meshX + indYY * gridSize;
//            ofVec2f gradient2 = grad[indXX][indYY];
//            ofVec3f slop1 = slop[ind1];
//            ofVec3f slop2 = slop[ind2];
//            float z1 = mainMesh.getVertex(ind1).z;
//            float z2 = mainMesh.getVertex(ind2).z;
//            if(gradient1.dot(gradient2) > 0) {
//                if (z1 > z2) { // from high to low
//                    visitedList[indX][indY].z = ind2; // current node has child ind2
//                    visitedList[indXX][indYY].y = ind1; // current node has parent ind1
//                    findPath(indXX,indYY);
//                } else if(z1 < z2){ // from low to high
//                    visitedList[indX][indY].y = ind2; // current node has parent ind2
//                    visitedList[indXX][indYY].z = ind1; // current node has child ind1
//                    findPath(indXX,indYY);
//                }
//            }
//        }
//    } else return;
//}

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
    //    std::cout<< "vertcies:\n";
    //    std::cout<<vertcies;
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
void ofApp::renderVectorField(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &grad, Eigen::VectorXd &cur, Eigen::MatrixXd &wind, ofVec3f* slop, float vectorFieldMode) {
    if(vectorFieldMode == 0) {
        for (int i = 0; i < mainMesh.getNumVertices(); i ++) {
            //            ofDrawLine(V.row(i)[0], V.row(i)[1], V.row(i)[2],V.row(i)[0] + slop[i].x,V.row(i)[1] + slop[i].y,V.row(i)[2] + slop[i].z);
            ofVec3f arrowTailPoint(V.row(i)[0], V.row(i)[1], V.row(i)[2]);
            ofVec3f arrowHeadPoint(V.row(i)[0] + slop[i].x,V.row(i)[1] + slop[i].y,V.row(i)[2] + slop[i].z);
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
    } else {
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
