#pragma once

#include "ofMain.h"
#include "ofxShader.h"
#include "ofxLibigl.h"
#include "ofxEigen.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxGui.h"
#include "Particle.h"
#include "WaterDrainage.h"

#define SIZE 80

extern float u_mode, snowHeight, meshX, meshY, meshZ, cloudAmount, lineDarkness, specular, shadowRate, wAlpha, mAlpha, mBrightness, mSaturation;
extern int season, waterMode;

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    void exit();
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    void updateMeshInfo(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &N, float* normals);
    void constructGradField(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &gradU, Eigen::MatrixXd &gradV, Eigen::VectorXd &curU, Eigen::VectorXd &curV);
    void renderVectorField(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &grad, Eigen::VectorXd &cur, Eigen::MatrixXd &wind, ofVec3f* slop, float drawMode);
    void constructWindField(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &windU, Eigen::MatrixXd &windV);
    void findPath(int indX, int indY);
    
    // shader programs
    ofxShader shader;
    ofxShader sunShader;
    ofxShader cloudShader;
    ofVboMesh mainMesh;

    ofEasyCam mainCam;
    float perlinRange, perlinHeight; // for constructing the default noise mesh
    bool displayParticles = false;
    bool renderField = false;
    bool drawParticle = false;
    bool updateMesh = false;
    bool realTime = true;
    bool displayTopView = false;
    bool displayRain = false;
    Eigen::MatrixXd V; // vertex
    Eigen::MatrixXi F; // faces
    Eigen::MatrixXd N; // normals
    float* normals; // unpack the normals to pass to the shader
    float* directions;
    float* curvatures;
    ofVec3f* slop;
    float* randOffset;
    int gridSize = 1;
    int gridNum;
    int waterVectorDis = 1; // distance for the water drainage vectors
    ofVec2f grad[SIZE][SIZE]; // store the average grad of the mesh
    ofVec3f visitedList[SIZE][SIZE]; // first digit: visited or not; second digit: parent; third digit: child
    WaterDrainage* vectorList[SIZE][SIZE];
    int waterColor[11][3] = {{51,110,255},{51,149,255},{51, 174, 255},{51,189,255},{51,209,255},{17,230,235},{17,235,192},{17,235,138},{235,206,17},{235,152,17},{235,82,17}}; // color palette for water drainage
    Eigen::MatrixXd gradU;
    Eigen::MatrixXd gradV;
    Eigen::VectorXd curU;
    Eigen::VectorXd curV;
    Eigen::MatrixXd windU;
    Eigen::MatrixXd windV;
    Eigen::MatrixXd BC;
    vector <Particle> particles;
    vector <Particle> flowers;
    ofVec3f gravity;
    ofVec3f wind;
    ofVec3f* windField;
    ofFbo fbo_water;
    ofFbo fbo_particle;
    ofFbo fbo_topView;
    ofFbo fbo_grad;
    
    // For Kinect
    ofxKinect kinect;
    
    // depth boundaries // MAKE THIS ADJUSTABLE
    int nearThreshold_ = 220;
    int farThreshold_ = 100;
    
    // pixel boundaries of sand // MAKE THIS ADJUSTABLE
    int xLeftBoundary_ = 244;
    int xRightBoundary_ = 450;
    int yTopBoundary_ = 108;
    int yBottomBoundary_ = 313;
    int blur_ = 10;
    float scale_ = 1;
    int topViewX_ = 1000;
    int topViewY_ = 100;
    int smoothRound_ = 5;
    int smoothWeight_ = 4;
    float cloud_ = 0.1;
    float water_ = 0.5;
    float contourLineDarkness_ = 0.5;
    float shadow_ = 0.5;
    float specular_ = 1.;
    float waterAlpha_ = 0.5;
    float meshAlpha_ = 1.;
    float meshBrightness_ = 1.;
    float meshSaturation_ = 1.;
    int gridSize_ = 2;
    float dotSize_ = 2.5;
    int lineWidth_ = 10;
    // setting the screen black after <timeout> seconds
    float timeout = 60; //seconds
    int startTimeout; //seconds
    bool isTimeout = false;
    ofxCvGrayscaleImage grayImage, grayImageSmall, grayImage_avg, grayThreshNear, grayThreshFar, grayOverall; // grayscale depth image
    // array to take  averages
    double prevPix[39149988];
    
    ofxIntSlider nearThreshold;
    ofxIntSlider farThreshold;
    
    ofxIntSlider xLeftBoundary;
    ofxIntSlider xRightBoundary;
    ofxIntSlider yTopBoundary;
    ofxIntSlider yBottomBoundary;
    ofxIntSlider blur;
    ofxFloatSlider scale;
    ofxIntSlider topViewX;
    ofxIntSlider topViewY;
    ofxIntSlider smoothRound;
    ofxIntSlider smoothWeight;
    ofxFloatSlider cloud;
    ofxFloatSlider water;
    ofxFloatSlider contourLineDarkness;
    ofxFloatSlider shadowContrast;
    ofxFloatSlider shadowRatio;
    ofxFloatSlider meshAlpha;
    ofxFloatSlider waterAlpha;
    ofxFloatSlider meshBrightness;
    ofxFloatSlider meshSaturation;
    ofxIntSlider grid;
    ofxFloatSlider dotSize;
    ofxIntSlider lineWidth;
    
    ofxCvGrayscaleImage croppedImg;
    int cropped_w;
    int cropped_h;
    
    
    //ofxVec2Slider DepthRange;
    ofxPanel gui;
    bool bHide;
    
    ofSpherePrimitive sun;
    
    ofVec2f up;
    ofVec2f down;
    ofVec2f left;
    ofVec2f right;
};
