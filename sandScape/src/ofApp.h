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

#define SIZE 80 // size of the mesh: SIZE * SIZE

extern float u_mode, snowHeight, meshX, meshY, meshZ, cloudAmount, specular;
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
    void renderVectorField(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &grad, Eigen::VectorXd &cur, Eigen::MatrixXd &wind, ofVec3f* slope, float drawMode);
    void constructWindField(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &windU, Eigen::MatrixXd &windV);
    void findPath(int indX, int indY);
    
    // shader programs
    ofxShader shader;
    ofxShader sunShader;
//    ofxShader cloudShader; // replaced with video cloud for stability
    ofxShader rainShader;
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
    ofVec3f* slope;
    float* randOffset;
    int gridSize = 1;
    int gridNum;
    int waterVectorDis = 1; // distance for the water drainage vectors
    ofVec2f grad[SIZE][SIZE]; // store the average grad of the mesh
    ofVec3f visitedList[SIZE][SIZE]; // first digit: visited or not; second digit: parent; third digit: child
    WaterDrainage* vectorList[SIZE][SIZE];
    int waterColor[11][3] = {{51,110,255},{51,149,255},{51, 174, 255},{51,189,255},{51,209,255},{17,230,235},{17,235,192},{17,235,138},{235,206,17},{235,152,17},{235,82,17}}; // color palette for the water drainage acceleration: slow -> fast
    Eigen::MatrixXd gradU;
    Eigen::MatrixXd gradV;
    Eigen::VectorXd curU;
    Eigen::VectorXd curV;
    Eigen::MatrixXd windU;
    Eigen::MatrixXd windV;
    Eigen::MatrixXd BC;
    
    vector <Particle> particles;
    vector <Particle> flowers;
    vector <Particle> rain;
    
    ofVec3f gravity;
    ofVec3f wind;
    ofVec3f* windField;
    
    // frame buffers for rain/flowers/snow particle rendering
    ofFbo fbo_water;
    ofFbo fbo_particle;
    ofFbo fbo_topView;
    ofFbo fbo_sideView;
    ofFbo fbo_grad;
    ofFbo fbo_rainBack;
    ofFbo fbo_rainMid;
    ofFbo fbo_rainFront;
    
    // For Kinect
    ofxKinect kinect;
    
    // default parameters for the GUI
    int nearThreshold_ = 220;
    int farThreshold_ = 100;
    int xLeftBoundary_ = 244;
    int xRightBoundary_ = 450;
    int yTopBoundary_ = 108;
    int yBottomBoundary_ = 313;
    float scale_ = 1;
    int topViewX_ = 1000;
    int topViewY_ = 100;
    int smoothRound_ = 5;
    int smoothWeight_ = 4;
    float specular_ = 1.;
    float transitionSpeed_ = 0.1;
    int gridSize_ = 2;
    float dotSize_ = 2.5;
    int lineWidth_ = 10;
    float rotX_ = 45;
    float posX_ = 0;
    float posY_ = 0;
    float posZ_ = 0;
    // setting the screen black after <timeout> seconds
    float timeout = 60; //seconds
    int startTimeout; //seconds
    bool isTimeout = false;
    ofxCvGrayscaleImage grayImage, grayImageSmall, grayImage_avg, grayThreshNear, grayThreshFar, grayOverall; // grayscale depth image
    // array to take  averages
    double prevPix[39149988];
    
    // parameters to set up the Kinect
    ofxIntSlider nearThreshold;
    ofxIntSlider farThreshold;
    ofxIntSlider xLeftBoundary;
    ofxIntSlider xRightBoundary;
    ofxIntSlider yTopBoundary;
    ofxIntSlider yBottomBoundary;
    // to scale and translate the top view projection
    ofxFloatSlider scale;
    ofxIntSlider topViewX;
    ofxIntSlider topViewY;
    ofxIntSlider smoothRound;
    ofxIntSlider smoothWeight;
    // the transition speed of the elevation <-> water drainage
    ofxFloatSlider transitionSpeed;
    // the shadow constrast in the shadow mode
    ofxFloatSlider shadowContrast;
    // parameters for the water drainage visual
    ofxIntSlider grid;
    ofxFloatSlider dotSize;
    ofxIntSlider lineWidth;
    // adjustment for the side view of the mesh
    ofxFloatSlider rotX;
    ofxFloatSlider posX;
    ofxFloatSlider posY;
    ofxFloatSlider posZ;
    
    // for getting and storing the valid gray scale image from Kinect
    ofxCvGrayscaleImage croppedImg;
    int cropped_w;
    int cropped_h;
    unsigned char* pre_pix;
    unsigned char* cropped;
    unsigned char* pix;
    
    //ofxVec2Slider DepthRange;
    ofxPanel gui;
    bool bHide;
    
    // the 3D sun in the shadow mode
    ofSpherePrimitive sun;
    
    // the plane to render the rain
    ofPlanePrimitive rainPlane;
    
    // for base vectors used in water drainage calculation
    ofVec2f up;
    ofVec2f down;
    ofVec2f left;
    ofVec2f right;
    
    // the cloud video played in the elevation <-> water drainage mode
    ofVideoPlayer cloudVideo;
    bool resetVideo = false;
    
    // screen size
    int DISPLAY_WIDTH;
    int DISPLAY_HEIGHT;
};
