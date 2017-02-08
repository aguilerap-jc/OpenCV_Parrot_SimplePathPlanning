#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "SDL/SDL.h"
#include <fstream>
#include <cmath>
/*
 * A simple 'getting started' interface to the ARDrone, v0.2 
 * author: Tom Krajnik
 * The code is straightforward,
 * check out the CHeli class and main() to see 
 */
#include <stdlib.h>
#include "CHeli.h"
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

bool TESTING = false;

int iLowH = 40;
int iHighH = 76;
int iLowS = 68;
int iHighS = 150;
int iLowV = 115;
int iHighV = 255;

int Px;
int Py;
int vR;
int vG;
int vB;

int n00 = 0;
int n10 = 1;
int n01 = 2;
int n20 = 3;
int n02 = 4;
int n11 = 5;

int counter = -1;

bool before = false;
bool stop = false;

bool freezedImg = false;
bool previousFreezedState = false;

bool conVuelo = true;

string filterNames[3] = {"RGB_", "HSV_","YIQ_"} ;
vector<Point> points;


CRawImage *image;
CHeli *heli;
float pitch, roll, yaw, height;
int hover=0;
int vectorCounter = 0;
vector<int> desitions;
// Joystick related
SDL_Joystick* m_joystick;
bool useJoystick;
int joypadRoll, joypadPitch, joypadVerticalSpeed, joypadYaw;
bool navigatedWithJoystick, joypadTakeOff, joypadLand, joypadHover;
string ultimo = "init";

Mat imagenClick;
Mat savedImage;

Mat grayscaleFilter(Mat originalImage);

//////////////////////////////////////////////
void getHuMoments(Moments oMoments, double xp, double yp, double  *mu);
Point getCentroid(Moments oMoments);
double getWidth(Moments oMoments);
double getHeight(Moments oMoments);
double getPhi(Moments oMoments, double *mu);
void getDecisions(Moments oMoments, double *huMoments, int vtc, double phi1, double phi2, double theta);
void executeDecisions();
double getPhi1(double *mu);
double getPhi2(double *mu);
//////////////////////////////////////////
/////////Other Filters Functions//////////
//////////////////////////////////////////
Mat MorphOpening(Mat original);

Mat MorphClosing(Mat original);

Mat waterShed(Mat originalImage);

Mat bWaterShed(Mat originalImage);

Mat freezeImage(char key, Mat image);
//////////////////////////////////////////
//////////////const IplImage *imageRGB///////////////
//////////////////////////////////////////
void RGBToHSVvalues(double *arrBGR, double *arrHSV);

void setHSVLimitValues(double *arrHSV, double *arrHSVLimits, double *arrThreshold);

void HSVcolorFilter(Mat originalImage);

////////////////////////////////////////////////////
///////////////////////VUELO////////////////////////
////////////////////////////////////////////////////
void pruebaVuelo() ;
void rightFlight();
void leftFlight();
void frontFlight();
void backFlight();
void setFlight();
void upFlight();
void downFlight();
//////////////////////////////////////////
//////////Other Matrix Functions//////////
//////////////////////////////////////////

//Get Histrograms of a given Matrix on BGR, 
//The imgType could be any given string
void histograms(Mat src, string imgType);

//Get the binarized image of the given matrix
void binFilter(Mat originalImage);

//Flip the given image, and store it on the destination matrix
void flipImageBasic(const Mat &sourceImage, Mat &destinationImage);

//codigo del click en pantalla
void mouseCoordinatesExampleCallback(int event, int x, int y, int flags, void* param);

// Convert CRawImage to Mat
void rawToMat( Mat &destImage, CRawImage* sourceImage);

//Drone Functions
void printPoint();

void printTelemetricData();

void controlFunction(char key);

void joyStickConfiguration();


//General Purpose Functions
double getMax(double *arr);

double getMin(double *arr);


int main(int argc,char* argv[]){
    //vector<Point> myPoints;
    //establishing connection with the quadcopter
    char key = 'c';
    heli = new CHeli();
    
    //this class holds the image from the drone 
    image = new CRawImage(320,240);
    
    // Initial values for control   
    pitch = roll = yaw = height = 0.0;
    joypadPitch = joypadRoll = joypadYaw = joypadVerticalSpeed = 0.0;

    // Destination OpenCV Mat   
    Mat currentImage = Mat(240, 320, CV_8UC3);

    // Initialize joystick
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK);
    useJoystick = SDL_NumJoysticks() > 0;
    if (useJoystick){
        SDL_JoystickClose(m_joystick);
        m_joystick = SDL_JoystickOpen(0);
    }
    joyStickConfiguration();
    /////////////////////////////////////////////////////////////////
    //////////////////FUNCIONES CLICK EN PANTALLA////////////////////
    /////////////////////////////////////////////////////////////////
    //namedWindow("Click");
    //setMouseCallback("Click", mouseCoordinatesExampleCallback);
    //Px = -10;
    //Py = -10;
    /////////////////////////////////////////////////////////////////
    //////////////////FUNCIONES CLICK EN PANTALLA////////////////////
    /////////////////////////////////////////////////////////////////


    //Flipped Image Original Matrices
    //Mat currentImage;
    Mat flippedImage;
    Mat fImage;

    while (key != 27){
        // Clear the console
        printf("\033[2J\033[1;1H");

        if (useJoystick){
            SDL_Event event;
            SDL_PollEvent(&event);

            joyStickConfiguration();
        }

        ///Print Telemetric Data
        printTelemetricData();

        //image is captured
        heli->renewImage(image);

        //namedWindow("Click");
        //setMouseCallback("Click", mouseCoordinatesExampleCallback);

        rawToMat(currentImage, image);
        imshow("ParrotCam", currentImage);
        
        if(key == 'f'){
            fImage = currentImage.clone();
            fImage = freezeImage(key, fImage);
            imshow("fImage", fImage);
            HSVcolorFilter(fImage);
        }
        HSVcolorFilter(currentImage);

        if (joypadTakeOff) {
            heli->takeoff();
        }
        if (joypadLand) {
            heli->land();
        }

        //hover = joypadHover ? 1 : 0;

        //setting the drone angles
        if (joypadRoll != 0 || joypadPitch != 0 || joypadVerticalSpeed != 0 || joypadYaw != 0)
        {
            heli->setAngles(joypadPitch, joypadRoll, joypadYaw, joypadVerticalSpeed, hover);
            navigatedWithJoystick = true;
        }
        else
        {
            heli->setAngles(pitch, roll, yaw, height, hover);
            navigatedWithJoystick = false;
        }
        //imshow("FImage", fImage);
        cout << "Freezed Image bool " << freezedImg << endl;
        key = waitKey(5);
        usleep(15000);
    }
    
    heli->land();
    SDL_JoystickClose(m_joystick);
    delete heli;
    delete image;
    return 0;
}

Mat freezeImage(char key, Mat image){
    //Mat frame = imread("greenSports.jpg",1);
    Mat img = image.clone();
    if(key == 'f'){
        if(previousFreezedState){
           previousFreezedState = false;
        }
        else{
            previousFreezedState = true;
        }
        
    }

    if(freezedImg == false && previousFreezedState == true){
        img = image.clone();
        freezedImg = true;
        savedImage = img.clone(); //imwrite("imagenVuelo.jpg", img);
        return img;
    }

    if(freezedImg && !previousFreezedState){
        freezedImg = false;
        destroyWindow("fImage");
    }
}

void setFlight(){
    for(int i = 0; i < (int)desitions.size(); i++){
        switch(desitions[i]){
            case 0:
                //NFL
                frontFlight();
                break;
            case 1:
                //Soccer
                backFlight();
                break;
            case 2:
                //Bate
                rightFlight();
                break;
            case 3:
                //Raqueta
                leftFlight();
                break;
            case 4:
                upFlight();
                break;
            case 5:
                downFlight();
                break;
            default :
                break;
            }
    }
}

void pruebaVuelo() {

    cout<<"funcion prueba"<<endl;
    //Despegue
    heli->takeoff();
    usleep(3500000);
    cout<<"takeoff"<<endl;

    //hover
    //heli->setAngles(pitch, roll, yaw, height, hover);
    heli->setAngles(0.0, 0.0, 0.0, 0.0, 1);
    usleep(2000000);
    cout<<"hover"<<endl;

    //Yaw
    //heli->setAngles(pitch, roll, yaw, height, hover);
    heli->setAngles(0.0, 0.0, 20000.0, 0.0, 0.0);
    usleep(1000000);
    cout<<"yaw"<<endl;

    //hover
    //heli->setAngles(pitch, roll, yaw, height, hover);
    heli->setAngles(0.0, 0.0, 0.0, 0.0, 1);
    usleep(2000000);
    cout<<"hover2"<<endl;

    //Pitch
    //heli->setAngles(pitch, roll, yaw, height, hover);
    heli->setAngles(-10000, 0.0, 0.0, 0.0, 0.0);
    usleep(500000);
    cout<<"pitch"<<endl;
}

int movementC = 0;
void hoverFlight(){
    //cout<<"Flying Hover"<<endl;
    heli->setAngles(0.0, 0.0, 0.0, 0.0, 1);
    usleep(200000);
    //movementC++;
    //executeDecisions();
}

void rightFlight(){
    //Roll
    cout << "Flying Right" << endl;
    heli->setAngles(0.0, -10000.0, 0.0, 0.0, 0.0);
    usleep(1000000);
    hoverFlight();
}

void leftFlight(){
    //Roll
    cout << "Flying left" << endl;
    heli->setAngles(0.0, 10000.0, 0.0, 0.0, 0.0);
    usleep(1000000);
    hoverFlight();
}

void frontFlight(){
    //Pitch
    cout << "Flying Front" << endl;
    heli->setAngles(-10000, 0.0, 0.0, 0.0, 0.0);
    usleep(1000000);
    hoverFlight();
}

void backFlight(){
    //Pitch
    cout << "Flying back" << endl;
    heli->setAngles(10000, 0.0, 0.0, 0.0, 0.0);
    usleep(1000000);
    hoverFlight();
}

void upFlight(){
    //Pitch
    cout << "Flying up" << endl;
    heli->setAngles(0.0, 0.0, 0.0, -35000.0, 0.0);
    usleep(1000000);
    hoverFlight();
}

void downFlight(){
    //Pitch
    cout << "Flying down" << endl;
    heli->setAngles(0.0, 0.0, 0.0, 25000.0, 0.0);
    usleep(1000000);
    hoverFlight();
}

double getWidth(Moments oMoments){ 
    return oMoments.m20/pow(oMoments.m00,2);
}

double getHeight(Moments oMoments){
    return oMoments.m02/pow(oMoments.m00,2);
}


double getPhi(Moments oMoments, double *mu){
    return 0.5*atan2(2*mu[n11], mu[n20] - mu[n02]);
}

Point getCentroid(Moments oMoments){
    return Point(oMoments.m10/oMoments.m00 , oMoments.m01/oMoments.m00);
}

void getHuMoments(Moments oMoments, double xp, double yp, double *mu){
    mu[n00] = oMoments.m00;
    mu[n10] = 0;
    mu[n01] = 0;
    mu[n20] = oMoments.m20 - xp*oMoments.m10;
    mu[n02] = oMoments.m02 - yp*oMoments.m01;
    mu[n11] = oMoments.m11 - yp*oMoments.m10;
}

double getPhi1(double *mu){
    return mu[n20]+mu[n02];
}
double getPhi2(double *mu){
    return pow(mu[n20]-mu[n02],2)+4*pow(mu[n11],2);
}

int movement1 = 0;
int movement2 = 0;
int movement3 = 0;
enum movements {kFront, kBack, kRight, kLeft, kUp, kDown};
void getDecisions(Moments oMoments, double *huMoments, int vtc, double phi1, double phi2, double theta){
    //for(int i = 0; i < contours.size(); i++){
    double area = oMoments.m00;
    //double phi = (getPhi(oMoments, huMoments))*180/CV_PI;
    //cout << "Moments: " << huMoments << endl;
    cout << "The Area: " << area << endl;
    //cout << "PHI... " << phi << endl;
    cout << "counter: " << counter << endl;
    // Soccer
    if (phi1 > 0.15 && phi1 < 0.175 && phi2 > 0 && phi2 < 0.005){
        if (movement1 == 0)
            movement1 = kLeft;
        else
            movement2 = kLeft;
        cout << "1" << endl;
    }
    // Americano
    else if (phi1 > 0.175 && phi1 < 0.21 && phi2 > 0.005 && phi2 < 0.018){
        if (movement1 == 0)
            movement1 = kRight;
        else
            movement2 = kRight;
        cout << "2" << endl;
    }
    // Raqueta
    else if (phi1 > 0.21 && phi1 < 0.275 && phi2 > 0.018 && phi2 < 0.04){
        if (movement1 == 0)
            movement1 = kFront;
        else
            movement2 = kFront;
        if (theta > 30)
            movement3 = kDown;
        else
            movement3 = kUp;
        cout << "3" << endl;
        cout << "Raqueta Theta " << theta << endl;
        // > 30 abajo
        // <= 30 arriba
    }
    // Bate
    else if (phi1 > 0.4 && phi1 < 0.45 && phi2 > 0.13 && phi2 < 0.16){
        if (movement1 == 0)
            movement1 = kBack;
        else
            movement2 = kBack;
        if (theta > 34)
            movement3 = kDown;
        else
            movement3 = kUp;
        cout << "4" << endl;
        cout << "Bate Theta " << theta << endl;
    }
// >34 abajo
// =<34 arriba
    // Dar prioridad al movimiento left|right y después front|back
    if (movement1 > 0 && movement2 > 0){
        if (movement1 == kFront || movement1 == kBack){
            int m = movement2;
            movement2 = movement1;
            movement1 = m;
        }
        cout << "5" << endl;
    }
}

void executeDecisions(){
    //if (movementC == 0)
    switch (movement1){
        case kFront:{frontFlight(); movement1 = 0;} break;
        case kBack:{backFlight(); movement1 = 0;} break;
        case kRight:{rightFlight(); movement1 = 0;} break;
        case kLeft:{leftFlight(); movement1 = 0;} break;
        case kUp:{upFlight(); movement1 = 0;} break;
        case kDown:{downFlight(); movement1 = 0;} break;
        default: break;
    }
    //else if (movementC == 1)
    switch (movement2){
        case kFront:{frontFlight(); movement2 = 0;} break;
        case kBack:{backFlight(); movement2 = 0;} break;
        case kRight:{rightFlight(); movement2 = 0;} break;
        case kLeft:{leftFlight(); movement2 = 0;} break;
        case kUp:{upFlight(); movement2 = 0;} break;
        case kDown:{downFlight(); movement2 = 0;} break;
        default: break;
    }
    //else if (movementC == 2)
    switch (movement3){
        case kFront:{frontFlight(); movement3 = 0;} break;
        case kBack:{backFlight(); movement3 = 0;} break;
        case kRight:{rightFlight(); movement3 = 0;} break;
        case kLeft:{leftFlight(); movement3 = 0;} break;
        case kUp:{upFlight(); movement3 = 0;} break;
        case kDown:{downFlight(); movement3 = 0;} break;
        default: break;
    }
}


//////////////////////////////////////////
//////////////HSV Functions///////////////
//////////////////////////////////////////

///Create a class of Image Processing///


void HSVcolorFilter(Mat originalImage) {
    Mat orig = originalImage.clone();
    Mat src_gray;
    Mat imgHSV;
    Mat imgThresholded;
    //ImageFilter filtro;
    Mat BGRHSVimg;

    cvtColor(orig, imgHSV, COLOR_BGR2HSV);       // Convert the captured frame from BGR to HSV.
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), 
    Scalar(iHighH, iHighS, iHighV), imgThresholded); // Threshold the image.
    
    // Morphological opening (remove small objects from the foreground).
    imgThresholded = MorphOpening(imgThresholded);
    // Morphological closing (fill small holes in the foreground).
    imgThresholded = MorphClosing(imgThresholded);
    // Morphological opening (remove small objects from the foreground).
    imgThresholded = MorphOpening(imgThresholded);
    // Morphological closing (fill small holes in the foreground).
    imgThresholded = MorphClosing(imgThresholded);
    
    // Get the moments.
    //Moments oMoments = moments(imgThresholded);

    // Receive the centroid area.
    
    // Cloned the modified image to calculate the points.
    src_gray = imgThresholded.clone();
    blur(src_gray, src_gray, Size(3,3));
    src_gray = imgThresholded.clone();
    blur(src_gray, src_gray, Size(3,3));

    Mat other_filter;

    //Lamada de Watershed evitando la binarización

    bitwise_and(imgHSV,imgHSV,other_filter, src_gray = src_gray);
    cvtColor(other_filter, BGRHSVimg, CV_HSV2BGR);

    if(TESTING){
        namedWindow( "HSV_Filter", WINDOW_NORMAL );
        cvCreateTrackbar("LowH" , "HSV_Filter", &iLowH, 179);   // Hue (0 - 179).
        cvCreateTrackbar("HighH", "HSV_Filter", &iHighH, 179);
        cvCreateTrackbar("LowS" , "HSV_Filter", &iLowS, 255);   // Saturation (0 - 255).
        cvCreateTrackbar("HighS", "HSV_Filter", &iHighS, 255);
        cvCreateTrackbar("LowV" , "HSV_Filter", &iLowV, 255);   // Value (0 - 255).
        cvCreateTrackbar("HighV", "HSV_Filter", &iHighV, 255);
    }

    //imshow("BGR", BGRHSVimg);
    Mat myW = waterShed(BGRHSVimg);

    imshow("WaterS", myW);
    imshow("HSV Filter", src_gray);
    //imshow("HSV Filter", src_gray);
    //namedWindow("HSV Filter", CV_WINDOW_AUTOSIZE);         // Create a window called "Control".

}

Mat graphic(500,500,CV_8UC3,Scalar(0));
int phisCount = 0;

Mat waterShed(Mat originalImage){
    int vtc;
    Mat src = originalImage.clone();
    
    if(TESTING)
        imshow("Input Image WaterShed", originalImage);

    Mat bw;
    cvtColor(src, bw, CV_BGR2GRAY);
    threshold(bw, bw, 127, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    if(TESTING)
        imshow("Imagen Binaria", bw);
    
    // Morphological opening (remove small objects from the foreground).
    bw = MorphOpening(bw);
    // Morphological closing (fill small holes in the foreground).
    bw = MorphClosing(bw);

    // Morphological opening (remove small objects from the foreground).
    bw = MorphOpening(bw);
    // Morphological closing (fill small holes in the foreground).
    bw = MorphClosing(bw);

    Mat dist = originalImage.clone();
    distanceTransform(bw, dist, CV_DIST_L1, 3);
    normalize(dist, dist, 0, 4., NORM_MINMAX);
    if(TESTING)
        imshow("Dist", dist);

    if(TESTING)
        imshow("Dist,Thresh", dist);

    Mat dist_8u;
    dist.convertTo(dist_8u, CV_8U);
    vector< vector<Point> > contours;
    findContours(dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    Mat markers = Mat::zeros(dist.size(), CV_32SC1);
    for (size_t i = 0; i < contours.size(); i++)
        drawContours(markers, contours, i, Scalar::all(i+1), -1);
    circle(markers, Point(5,5), 3, CV_RGB(255,255,255), -1);
    //imshow("Marcadores", markers*10000);

    // Aplicamos el efecto de gota de aceite
    watershed(src, markers);
    Mat mark = Mat::zeros(markers.size(), CV_8UC1);
    markers.convertTo(mark, CV_8UC1);
    bitwise_not(mark, mark);
    //imshow("Markers_v2", mark);

    //Arrat for storing n number of areas
    double areas[contours.size()];
    //Obtener x testada y testada
    double xp[contours.size()];
    double yp[contours.size()];

    double huMoments[contours.size()][7];
    Moments oMoments[contours.size()];

    vector<Moments> mu(contours.size());
    vector<Point2f> mc(contours.size());
    vector<Point> approx(contours.size());
    vector<Vec4f> vPoints(contours.size());
    //vector<Point> approx;

    for(int i = 0; i < contours.size(); i++){
        //Vec4f vPoints;
        //Get the moments of the area
        //oMomens[i] = moments(bw);
        areas[i] = contourArea(contours[i]);
        oMoments[i] = moments(Mat(contours[i]));
        HuMoments(oMoments[i], huMoments[i]);

        mc[i] = Point2f(oMoments[i].m10/oMoments[i].m00, oMoments[i].m01/oMoments[i].m00);
        //Get area
    }

    vector<Vec3b> colorsG;
    colorsG.push_back(Vec3b((uchar)0, (uchar)0, (uchar)255));
    colorsG.push_back(Vec3b((uchar)0, (uchar)255, (uchar)0));
    colorsG.push_back(Vec3b((uchar)255, (uchar)0, (uchar)0));
    colorsG.push_back(Vec3b((uchar)255, (uchar)0, (uchar)255));
    colorsG.push_back(Vec3b((uchar)0, (uchar)255, (uchar)255));
    colorsG.push_back(Vec3b((uchar)255, (uchar)255, (uchar)0));
    colorsG.push_back(Vec3b((uchar)255, (uchar)255, (uchar)255));
    

    for(int i = 0; i < contours.size(); i++){
        //cout << "Area Contours "<<areas[i] << endl;
        //cout << "Area Moments " << oMoments[i].m00 << endl;
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.01, true);
        double theta = (0.5*atan2(2*oMoments[i].m11,oMoments[i].m20-oMoments[i].m02))*180/CV_PI;
        if(freezedImg){//!conVuelo){
            //counter++;
            getDecisions(oMoments[i], huMoments[i], vtc, huMoments[i][0],huMoments[i][1], theta);
        }
        //HuMoments(oMoments[i], huMoments[i]);
        //cout << "Phi " << (getPhi(oMoments[i], huMoments[i]))*180/CV_PI  << endl;
        //cout << "Phi2 " << (0.5*atan2(2*oMoments[i].m11,oMoments[i].m20-oMoments[i].m02))*180/CV_PI  << endl;
    }   

    if(freezedImg && movement1 > 0){
        movementC = 0;
        executeDecisions();
        freezedImg = false;
    }

    vector<Vec3b> colors;
    for (size_t i = 0; i < contours.size(); i++){
        int b = theRNG().uniform(0, 255);
        int g = theRNG().uniform(0, 255);
        int r = theRNG().uniform(0, 255);
        colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
    }

    Mat dst = Mat::zeros(markers.size(), CV_8UC3);
    for (int i = 0; i < markers.rows; i++){
        for (int j = 0; j < markers.cols; j++){
            int index = markers.at<int>(i,j);
            if (index > 0 && index <= static_cast<int>(contours.size()))
                dst.at<Vec3b>(i,j) = colors[index-1];
            else
                dst.at<Vec3b>(i,j) = Vec3b(0,0,0);
        }
    }

    
    for(int i = 0; i < contours.size(); i++){
        double theta = (0.5*atan2(2*oMoments[i].m11,oMoments[i].m20-oMoments[i].m02))*180/CV_PI;
        
        circle(dst,mc[i], 3, Scalar(0,0,255),3,0,0);
        RotatedRect minRect = minAreaRect(Mat(contours[i]));
        Point2f rect_points[4];
        minRect.points(rect_points);
        vector<Point> vPoints2(4);

        double angle = (getPhi(oMoments[i], huMoments[i]))*180/CV_PI;
        Rect rect = boundingRect(contours[i]);
        Point p1 = Point(mc[i].x+rect.width/2.0,mc[i].y-rect.height/2.0);
        Point p2 = Point(mc[i].x-rect.width/2.0,mc[i].y+rect.height/2.0);
        line(dst, p2, p1, Scalar(0,255,0),2);
        Point p1c = Point(mc[i].x-rect.width/2.0,mc[i].y-rect.height/2.0);
        Point p2c = Point(mc[i].x+rect.width/2.0,mc[i].y+rect.height/2.0);
        line(dst, p2c, p1c, Scalar(0,255,0),2);
        circle(dst,mc[i], 3, Scalar(0,0,255),3,0,0);

        /*
        for(int j = 0; j < 4; j++){

            line(dst, rect_points[j], rect_points[(j+1)%4], Scalar(255,0,0), 1, 8); // blue
            double maxX = MAX(rect_points[j].x,rect_points[(j+1)%4].x);
            double maxY = MAX(rect_points[j].y,rect_points[(j+1)%4].y);
            double deltaX = maxX-MIN(rect_points[j].x,rect_points[(j+1)%4].x);
            double deltaY = maxY-MIN(rect_points[j].y,rect_points[(j+1)%4].y);
            Point p1 = Point(maxX-deltaX/2.0, maxY-deltaY/2.0);
            vPoints2.push_back(p1);
        }
        */
        line(dst, vPoints2[0], vPoints2[2], Scalar(0,0,255),2); // red
        line(dst, vPoints2[1], vPoints2[3], Scalar(255,255,0),2); // yellow

        double *mu = huMoments[i];
        double phi1 = huMoments[i][0];
        double phi2 = huMoments[i][1];
        //double phi1 = getPhi1(huMoments[i]);
        //double phi2 = getPhi2(huMoments[i]);
        
        /*
        ofstream myFiles;
        stringstream out;
        out << i;
        std::string fileName1 = "phi1" + out.str() + ".txt";
        std::string fileName2 = "phi2" + out.str() + ".txt";
        myFiles.open(fileName1.c_str(),std::ios_base::app);
        myFiles << phi1 << "\n";
        myFiles.close();
        myFiles.open(fileName2.c_str(),std::ios_base::app);
        myFiles << phi2 << "\n";
        myFiles.close();
        */

        cout << "Theta " << theta << endl;
        cout << "1: " << phi1 << " 2: " << phi2 << endl;
        circle(graphic, Point(phi1*1000, phi2*1000), 5, colorsG[i], -1);
    }
    
    imshow("Graphic", graphic);

    return dst;
}
///////////////////////////////////////////////////////////////////////////
///////////////////////END OF MY FUNCTIONS/////////////////////////////////
///////////////////////////////////////////////////////////////////////////

///Create a class of parrot control///

/////////////Drone Image //////////////
/*
 * This method flips horizontally the sourceImage into destinationImage. Because it uses 
 * "Mat::at" method, its performance is low (redundant memory access searching for pixels).
 */
void flipImageBasic(const Mat &sourceImage, Mat &destinationImage){
/*
 * Title: Flipped Image Sample (Cascaron)
 * Class: Vision para Robot
 * Instructor: Dr. Jose Luis Gordillo (http://LabRob.mty.itesm.mx/)
 * Code: Manlio Barajas (manlito@gmail.com)
 * Institution: Tec de Monterrey, Campus Monterrey
 * Date: January 10, 2012
 *
 * Description: This Function takes input from a camera (recognizable by
 * OpenCV) and it flips it horizontally.
 * "Basic" version uses frequently the "Cv::Mat::At" method which slows down
 * performance. This Function has illustrative purposes, provided the existence
 * of cv::flip method.
 *
 * TODO: Validate when source and destination image are the same
 *
 * This programs uses OpenCV http://opencv.willowgarage.com/wiki/
 */

    if (destinationImage.empty())
        destinationImage = Mat(sourceImage.rows, sourceImage.cols, sourceImage.type());

    for (int y = 0; y < sourceImage.rows; ++y)
        for (int x = 0; x < sourceImage.cols / 2; ++x)
            for (int i = 0; i < sourceImage.channels(); ++i)
            {
                destinationImage.at<Vec3b>(y, x)[i] = sourceImage.at<Vec3b>(y, sourceImage.cols - 1 - x)[i];
                destinationImage.at<Vec3b>(y, sourceImage.cols - 1 - x)[i] = sourceImage.at<Vec3b>(y, x)[i];
            }
}

//codigo del click en pantalla
void mouseCoordinatesExampleCallback(int event, int x, int y, int flags, void* param){
    uchar* destination;
    switch (event)
    {
        case CV_EVENT_LBUTTONDOWN:
            Px=x;
            Py=y;
            destination = (uchar*) imagenClick.ptr<uchar>(Py);
            vB=destination[Px * 3];
            vG=destination[Px*3+1];
            vR=destination[Px*3+2];
            //cout << vB << " " << vG << " " << vR << endl;
            //cout << "  Mouse X, Y: " << x << ", " << y ;
            //cout << endl;
            before = true;
            points.push_back(Point(x, y));
            vectorCounter++;
            break;
        case CV_EVENT_MOUSEMOVE:
            break;
        case CV_EVENT_LBUTTONUP:
            break;
        case CV_EVENT_RBUTTONDOWN:
        //flag=!flag;
            break;
        
    }
}

// Convert CRawImage to Mat
void rawToMat( Mat &destImage, CRawImage* sourceImage){ 
    uchar *pointerImage = destImage.ptr(0);
    
    for (int i = 0; i < 240*320; i++)
    {
        pointerImage[3*i] = sourceImage->data[3*i+2];
        pointerImage[3*i+1] = sourceImage->data[3*i+1];
        pointerImage[3*i+2] = sourceImage->data[3*i];
    }
}

/////////////Drone Functions//////////////
void printPoint(){
    if(before){
        circle(imagenClick,Point(Px,Py),5,Scalar(0,255,0), -1, 8, 0);
        /*
        if(points.size() > 0){
            for(int i = 0; i < points.size()-1; i++){
                line(imagenClick,points[i],points[i+1], Scalar(255,0,0), 10, 8, 0);
            }
        }
        */
    }
}

void printTelemetricData(){

        // prints the drone telemetric data, helidata struct contains drone angles, speeds and battery status
        printf("===================== Parrot Basic Example =====================\n\n");
        fprintf(stdout, "Angles  : %.2lf %.2lf %.2lf \n", helidata.phi, helidata.psi, helidata.theta);
        fprintf(stdout, "Speeds  : %.2lf %.2lf %.2lf \n", helidata.vx, helidata.vy, helidata.vz);
        fprintf(stdout, "Battery : %.0lf \n", helidata.battery);
        fprintf(stdout, "Hover   : %d \n", hover);
        fprintf(stdout, "Joypad  : %d \n", useJoystick ? 1 : 0);
        fprintf(stdout, "  Roll    : %d \n", joypadRoll);
        fprintf(stdout, "  Pitch   : %d \n", joypadPitch);
        fprintf(stdout, "  Yaw     : %d \n", joypadYaw);
        fprintf(stdout, "  V.S.    : %d \n", joypadVerticalSpeed);
        fprintf(stdout, "  TakeOff : %d \n", joypadTakeOff);
        fprintf(stdout, "  Land    : %d \n", joypadLand);
        fprintf(stdout, "Navigating with Joystick: %d \n", navigatedWithJoystick ? 1 : 0);
        cout<<"Pos X: "<<Px<<" Pos Y: "<<Py<<" Valor RGB: ("<<vR<<","<<vG<<","<<vB<<")"<<endl;
}

void controlFunction(char key){
     //char key = waitKey(5);
        switch (key) {
            case 'a': yaw = -20000.0; break;
            case 'd': yaw = 20000.0; break;
            case 'w': height = -20000.0; break;
            case 's': height = 20000.0; break;
            case 'q': heli->takeoff(); break;
            case 'e': heli->land(); break;
            case 'z': heli->switchCamera(0); break;
            case 'x': heli->switchCamera(1); break;
            case 'c': heli->switchCamera(2); break;
            case 'v': heli->switchCamera(3); break;
            case 'j': roll = -20000.0; break;
            case 'l': roll = 20000.0; break;
            case 'i': pitch = -20000.0; break;
            case 'k': pitch = 20000.0; break;
            case 'h': hover = (hover + 1) % 2; break;
            case 27: stop = true; break;
            case 'm': pruebaVuelo(); break;
            default: pitch = roll = yaw = height = 0.0;
        }
}

void joyStickConfiguration(){
    joypadRoll = SDL_JoystickGetAxis(m_joystick, 2);
    joypadPitch = SDL_JoystickGetAxis(m_joystick, 3);
    joypadVerticalSpeed = SDL_JoystickGetAxis(m_joystick, 1);
    joypadYaw = SDL_JoystickGetAxis(m_joystick, 0);
    joypadTakeOff = SDL_JoystickGetButton(m_joystick, 1);
    joypadLand = SDL_JoystickGetButton(m_joystick, 2);
    joypadHover = SDL_JoystickGetButton(m_joystick, 0);
}

void RGBToHSVvalues(double *arrBGR, double *arrHSV){
    double min,max;
    arrBGR[0] = vB;
    arrBGR[1] = vG;
    arrBGR[2] = vR;
    arrBGR[0] = arrBGR[0]/255;
    arrBGR[1] = arrBGR[1]/255;
    arrBGR[2] = arrBGR[2]/255; 

    min = getMin(arrBGR);
    max = getMax(arrBGR);

    if(min == max){
        arrHSV[0] = 0; 
        arrHSV[1] = 0;
        arrHSV[2] = min;
    }else{
        double d = (arrBGR[2]==min) ? arrBGR[1]-arrBGR[0]: ((arrBGR[0]==min) ? arrBGR[2]-arrBGR[1] : arrBGR[0]-arrBGR[2]);
        double h = (arrBGR[2]==min) ? 3 : ((arrBGR[0]==min) ? 1 : 5);
        arrHSV[0] =  60*(h - d/(max - min));
        arrHSV[1] =  ((max - min)/max)*100; //Normalizing it to percentage
        arrHSV[2] =  max * 100; //Normalizing it to percentage
    }
    cout << arrBGR[0] << endl;
    cout << arrHSV[0] << endl;
}

void setHSVLimitValues(double *arrHSV, double *arrHSVLimits, double *arrThreshold){
    //Set the limits of the Hue between 0 and 179 
    if(  ((arrHSV[0]/2)- arrThreshold[0]) < 0)
        arrHSVLimits[0] = 0;
    else
        arrHSVLimits[0] = (arrHSV[0]/2)-arrThreshold[0];

    if(((arrHSV[0]/2)+arrThreshold[0]) > 179)
        arrHSVLimits[1] = 179;
    else
        arrHSVLimits[1] = (arrHSV[0]/2)+arrThreshold[0];

    //Set the limits of the Saturation between 0 and 255 
    if((((arrHSV[1]/2)*2.55)-arrThreshold[1]) < 0)
        arrHSVLimits[2] = 0;
    else 
        arrHSVLimits[2] = ((arrHSV[1]/2)*2.55)-arrThreshold[1];
    
    if((((arrHSV[1]/2)*2.55)+arrThreshold[1]) > 255)
        arrHSVLimits[3] = 255;
    else
        arrHSVLimits[3] = ((arrHSV[1]/2)*2.55)+arrThreshold[1];
    
    //Set the limits of the Value between 0 and 255 
    if((((arrHSV[2]/2)*2.55)-arrThreshold[2]) < 0)
        arrHSVLimits[4] = 0;
    else
        arrHSVLimits[4] = ((arrHSV[2]/2)*2.55)-arrThreshold[2];
    
    if((((arrHSV[2]/2)*2.55)+arrThreshold[2]) > 255)
        arrHSVLimits[5] = 255;
    else
        arrHSVLimits[5] = ((arrHSV[2]/2)*2.55)+arrThreshold[2];
}

Mat MorphClosing(Mat original){
    // Morphological closing (fill small holes in the foreground).
    dilate( original, original, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(original, original, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    return original;
}

Mat MorphOpening(Mat original){
    // Morphological opening (remove small objects from the foreground).
    erode(original, original, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( original, original, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    return original;
}
///Create a class of Image Processing///



//////////////////////////////////////////
//////////Other Matrix Functions//////////
//////////////////////////////////////////

void binFilter(Mat originalImage){
    Mat src_gray = originalImage.clone();
    cvtColor( originalImage, src_gray, CV_BGR2GRAY );
    //Binary image
    Mat binaryMat(src_gray.size(), src_gray.type());

    //Apply thresholding
    threshold(src_gray, binaryMat, 100, 255, cv::THRESH_BINARY);

    //Show the results
    namedWindow("Binarized", cv::WINDOW_AUTOSIZE);
    imshow("Binarized", binaryMat);
}

Mat grayscaleFilter(Mat originalImage){
    Mat src_gray = originalImage.clone();
    cvtColor( originalImage, src_gray, CV_BGR2GRAY );
    //imshow("src_gray",src_gray);
    return src_gray;
}

void histograms(Mat src, string imgType){
    Mat dst;
    //int m = 4, n = 4, x ,y ;
    int scale = 1;
    /// Separate the image in 3 places ( B, G and R )
    vector<Mat> bgr_planes;
    split( src, bgr_planes );

    /// Establish the number of bins
    int histSize = 256;

    /// Set the ranges ( for B,G,R) )
    float range[] = { 0, 256 } ;
    const float* histRange = { range };

    bool uniform = true; bool accumulate = false;

    Mat b_hist, g_hist, r_hist;

    /// Compute the histograms:
    calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
    calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
    calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

    // Draw the histograms for B, G and R
    int hist_w = 320; int hist_h = 240;
    int bin_w = cvRound( (double) hist_w/histSize );

    Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
    Mat histImageB( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
    Mat histImageG( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
    Mat histImageR( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

    /// Normalize the result to [ 0, histImage.rows ]
    normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    /// Draw for each channel
    for( int i = 1; i < histSize; i++ ){
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                       Scalar( 255, 0, 0), 2, 8, 0  );

      line( histImageB, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                       Scalar( 255, 0, 0), 2, 8, 0  );
      
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                       Scalar( 0, 255, 0), 2, 8, 0  );

      line( histImageG, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                       Scalar( 0, 255, 0), 2, 8, 0  );
      
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                       Scalar( 0, 0, 255), 2, 8, 0  );

      line( histImageR, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                       Scalar( 0, 0, 255), 2, 8, 0  );
      
    }

    imshow(imgType, histImage );
    imshow(imgType + "B", histImageB );
    imshow(imgType + "G", histImageG );
    imshow(imgType + "R", histImageR );
    //namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
}

double getMin(double *arr){
    int size = 3;
    double min = 999999;
    for(int i = 0; i < size; i++){
        if(arr[i] < min){
            min = arr[i];
        }
    }

    return min;
}

double getMax(double *arr){
    int size = 3;
    double max = -1;
    for(int i = 0; i < size; i++){
        if(arr[i] > max){
            max = arr[i];
        }
    }

    return max;
}