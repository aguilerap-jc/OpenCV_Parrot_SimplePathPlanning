#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "SDL/SDL.h"
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

using namespace std;
using namespace cv;

int iLowH;
int iHighH;
int iLowS;
int iHighS;
int iLowV;
int iHighV;

vector<Point> points;

bool stop = false;
CRawImage *image;
CHeli *heli;
float pitch, roll, yaw, height;
int hover=0;
// Joystick related
SDL_Joystick* m_joystick;
bool useJoystick;
int joypadRoll, joypadPitch, joypadVerticalSpeed, joypadYaw;
bool navigatedWithJoystick, joypadTakeOff, joypadLand, joypadHover;
string ultimo = "init";
bool before = false;

int Px;
int Py;
int vR;
int vG;
int vB;

Mat imagenClick;

Mat grayscaleFilter(Mat originalImage);
/*
 * This method flips horizontally the sourceImage into destinationImage. Because it uses 
 * "Mat::at" method, its performance is low (redundant memory access searching for pixels).
 */
void flipImageBasic(const Mat &sourceImage, Mat &destinationImage);

void printPoint();

void printTelemetricData();

void controlFunction();

void HSVcolorFilter(Mat originalImage);
// Convert CRawImage to Mat
void rawToMat( Mat &destImage, CRawImage* sourceImage);

//codigo del click en pantalla
void mouseCoordinatesExampleCallback(int event, int x, int y, int flags, void* param);

int main(int argc,char* argv[]){
	//establishing connection with the quadcopter
	heli = new CHeli();
	
	//this class holds the image from the drone	
	image = new CRawImage(320,240);
	
	// Initial values for control	
    pitch = roll = yaw = height = 0.0;
    joypadPitch = joypadRoll = joypadYaw = joypadVerticalSpeed = 0.0;

	// Destination OpenCV Mat	
	Mat currentImage = Mat(240, 320, CV_8UC3);
	// Show it	
	imshow("ParrotCam", currentImage);

    // Initialize joystick
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK);
    useJoystick = SDL_NumJoysticks() > 0;
    if (useJoystick)
    {
        SDL_JoystickClose(m_joystick);
        m_joystick = SDL_JoystickOpen(0);
    }

    namedWindow("Click");
    setMouseCallback("Click", mouseCoordinatesExampleCallback);

    //Flipped Image Original Matrices
    //Mat currentImage;
    Mat flippedImage;
    
    Px = -10;
    Py = -10;
    while (stop == false)
    {

        // Clear the console
        printf("\033[2J\033[1;1H");

        if (useJoystick){
            SDL_Event event;
            SDL_PollEvent(&event);

            joypadRoll = SDL_JoystickGetAxis(m_joystick, 2);
            joypadPitch = SDL_JoystickGetAxis(m_joystick, 3);
            joypadVerticalSpeed = SDL_JoystickGetAxis(m_joystick, 1);
            joypadYaw = SDL_JoystickGetAxis(m_joystick, 0);
            joypadTakeOff = SDL_JoystickGetButton(m_joystick, 1);
            joypadLand = SDL_JoystickGetButton(m_joystick, 2);
            joypadHover = SDL_JoystickGetButton(m_joystick, 0);
        }

        ///Print Telemetric Data
        printTelemetricData();

		//image is captured
		heli->renewImage(image);

        namedWindow("Click");
        setMouseCallback("Click", mouseCoordinatesExampleCallback);

		// Copy to OpenCV Mat


		rawToMat(currentImage, image);
		imshow("ParrotCam", currentImage);
        
        //
        imagenClick=currentImage.clone();
        printPoint();
        imshow("Click", imagenClick);
        
        //Flip the original image
        flipImageBasic(currentImage,flippedImage);
        imshow("Flipped", flippedImage);
        
        //Convert original Image to gray
        Mat src_gray = grayscaleFilter(currentImage);
        imshow("src_gray", src_gray);

        //Control Function key Wait
        controlFunction();
       

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

        usleep(15000);
	}
	
	heli->land();
    SDL_JoystickClose(m_joystick);
    delete heli;
	delete image;
	return 0;
}

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
            cout << "  Mouse X, Y: " << x << ", " << y ;
            cout << endl;
            //rectangle(imagenClick, Point(x,y), Point(x+55,y+55) , Scalar(0,255,255), -1, 8, 0);
            //imshow("Click", imagenClick);
            /*  Draw a point */
            before = true;
            points.push_back(Point(x, y));

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

void controlFunction(){
     char key = waitKey(5);
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
            default: pitch = roll = yaw = height = 0.0;
        }
}

Mat grayscaleFilter(Mat originalImage){
    Mat src_gray = originalImage.clone();
    cvtColor( originalImage, src_gray, CV_BGR2GRAY );
    //imshow("src_gray",src_gray);
    return src_gray;
}

void printPoint(){
    if(before){
        rectangle(imagenClick, Point(Px,Py), Point(Px+10,Py+10) , Scalar(0,255,255), -1, 8, 0);
    }
}

void HSVcolorFilter(Mat originalImage) {
    Mat src_gray;
    Mat imgHSV;
    Mat imgThresholded;
    imgHSV = originalImage.clone(); 
    cvtColor(imgHSV, imgHSV, COLOR_BGR2HSV);// Convert the captured frame from BGR to HSV.
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), 
    Scalar(iHighH, iHighS, iHighV), imgThresholded);// Threshold the image.

    // Morphological opening (remove small objects from the foreground).
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    // Morphological closing (fill small holes in the foreground).
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    // Get the moments.
    Moments oMoments = moments(imgThresholded);

    // Receive the centroid area.
    
    // Cloned the modified image to calculate the points.
    src_gray = imgThresholded.clone();
    blur(src_gray, src_gray, Size(3,3));

    imshow("HSV Filter", src_gray);
    namedWindow("HSV Filter", CV_WINDOW_AUTOSIZE);         // Create a window called "Control".

    // Create trackbars in "Control" window.
    cvCreateTrackbar("LowH" , "HSV Filter", &iLowH, 179);   // Hue (0 - 179).
    cvCreateTrackbar("HighH", "HSV Filter", &iHighH, 179);
    cvCreateTrackbar("LowS" , "HSV Filter", &iLowS, 255);   // Saturation (0 - 255).
    cvCreateTrackbar("HighS", "HSV Filter", &iHighS, 255);
    cvCreateTrackbar("LowV" , "HSV Filter", &iLowV, 255);   // Value (0 - 255).
    cvCreateTrackbar("HighV", "HSV Filter", &iHighV, 255);
    //
    //return src_gray;
}
