
#include <cv.h> 
#include <highgui.h>
#include <cxcore.h>
#include <iostream>
#include "ros/package.h"
#include <string>

CvCapture* capture;
int device = 0; //num of camera which shall be used

void end (void) __attribute__((destructor));

void end(){
    cvReleaseCapture(&capture);
}
 
//callback for trackbar. nothing to be done here     
void on_trackbar( int, void* )
{
}

int main(int argc, char* argv[])
{
      int height,width,step,channels;  //parameters of the image we are working on
      int i,j,k,t1min=0,t1max=0,t2min=0,t2max=0,t3min=0,t3max=0; // other variables used

      printf("Tune the threshold parameters such that in every frame the Ball is white. Then hit escp to safe your result. \nPress any key to continue...\n");
      std::cin.get();
      
     
      CvMat* threshold_matrix  = cvCreateMat(2,3,CV_32FC1);
      
      CvFileStorage* temp = cvOpenFileStorage("src/tas_group_10/follow_ball/threshold_matrix.xml",NULL,CV_STORAGE_READ);
    
    // Load the previous values of the threshold if they exist
    if(temp)
    {
        threshold_matrix = (CvMat*)cvLoad("src/tas_group_10/follow_ball/threshold_matrix.xml");
        t1min =(int) CV_MAT_ELEM(*threshold_matrix,float,0,0) ;t2min =(int) CV_MAT_ELEM(*threshold_matrix,float,0,1) ;t3min =(int) CV_MAT_ELEM(*threshold_matrix,float,0,2);
        t1max =(int) CV_MAT_ELEM(*threshold_matrix,float,1,0) ;t2max =(int) CV_MAT_ELEM(*threshold_matrix,float,1,1) ;t3max =(int) CV_MAT_ELEM(*threshold_matrix,float,1,2) ;
    }
 
    // Open capture device. 0 is /dev/video0, 1 is /dev/video1, etc.
    capture = cvCaptureFromCAM(device);
    
    if( !capture )
    {
            fprintf( stderr, "ERROR: capture is NULL \n" );
            getchar();
            return -1;
    }
    // grab an image from the capture
    IplImage* frame = cvQueryFrame( capture );
    printf("läuft noch");
    
    // Create a window in which the captured images will be presented
    cvNamedWindow( "Camera", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "HSV", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "F1", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "F2", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "F3", CV_WINDOW_AUTOSIZE );
    //cvNamedWindow( "EdgeDetection", CV_WINDOW_AUTOSIZE );
    
    // Create Trackbars
     char TrackbarName1[50]="t1min";
     char TrackbarName2[50]="t1max";
     char TrackbarName3[50]="t2min";
     char TrackbarName4[50]="t2max";
     char TrackbarName5[50]="t3min";
     char TrackbarName6[50]="t3max";
 
      cvCreateTrackbar( TrackbarName1, "F1", &t1min, 260 , NULL );
      cvCreateTrackbar( TrackbarName2, "F1", &t1max, 260,  NULL  );
      
      cvCreateTrackbar( TrackbarName3, "F2", &t2min, 260 , NULL );
      cvCreateTrackbar( TrackbarName4, "F2", &t2max, 260,  NULL  );
      
      cvCreateTrackbar( TrackbarName5, "F3", &t3min, 260 , NULL );
      cvCreateTrackbar( TrackbarName6, "F3", &t3max, 260,  NULL  );
 
   // Load threshold from the slider bars in these 2 parameters
    CvScalar hsv_min = cvScalar(t1min, t2min, t3min, 0);
    CvScalar hsv_max = cvScalar(t1max, t2max ,t3max, 0);
    
    // get the image data
      height    = frame->height;
      width     = frame->width;
      step      = frame->widthStep;
      
     // capture size - 
    CvSize size = cvSize(width,height);
    
    // Initialize different images that are going to be used in the program
    IplImage*  hsv_frame    = cvCreateImage(size, IPL_DEPTH_8U, 3); // image converted to HSV plane
    IplImage*  thresholded   = cvCreateImage(size, IPL_DEPTH_8U, 1); // final thresholded image
    IplImage*  thresholded1   = cvCreateImage(size, IPL_DEPTH_8U, 1); // Component image threshold
    IplImage*  thresholded2   = cvCreateImage(size, IPL_DEPTH_8U, 1);
    IplImage*  thresholded3   = cvCreateImage(size, IPL_DEPTH_8U, 1);
    IplImage*  filtered   = cvCreateImage(size, IPL_DEPTH_8U, 1);  //smoothed image
    
  
    while( 1 )
    {   
        // Load threshold from the slider bars in these 2 parameters
       hsv_min = cvScalar(t1min, t2min, t3min, 0);
       hsv_max = cvScalar(t1max, t2max ,t3max, 0);
    
        // Get one frame
        frame = cvQueryFrame( capture );
        
        if( !frame )
        {
                fprintf( stderr, "ERROR: frame is null...\n" );
                getchar();
                break;
        }
 
        // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
        cvCvtColor(frame, hsv_frame, CV_BGR2HSV);
        
        // Filter out colors which are out of range.
        cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);
        
        // the below lines of code is for visual purpose only remove after calibration 
        //--------------FROM HERE-----------------------------------
        //Split image into its 3 one dimensional images
        cvSplit( hsv_frame,thresholded1, thresholded2, thresholded3, NULL );
       
        // Filter out colors which are out of range.
        cvInRangeS(thresholded1,cvScalar(t1min,0,0,0) ,cvScalar(t1max,0,0,0) ,thresholded1);
        cvInRangeS(thresholded2,cvScalar(t2min,0,0,0) ,cvScalar(t2max,0,0,0) ,thresholded2);
        cvInRangeS(thresholded3,cvScalar(t3min,0,0,0) ,cvScalar(t3max,0,0,0) ,thresholded3);
        
        //-------------REMOVE OR COMMENT AFTER CALIBRATION TILL HERE ------------------

        IplConvKernel* kernelDilate = cvCreateStructuringElementEx(8, 8, 0, 0, CV_SHAPE_RECT);
        IplConvKernel* kernelErode = cvCreateStructuringElementEx(3, 3, 0, 0, CV_SHAPE_RECT);
        cvErode(thresholded, thresholded, kernelErode);
        cvErode(thresholded, thresholded, kernelErode);
        cvDilate(thresholded, thresholded, kernelDilate);
        cvDilate(thresholded, thresholded, kernelDilate);
    
        // Memory for hough circles
        CvMemStorage* storage = cvCreateMemStorage(0);
        
        // hough detector works better with some smoothing of the image
        cvSmooth( thresholded, thresholded, CV_GAUSSIAN, 9, 9 );
        
        //hough transform to detect circle
        CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2,
                                        thresholded->height/4, 100, 50, 10, 400);
 
        for (int i = 0; i < circles->total; i++)// UNDO 1 -> circles->total
        {   //get the parameters of circles detected
            if(circles->total == 0){
                continue;
            }
            float* p = (float*)cvGetSeqElem( circles, i );
            printf("Ball! x=%f y=%f r=%f\n\r",p[0],p[1],p[2] );
           
            // draw a circle with the centre and the radius obtained from the hough transform
            cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),  //plot centre
                                    2, CV_RGB(255,255,255),-1, 8, 0 );
            cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),  //plot circle
                                    cvRound(p[2]), CV_RGB(0,255,0), 2, 8, 0 );
        }
           
         /* for testing purpose you can show all the images but when done with calibration 
         only show frame to keep the screen clean  */  
           
         cvShowImage( "Camera", frame ); // Original stream with detected ball overlay
         cvShowImage( "HSV", hsv_frame); // Original stream in the HSV color space
         cvShowImage( "After Color Filtering", thresholded ); // The stream after color filtering
         cvShowImage( "F1", thresholded1 ); // individual filters
         cvShowImage( "F2", thresholded2 );
         cvShowImage( "F3", thresholded3 );
        
        //cvShowImage( "filtered", thresholded );
        
      
 
        cvReleaseMemStorage(&storage);

        //Save the threshold values before exiting
        *((float*)CV_MAT_ELEM_PTR( *threshold_matrix, 0, 0 ) ) = t1min ;*((float*)CV_MAT_ELEM_PTR( *threshold_matrix, 0, 1 ) ) = t2min ;*((float*)CV_MAT_ELEM_PTR( *threshold_matrix, 0, 2 ) ) = t3min ;
        *((float*)CV_MAT_ELEM_PTR( *threshold_matrix, 1, 0 ) ) = t1max ;*((float*)CV_MAT_ELEM_PTR( *threshold_matrix, 1, 1 ) ) = t2max ;*((float*)CV_MAT_ELEM_PTR( *threshold_matrix, 1, 2 ) ) = t3max ;
        cvSave("src/tas_group_10/follow_ball/threshold_matrix.xml",threshold_matrix);
 
      
        //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
        //remove higher bits using AND operator
        if( (cvWaitKey(10) & 255) == 27 ) break;
    }
     
     //Save the threshold values before exiting
     *((float*)CV_MAT_ELEM_PTR( *threshold_matrix, 0, 0 ) ) = t1min ;*((float*)CV_MAT_ELEM_PTR( *threshold_matrix, 0, 1 ) ) = t2min ;*((float*)CV_MAT_ELEM_PTR( *threshold_matrix, 0, 2 ) ) = t3min ;
     *((float*)CV_MAT_ELEM_PTR( *threshold_matrix, 1, 0 ) ) = t1max ;*((float*)CV_MAT_ELEM_PTR( *threshold_matrix, 1, 1 ) ) = t2max ;*((float*)CV_MAT_ELEM_PTR( *threshold_matrix, 1, 2 ) ) = t3max ;
     cvSave("src/tas_group_10/follow_ball/threshold_matrix.xml",threshold_matrix);
     
     
     // Release the capture device housekeeping
     cvReleaseCapture( &capture );
     cvDestroyWindow( "mywindow" );
     return 0;
   }

