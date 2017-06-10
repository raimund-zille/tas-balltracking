#include "../include/follow_ball/find_ball.h"

FindBall::FindBall(){
    start();
}

FindBall::FindBall(int camera){
    start(camera);
}

FindBall::~FindBall(){
    printf("destructor");
    cvReleaseCapture(&capture_);
    //delete capture_;
}

void FindBall::start(int camera){
    CvMat* threshold_matrix  = cvCreateMat(2,3,CV_32FC1);
    camera_found_ = true;
    threshold_matrix_found_ = true;

    last_circle_idx_ = -1;
    Circle dummy;
    for(int i = 0; i < SAVE_CIRCLES; i++){
        last_circles_.push_back(dummy);
    }
    collected_circles_ = false;

    CvFileStorage* temp = cvOpenFileStorage("src/tas_group_10/follow_ball/threshold_matrix.xml",NULL,CV_STORAGE_READ);
    int t1min, t2min, t3min, t1max, t2max, t3max;
    if(temp){
        threshold_matrix = (CvMat*)cvLoad("src/tas_group_10/follow_ball/threshold_matrix.xml");
        t1min =(int) CV_MAT_ELEM(*threshold_matrix,float,0,0) ;t2min =(int) CV_MAT_ELEM(*threshold_matrix,float,0,1) ;t3min =(int) CV_MAT_ELEM(*threshold_matrix,float,0,2);
        t1max =(int) CV_MAT_ELEM(*threshold_matrix,float,1,0) ;t2max =(int) CV_MAT_ELEM(*threshold_matrix,float,1,1) ;t3max =(int) CV_MAT_ELEM(*threshold_matrix,float,1,2);
    }else{
        ROS_WARN("Calibrate your parameters first! Run: <rosrun follow_ball calibrate_params>!");
        hsv_min_ = cvScalar(0,0,0,0);
        hsv_max_ = cvScalar(0,0,0,0);
        threshold_matrix_found_ = false;
    }

    //load tresholds
    hsv_min_ = cvScalar(t1min, t2min, t3min, 0);
    hsv_max_ = cvScalar(t1max, t2max ,t3max, 0);

    //Open capture device. 0 for /dev/video0
    capture_ = cvCaptureFromCAM( camera );
    //cvSetCaptureProperty( capture_, CV_CAP_PROP_FRAME_WIDTH, 640 );

    //cvSetCaptureProperty( capture_, CV_CAP_PROP_FRAME_HEIGHT, 480 );


    if( !capture_ )
    {
         ROS_WARN("No camera found!");
         camera_found_ = false;
    }
}

int FindBall::getWidth(){
    return cvQueryFrame(capture_)->width;
}

int FindBall::getHeight(){
    return cvQueryFrame(capture_)->height;
}

void FindBall::drawCircle(Circle circle){
    if(!camera_found_){
        ROS_WARN("No camera found!");
    }
    IplImage* frame = cvQueryFrame(capture_); //raw image
    if(!frame){
        ROS_WARN("ERROR: No new frame...\n");
    }

    if(circle.r != 0){
        cvCircle( frame, cvPoint(cvRound(circle.x),circle.y),  //plot centre
                                2, CV_RGB(255,255,255),-1, 8, 0 );
        cvCircle( frame, cvPoint(cvRound(circle.x),cvRound(circle.y)),  //plot circle
                                cvRound(circle.r), CV_RGB(255,0,0), 2, 8, 0 ); //draw first cirlce yellow
    }
    cvNamedWindow( "Camera", CV_WINDOW_AUTOSIZE );
    cvShowImage("Camera", frame);
    cvWaitKey(1);
}


std::vector<Circle> FindBall::getAllCircles(bool show){

    std::vector<Circle> returnArray;
    if(!camera_found_){
        ROS_WARN("No camera found!");
        return returnArray;
    }
    if(!threshold_matrix_found_){
        ROS_WARN("Found no threshold matrix to filter the image.\nRun <rosrun follow_ball calibrate_params> first!");
    }
    /* FILTER IMAGE */
    IplImage* frame = cvQueryFrame(capture_); //raw image
    CvSize size = cvSize(frame->width, frame->height);
    IplImage* hsv_frame = cvCreateImage(size, IPL_DEPTH_8U, 3); //hsv image
    IplImage*  thresholded   = cvCreateImage(size, IPL_DEPTH_8U, 1); //thresholded image



    if(!frame){
        ROS_WARN("ERROR: No new frame...\n");
        return returnArray;
    }

    //convert to hsv for better color detection
    cvCvtColor(frame, hsv_frame, CV_BGR2HSV);
    cvInRangeS(hsv_frame, hsv_min_, hsv_max_, thresholded);


    /* FIND CIRCLES */
    // Memory for hough circles
    CvMemStorage* storage = cvCreateMemStorage(0);

    // hough detector works better with some smoothing of the image
    cvSmooth( thresholded, thresholded, CV_GAUSSIAN, 9, 9 );

    IplConvKernel* kernelDilate = cvCreateStructuringElementEx(8, 8, 0, 0, CV_SHAPE_RECT);
    IplConvKernel* kernelErode = cvCreateStructuringElementEx(3, 3, 0, 0, CV_SHAPE_RECT);
    cvErode(thresholded, thresholded, kernelErode);
    cvErode(thresholded, thresholded, kernelErode);
    cvDilate(thresholded, thresholded, kernelDilate);
    cvDilate(thresholded, thresholded, kernelDilate);
    if(show){
    	cvNamedWindow( "CameraAllThresholded", CV_WINDOW_AUTOSIZE );
    	cvShowImage("CameraAllThresholded", thresholded);
    	cvWaitKey(1);
    }

    //hough transform to detect circle
    CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2,
                                    thresholded->height/4, 100, 50, 10, 400);

    for (int i = 0; i < circles->total; i++)// UNDO 1 -> circles->total
    {   //get the parameters of circles detected
        float* p = (float*)cvGetSeqElem( circles, i );
        Circle curr;
        curr.x = p[0]; //x
        curr.y = p[1]; //y
        curr.r = p[2]; //r

        returnArray.push_back(curr);
        ROS_INFO("Ball! x=%f y=%f r=%f\n\r",p[0],p[1],p[2] );

        if(show){


            // draw a circle with the centre and the radius obtained from the hough transform
            cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),  //plot centre
                                    2, CV_RGB(255,255,255),-1, 8, 0 );
            if(i == 0){
                cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),  //plot circle
                                    cvRound(p[2]), CV_RGB(255,255,0), 2, 8, 0 ); //draw first cirlce yellow
            }else{
                cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),  //plot circle
                                    cvRound(p[2]), CV_RGB(0,255,0), 2, 8, 0 );
            }
        }
    }
    if(show){
        cvNamedWindow( "CameraAll", CV_WINDOW_AUTOSIZE );
        cvShowImage("Camera", frame);
        cvWaitKey(1);
    }

    if(!returnArray.empty()){
        ROS_INFO("Found circle!");
        circle_idx_plus();
        last_circles_[last_circle_idx_] = returnArray[0];
    }
#if 1
    else{
        circle_idx_plus();
        Circle dummy;
        last_circles_[last_circle_idx_] = dummy;
    }
#endif
    return returnArray;
}

void FindBall::circle_idx_plus(){
    last_circle_idx_++;
    if(last_circle_idx_ >= SAVE_CIRCLES){
        collected_circles_ = true;
        last_circle_idx_ = 0;
    }
}

double FindBall::dist(Circle &c1, Circle &c2){
    double x_diff = c1.x - c2.x;
    double y_diff = c1.y - c2.y;
    double r_diff = c1.r - c2.r;
    return sqrt(pow(x_diff,2)+pow(y_diff,2)+pow(r_diff,2));
}

double FindBall::distWithoutR(Circle &c1, Circle &c2){
    double x_diff = c1.x - c2.x;
    double y_diff = c1.y - c2.y;
   return sqrt(pow(x_diff,2)+pow(y_diff,2));
}

Circle FindBall::getMostLikelyCircle(bool show){
    getAllCircles();
    Circle mostLikly;
    if(!collected_circles_){
        //not enough data
        ROS_INFO_STREAM("Not enough circles!");
        return mostLikly;
    }
    //returns Circle.r = 0 if no convincing circle was found
    std::vector<std::vector< Circle> > clusters = clusterCircle(last_circles_);

    int max_count = 0;
    int max_count_idx = 0;

    for(size_t i = 0; i < clusters.size(); i++){
        int curr_count = clusters[i].size();
        if(curr_count>max_count){
            max_count = curr_count;
            max_count_idx = i;
        }
    }

    ROS_INFO_STREAM("MostLikely count: "<< max_count);
    mostLikly = getMeanFromCluster(clusters[max_count_idx]);
    if(show){
        drawCircle(mostLikly);
    }

    return mostLikly;
}

Circle FindBall::consistentCircle(bool show){
    getAllCircles();
    Circle curr = last_circles_[last_circle_idx_];
    double dist = 0;
    for(int i = 0; i < MIN_CIRCLES; i++){
        //calculate error
        circle_idx_minus();
        Circle curr_m1 = last_circles_[last_circle_idx_];
        dist += distWithoutR(curr, curr_m1);
        curr = curr_m1;
    }
    for(int i = 0; i < MIN_CIRCLES; i++){
        //set index back to what it was before
        circle_idx_plus();
    }

    Circle returnCircle;
    ROS_INFO_STREAM("Dist: "<<dist);
    if(dist < MIN_CIRCLES_THRESHOLD){
        returnCircle = last_circles_[last_circle_idx_];
    }else{
        ROS_INFO_STREAM("no consistent Circle!");
    }
    if(show){
        drawCircle(returnCircle);
    }
    //return last_circles_[last_circle_idx_];
    return returnCircle;
}

void FindBall::circle_idx_minus(){
    last_circle_idx_--;
    if(last_circle_idx_<0){
        last_circle_idx_ = SAVE_CIRCLES -1;
    }
}

std::vector<std::vector<Circle> > FindBall::clusterCircle(std::vector<Circle> circles){
    //get circle
    //get all circles near that circle
    //continue till no circle are left ofer
    std::vector<std::vector< Circle> > returnCluster;
    int cluster_idx = 0;
    while(!circles.empty()){
        Circle circle = circles[circles.size()];
        circles.pop_back();
        std::vector<Circle> currCluster;
        currCluster.push_back(circle);
        for(int i = 0; i < circles.size(); i++){
            if(dist(circle, circles[i])<SAME_CIRCLE_THRESHOLD){
                //same circle
                currCluster.push_back(circles[i]);
                circles.erase(circles.begin()+i);
            }
        }
        cluster_idx++;
        returnCluster.push_back(currCluster);
    }
    return returnCluster;
}

Circle FindBall::getMeanFromCluster(const std::vector<Circle> &cluster){
    Circle mean;
    for(size_t i = 0; i < cluster.size(); i++){
        mean += cluster[i];
    }
    mean /= cluster.size();
    return mean;
}
