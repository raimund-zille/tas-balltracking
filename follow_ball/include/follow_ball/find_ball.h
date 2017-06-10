#ifndef FIND_BALL_H
#define FIND_BALL_H

#include <ros/ros.h>
#include <iostream>
#include "math.h"

#include <cv.h>
#include <highgui.h>
#include <cxcore.h>

/// how many last circles should be stored
#define SAVE_CIRCLES 20
/// if used consistenCircle, how many of the same circle shall be seen latley
#define MIN_CIRCLES 2
#define SAME_CIRCLE_THRESHOLD 10
#define MIN_CIRCLES_THRESHOLD 50

struct Circle{
    double r; ///radius
    double x; ///x-koordiante in picture
    double y; ///y-koordinate in picture
    Circle() : x(0), y(0), r(0){}
    struct Circle& operator +=(const Circle& rhs) { x += rhs.x; y += rhs.y; r += rhs.y; return *this; }
    struct Circle& operator /=(const size_t& rhs) { x /= rhs; y /= rhs; r /= rhs; return *this; }
};

class FindBall{

public:
    FindBall();
    FindBall(int camera);
    ~FindBall();

    /**
     * @brief getMostLikelyCircle trys to find the most likly circle of all found (doesnt work well)
     * @param show if there should be a image printed on the screen
     * @return most likley circle
     */
    Circle getMostLikelyCircle(bool show = false);

    /**
     * @brief consistentCircle checks if the last MIN_CIRCLES are the same
     * @param show if there should be a image printed on the screen
     * @return returns last circle if they are the same
     */
    Circle consistentCircle(bool show = false);

    /**
     * @brief getAllCircles finds all circle in the image
     * @param show if there should be a image printed on the screen
     * @return all circles which were found
     */
    std::vector<Circle> getAllCircles(bool show = false);

    /**
     * @brief getMeanFromCluster used for getMostLikelyCircle()
     * @param cluster
     * @return
     */
    Circle getMeanFromCluster(const std::vector<Circle> &cluster);

    /**
     * @brief drawCircle prints circle in the image on the screen
     * @param circle will be printed
     */
    void drawCircle(Circle circle);

    /**
     * @brief getWidth
     * @return width of the camera
     */
    int getWidth();

    /**
     * @brief getHeight
     * @return height of the camera
     */
    int getHeight();

    /**
     * @brief camera_found_ false if no camera found
     */
    bool camera_found_;

private:
    /**
     * @brief start init camera
     * @param camera which camera should be open 0 = video0, 1 = video1 ...
     */
    void start(int camera = 0);
    std::vector<std::vector<Circle> > clusterCircle(std::vector<Circle> circles);
    void circle_idx_plus();
    void circle_idx_minus();
    double dist(Circle &c1, Circle &c2);
    double distWithoutR(Circle &c1, Circle &c2);
    CvScalar hsv_min_;
    CvScalar hsv_max_;
    CvCapture* capture_;
    bool threshold_matrix_found_;
    bool collected_circles_;
    std::vector<Circle> last_circles_;
    int last_circle_idx_;
};
#endif
