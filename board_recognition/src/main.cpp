#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <serialConnectA255.h>
#include <game.h>

#define PI 3.14159265

using namespace cv;

std::vector<KeyPoint> recognizeKeypoints(Mat image)
{
    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;
    std::vector<KeyPoint> keypoints;

    SurfFeatureDetector detector(minHessian);
    detector.detect(image, keypoints);

    return keypoints;
}

std::vector<DMatch> matchKeypoints(Mat img_object, std::vector<KeyPoint> keypoints_object, Mat img_scene, std::vector<KeyPoint> keypoints_scene)
{
    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;

    Mat descriptors_object, descriptors_scene;

    extractor.compute(img_object, keypoints_object, descriptors_object);
    extractor.compute(img_scene, keypoints_scene, descriptors_scene);

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector<DMatch> matches;
    matcher.match(descriptors_object, descriptors_scene, matches);

    return matches;
}

std::vector<DMatch> filterGoodMatches(std::vector<DMatch> matches)
{
    double max_dist = 0;
    double min_dist = 100;
    //-- Quick calculation of max and min distances between keypoints
    for (int i = 0; i < matches.size(); i++)
    {
        double dist = matches[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector<DMatch> good_matches;

    for (int i = 0; i < matches.size(); i++)
    {
        if (matches[i].distance < 3 * min_dist)
        {
            good_matches.push_back(matches[i]);
        }
    }

    return good_matches;
}

Point2f computeReferenceImageCoordinatesOfTile(int tile)
{
    int imgWidth = 198;
    int imgHeight = 197;

    int tileWidth = 19;
    int offset = 14;

    int col = (tile - 1) % 10;       // Should return zero for all numbers that are factors of 10
    int row = ((tile - 1) / 10) + 1; // Should return 1 for all numbers <= 10, 2 for all numbers >10 <=20

    if (row % 2 == 0)
    {
        return Point2f(imgWidth - (offset + col * tileWidth), imgHeight - (offset + (row - 1) * tileWidth));
    }
    else
    {
        return Point2f(offset + col * tileWidth, imgHeight - (offset + (row - 1) * tileWidth));
    }
}

Point3f getWorldCoordinates(Point2f pixel)
{
    double f_i = 4.3;

    double p_x, p_y;
    p_x = p_y = 0.0081;

    int img_width = 640;
    int img_height = 480;

    // Relative offset of camera surface origin
    double a, b, c;
    a = 545;
    b = 92 - 15;
    c = 923;

    // Calculate surface coordinates from pixel values
    double d, e, f;
    d = (pixel.x - img_width / 2) * p_x * (f_i - c) / f_i;
    e = (pixel.y - img_height / 2) * p_y * (f_i - c) / f_i;
    f = c;

    cout << d << ", " << e << ", " << f << endl;

    return Point3f(a - e, b - d, c - f);
}

void illustrateHomography(Mat img_object, Mat img_matches, Mat H, int tilenum)
{
    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(5);
    obj_corners[0] = cvPoint(0, 0);
    obj_corners[1] = cvPoint(img_object.cols, 0);
    obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
    obj_corners[3] = cvPoint(0, img_object.rows);
    obj_corners[4] = computeReferenceImageCoordinatesOfTile(tilenum);
    std::vector<Point2f> scene_corners(5);

    perspectiveTransform(obj_corners, scene_corners, H);

    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line(img_matches, scene_corners[0] + Point2f(img_object.cols, 0), scene_corners[1] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
    line(img_matches, scene_corners[1] + Point2f(img_object.cols, 0), scene_corners[2] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
    line(img_matches, scene_corners[2] + Point2f(img_object.cols, 0), scene_corners[3] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
    line(img_matches, scene_corners[3] + Point2f(img_object.cols, 0), scene_corners[0] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
    circle(img_matches, scene_corners[4] + Point2f(img_object.cols, 0), 1, Scalar(0, 255, 0));

    //-- Show detected matches
    imshow("view", img_matches);
}

Mat computeHomography(Mat img_object, Mat img_scene, int tilenum)
{
    // Identify keypoints
    std::vector<KeyPoint>
        keypoints_object = recognizeKeypoints(img_object),
        keypoints_scene = recognizeKeypoints(img_scene);

    // Match keypoints
    std::vector<DMatch> matches = matchKeypoints(img_object, keypoints_object, img_scene, keypoints_scene);

    // Filter to only close matches
    std::vector<DMatch> good_matches = filterGoodMatches(matches);

    // Illustrate matches
    Mat img_matches;
    drawMatches(img_object, keypoints_object, img_scene, keypoints_scene,
                good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    // Compute homography from well mapped keypoints
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

    for (int i = 0; i < good_matches.size(); i++)
    {
        //-- Get the keypoints from the good matches
        obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
        scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
    }

    Mat H = findHomography(obj, scene, CV_RANSAC);

    // Illustrate homography
    illustrateHomography(img_object, img_matches, H, tilenum);
    return H;
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "image_listener");

    // Set up serial connection with robot
    serialConnectA255 armController;
    if (armController.openPort() < 0)
    {
        ROS_INFO("Something went wrong in opening the serial port. Please see the messages above.");
        return -1;
    }
    else
    {
        ROS_INFO("Port open successful");
    }

    if (armController.initialize() < 0)
    {
        cerr << "Initialization Failed" << endl;
        return -1;
    }

    // Initialize game
    game game;
    game.tile = 1;

    // Prepare callback for performing image recognition
    bool done = false;
    Mat img_object = imread("/home/me-547/Downloads/group7/board_reference_prepped.png", CV_LOAD_IMAGE_UNCHANGED);
    Mat H;
    auto imageCallback = [&game, &done, img_object, &armController, &H](const sensor_msgs::ImageConstPtr &msg) {
        if (!done)
        {
            try
            {
                H = computeHomography(img_object, cv_bridge::toCvShare(msg, "bgr8")->image, game.tile);
                cv::waitKey(30);
            }
            catch (cv_bridge::Exception &e)
            {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
            }
        }

        // Compute world coordinates
        cout << "========= Moving to tile" << game.tile << endl;
        std::vector<Point2f> ref_pixel(1);
        ref_pixel[0] = computeReferenceImageCoordinatesOfTile(game.tile);
        cout << ref_pixel[0].x << ", " << ref_pixel[0].y << endl;

        std::vector<Point2f> cam_pixel(1);
        perspectiveTransform(ref_pixel, cam_pixel, H);
        cout << cam_pixel[0].x << ", " << cam_pixel[0].y << endl;

        Point3f wc = getWorldCoordinates(cam_pixel[0]);
        cout << wc.x << ", " << wc.y << ", " << wc.z << endl;


        // Move robot
        int heightOffset = 330;
        armController.goReady();

        // double *DesTheta = new double[5];
        // DesTheta = armController.invKine(wc.x, wc.y, wc.z + 300, 0);
        int moveresult = armController.moveRobot(wc.x, wc.y, wc.z + heightOffset, 0, 90, 0);
        if (moveresult < 0)
        // if (armController.moveTheta(60, DesTheta[0], DesTheta[1], DesTheta[2], DesTheta[3], DesTheta[4]) < 0)
        {
            cerr << "Unable to move to location" << endl;
            if (moveresult == -2) {
                cerr << "Destination location unreachable. Please adjust board" << endl;
                waitKey(0);
            }
            return;
        }

        if (game.tile == 1)
        {
            if (armController.grip_open() < 0)
            {
                cerr << "Unable to Open Grip" << endl;
                return;
            }
        }

        armController.goInZ(-70);
        // armController.moveRobot(wc.x, wc.y, wc.z + heightOffset - 70, 0, 90, 0);

        if (game.tile == 1)
        {
            if (armController.grip_close() < 0)
            {
                cerr << "Unable to Close Grip" << endl;
                return;
            }
        }

        armController.goInZ(70);
        // armController.moveRobot(wc.x, wc.y, wc.z + heightOffset, 0, 90, 0);
        
        game.next();
    };

    // Listen for camera messages
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("view");

    // ros::init(argc, argv, "image_listener");
    // ros::NodeHandle nh;
    // cv::namedWindow("view");
    // cv::startWindowThread();
    // image_transport::ImageTransport it(nh);
    // image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
    // ros::spin();
    // cv::destroyWindow("view");
}