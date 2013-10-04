/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace camera_calib;
using namespace cv;

static void help()
{
    std::cout <<  "This is a camera calibration sample." << std::endl
         <<  "Usage: calibration configurationFile"  << std::endl
         <<  "Near the sample file you'll find the configuration file, which has detailed help of "
             "how to edit it.  It may be any OpenCV supported file format XML/YAML." << std::endl;
}


/** TASK START HERE **/
Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /** Help Message **/
    help();
    const string inputSettingsFile = _config_file.value();

    prevTimestamp = 0;

    /** Read the settings **/
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        std::cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << std::endl;
        return false;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file

    if (!s.goodInput)
    {
        std::cout << "Invalid input detected. Application stopping. " << std::endl;
        return false;
    }

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> leftFrame, rightFrame;
    Mat view_left;
    bool blinkOutput = false;

    /** Get the next frame **/
    if (_left_frame.read(leftFrame) == RTT::NewData)
    {
        base::samples::frame::Frame leftImage = *leftFrame;
//        view_left = frameHelper.convertToCvMat(leftFrame);
    }

    imageSize = view_left.size();  // Format input image.
    if( s.flipVertical )    flip( view_left, view_left, 0 );

    /** Find the pattern **/
    std::vector<cv::Point2f> pointBuf;
    bool found;
    switch( s.calibrationPattern ) // Find feature points on the input format
    {
        case Settings::CHESSBOARD:
            found = findChessboardCorners( view_left, s.boardSize, pointBuf,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
            break;
        case Settings::CIRCLES_GRID:
            found = findCirclesGrid( view_left, s.boardSize, pointBuf );
            break;
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid( view_left, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );
            break;
        default:
            found = false;
            break;
    }

    /** If the pattern is found **/
    if (found)
    {
        // improve the found corners' coordinate accuracy for chessboard
        if( s.calibrationPattern == Settings::CHESSBOARD)
        {
            Mat view_leftGray;
            cvtColor(view_left, view_leftGray, CV_BGR2GRAY);
            cornerSubPix( view_leftGray, pointBuf, Size(11,11),
                Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
        }

        if( clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC )
        {
            imagePoints.push_back(pointBuf);
            prevTimestamp = clock();
        }

        // Draw the corners.
        drawChessboardCorners( view_left, s.boardSize, Mat(pointBuf), found );
    }

    if( blinkOutput )
        bitwise_not(view_left, view_left);

    /** If undistorted output is selected in the configuration file **/
    if(s.showUndistorsed )
    {
        Mat temp = view_left.clone();
        undistort(temp, view_left, cameraMatrix, distCoeffs);
    }

    /** Show image and check for input commands **/
    imshow("Image View", view_left);
    char key = (char)waitKey(s.delay);

    const char ESC_KEY = 27;
    if( key  == ESC_KEY )
        this->stop();

}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();

//    runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints);
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
