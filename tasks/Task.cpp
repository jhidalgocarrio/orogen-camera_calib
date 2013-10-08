/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace camera_calib;
using namespace cv;

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

    /** Set time stamp **/
    prevTimestamp = 0;

    /** Read the settings **/
    s.boardSize.width = _BoardSize_Width.value();
    s.boardSize.height = _BoardSize_Height.value();
    s.squareSize = _Square_Size.value();
    s.patternToUse = _Calibrate_Pattern.value();
    s.flipVertical = _Input_FlipAroundHorizontalAxis.value();
    s.delay = _Input_Delay.value();
    s.aspectRatio = _Calibrate_FixAspectRatio.value();
    s.calibZeroTangentDist = _Calibrate_AssumeZeroTangentialDistortion.value();
    s.calibFixPrincipalPoint = _Calibrate_FixPrincipalPointAtTheCenter.value();
    s.bwritePoints = _Write_DetectedFeaturePoints.value();
    s.bwriteExtrinsics = _Write_extrinsicParameters.value();
    s.showUndistorsed = _Show_UndistortedImage.value();
    s.outputFileName = _Write_outputFileName.value();

    s.interprate();

    if (!s.goodInput)
    {
        std::cout << "Invalid input detected. Application stopping. " << std::endl;
        return false;
    }

    /** Set the output images **/
    ::base::samples::frame::Frame *outFrameLeft = new ::base::samples::frame::Frame();

    frameLeft_out.reset(outFrameLeft);
    outFrameLeft = NULL;

    ::base::samples::frame::Frame *outFrameRight = new ::base::samples::frame::Frame();

    frameRight_out.reset(outFrameRight);
    outFrameRight = NULL;

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
    std::vector<cv::Point2f> pointLeftBuf, pointRightBuf;
    bool foundLeft = false; bool foundRight = false;

    std::cout<<"[CAMERA_CALIB] Time: "<<clock() - prevTimestamp<<"\n";

    /** Get the next frame of camera one **/
    if (_left_frame.readNewest(leftFrame, false) == RTT::NewData)
    {
        base::samples::frame::Frame leftImage;

        /** Whatever format it is, try to convert to RGB unless it is GRAYSCALE **/
        if (leftFrame->getFrameMode() != base::samples::frame::MODE_GRAYSCALE)
        {
            leftImage.frame_mode = base::samples::frame::MODE_BGR;
            leftImage.setDataDepth(leftFrame->getDataDepth());
            try
            {
                frameHelper.convertColor (*leftFrame, leftImage);
            }
            catch(const std::runtime_error& error)
            {
                std::cout<<"[CATCH] "<<error.what()<<"\n";
                leftImage = *leftFrame;
            }
        }
        else
            leftImage = *leftFrame;

        /** Output image to show it in a port **/
        ::base::samples::frame::Frame *frame_ptr = frameLeft_out.write_access();

        /** Find the pattern **/
        foundLeft = this->findPattern(s, leftImage, *frame_ptr, pointLeftBuf);

        /** Write to the out port **/
        frame_ptr->time = leftFrame->time;
        frameLeft_out.reset(frame_ptr);
        _left_frame_out.write(frameLeft_out);

    }

    /** Get the next frame of camera two **/
    if (_right_frame.readNewest(rightFrame, false) == RTT::NewData)
    {
        base::samples::frame::Frame rightImage;

        /** Whatever format it is, try to convert to RGB unless it is GRAYSCALE **/
        if (rightFrame->getFrameMode() != base::samples::frame::MODE_GRAYSCALE)
        {
            rightImage.frame_mode = base::samples::frame::MODE_BGR;
            rightImage.setDataDepth(rightFrame->getDataDepth());
            try
            {
                frameHelper.convertColor (*rightFrame, rightImage);
            }
            catch(const std::runtime_error& error)
            {
                std::cout<<"[CATCH] "<<error.what()<<"\n";
                rightImage = *rightFrame;
            }
        }
        else
            rightImage = *rightFrame;

        /** Output image to show it in a port **/
        ::base::samples::frame::Frame *frame_ptr = frameRight_out.write_access();

        /** Find the pattern **/
        foundRight = this->findPattern(s, rightImage, *frame_ptr, pointRightBuf);

        /** Write to the out port **/
        frame_ptr->time = rightFrame->time;
        frameRight_out.reset(frame_ptr);
        _right_frame_out.write(frameRight_out);

    }

    prevTimestamp = clock();


    /** Stereo calibration **/
    if (_right_frame.connected() && _left_frame.connected())
    {
        /** Store the points in the image plane only if both cameras found the pattern **/
        if (foundLeft && foundRight)
        {
            imagePointsLeft.push_back(pointLeftBuf);
            imagePointsRight.push_back(pointRightBuf);
            std::cout<<"FOUND at ["<< leftFrame->time.toSeconds() <<"] ["<< leftFrame->time.toSeconds()<<"\n";
            std::cout<<"Time (msec) are Left: "<<leftFrame->time.toMilliseconds()<<" Right:"<<rightFrame->time.toMilliseconds()<<"\n";
        }
    }
    else if (_right_frame.connected()) // Mono calibration first camera connected
    {
    }
    else if (_left_frame.connected()) // Mono calibration second camera connected
    {

    }

    /** Exit the update Hook **/
    char key = (char)waitKey(s.delay);
    const char ESC_KEY = 27;
    if( key  == ESC_KEY )
    {

        std::cout<<"Running Calibration .... ";
        runCalibrationAndSave(s, imageSize,  cameraMatrixLeft, distCoeffsLeft, imagePointsLeft);
        std::cout<<"[DONE] ";

      //  /** If undistorted output is selected in the configuration file **/
      //  if(s.showUndistorsed )
      //  {
      //      Mat temp = view.clone();
      //      undistort(temp, view, cameraMatrix, distCoeffs);
      //  }

      //  imshow("Image View Undist", view);

        return;
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

bool Task::findPattern (const Settings &s, const base::samples::frame::Frame &imageInput,
                        base::samples::frame::Frame &imageOutput,
                        std::vector<cv::Point2f> &pointBuf,
                        const bool blinkOutput)
{
    /** Convert to OpenCv format **/
    cv::Mat view = frame_helper::FrameHelper::convertToCvMat(imageInput);

    imageSize = view.size();  // Format input image.
    if( s.flipVertical )    flip( view, view, 0 );

    /** Find the pattern **/
    bool found;
    switch( s.calibrationPattern ) // Find feature points on the input format
    {
        case Settings::CHESSBOARD:
            found = findChessboardCorners( view, s.boardSize, pointBuf,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
            break;
        case Settings::CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf );
            break;
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );
            break;
        default:
            found = false;
            break;
    }

    /** If the pattern is found **/
    if (found)
    {
        /** Improve the found corners' coordinate accuracy for chessboard **/
        if( s.calibrationPattern == Settings::CHESSBOARD)
        {
            cv::Mat viewGray;
            cvtColor(view, viewGray, CV_BGR2GRAY);
            cornerSubPix( viewGray, pointBuf, Size(11,11),
                Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
        }

        /** Draw the corners. **/
        drawChessboardCorners( view, s.boardSize, Mat(pointBuf), found );
    }

    /** Inverse of bits array **/
    if( blinkOutput )
        bitwise_not(view, view);

    /** Copy to Frame type **/
    frame_helper::FrameHelper::copyMatToFrame (view, imageOutput);

    return found;
}
double Task::computeReprojectionErrors( const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                         const std::vector<std::vector<cv::Point2f> >& imagePoints,
                                         const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                         const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs,
                                         std::vector<float>& perViewErrors)
{
    std::vector<cv::Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        cv::projectPoints( cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                       distCoeffs, imagePoints2);
        err = norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), CV_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

void Task::calcBoardCornerPositions(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
    corners.clear();

    switch(patternType)
    {
    case Settings::CHESSBOARD:
    case Settings::CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(cv::Point3f(float( j*squareSize ), float( i*squareSize ), 0));
        break;

    case Settings::ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(cv::Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
        break;
    default:
        break;
    }
}


bool Task::runCalibration (Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                            std::vector<std::vector<cv::Point2f> > imagePoints, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
                            std::vector<float>& reprojErrs,  double& totalAvgErr)
{

    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = 1.0;

    distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

    std::vector<std::vector<cv::Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, s.flag|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

    std::cout << "Re-projection error reported by calibrateCamera: "<< rms << std::endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                             rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}

bool Task::runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,vector<vector<Point2f> > imagePoints )
{
    std::vector<Mat> rvecs, tvecs;
    std::vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = this->runCalibration(s,imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
                             reprojErrs, totalAvgErr);
    std::cout << (ok ? "Calibration succeeded" : "Calibration failed")
        << ". avg re projection error = "  << totalAvgErr ;

/*    if( ok )
        this->saveCameraParams( s, imageSize, cameraMatrix, distCoeffs, rvecs ,tvecs, reprojErrs,
                            imagePoints, totalAvgErr);*/
    return ok;
}



