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
    bool blinkOutput = false;

    /** Get the next frame **/
    if (_left_frame.read(leftFrame) == RTT::NewData)
    {
        if( clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC )
        {

            std::cout<<"Time: "<<clock() - prevTimestamp<<"\n";

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

            /** Convert to OpenCv format **/
            view_left = frame_helper::FrameHelper::convertToCvMat(leftImage);

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
                    base::samples::frame::Frame leftImageGray;
                    leftImageGray.init(*leftFrame, false);
                    leftImageGray.frame_mode = base::samples::frame::MODE_GRAYSCALE;

                    try
                    {
                        frameHelper.convertColor (*leftFrame, leftImageGray);

                        std::cout<<"Frame Mode: "<<leftFrame->getFrameMode()<<"\n";
                        std::cout<<"Frame Mode(Gray): "<<leftImageGray.getFrameMode()<<"\n";

                        view_leftGray = frame_helper::FrameHelper::convertToCvMat(leftImageGray);

                        cornerSubPix( view_leftGray, pointBuf, Size(11,11),
                            Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

                    }
                    catch(const std::runtime_error& error)
                    {
                        std::cout<<"[CATCH] "<<error.what()<<"\n";
                    }

                }

                // Draw the corners.
                drawChessboardCorners( view_left, s.boardSize, Mat(pointBuf), found );
            }

            imagePoints.push_back(pointBuf);
            prevTimestamp = clock();

            if( blinkOutput )
                bitwise_not(view_left, view_left);

            /** Show image and check for input commands **/
            imshow("Image View", view_left);
        }
    }

    /** Exit the update Hook **/
    char key = (char)waitKey(s.delay);
    const char ESC_KEY = 27;
    if( key  == ESC_KEY )
    {

        std::cout<<"Running Calibration .... ";
        runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints);
        std::cout<<"[DONE] ";

      //  /** If undistorted output is selected in the configuration file **/
      //  if(s.showUndistorsed )
      //  {
      //      Mat temp = view_left.clone();
      //      undistort(temp, view_left, cameraMatrix, distCoeffs);
      //  }

      //  imshow("Image View Undist", view_left);

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



