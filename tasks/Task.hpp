/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CAMERA_CALIB_TASK_TASK_HPP
#define CAMERA_CALIB_TASK_TASK_HPP

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "frame_helper/FrameHelper.h" /** Rock lib for manipulate frames **/

#include "camera_calib/TaskBase.hpp"

namespace camera_calib
{
    class Settings
    {
    public:
        Settings() : goodInput(false) {}
        enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
        enum InputType {INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST};

        void interprate()
        {
            goodInput = true;
            if (boardSize.width <= 0 || boardSize.height <= 0)
            {
                std::cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << std::endl;
                goodInput = false;
            }
            if (squareSize <= 10e-6)
            {
                std::cerr << "Invalid square size " << squareSize << std::endl;
                goodInput = false;
            }
                        flag = 0;
            if(calibFixPrincipalPoint) flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
            if(calibZeroTangentDist)   flag |= CV_CALIB_ZERO_TANGENT_DIST;
            if(aspectRatio)            flag |= CV_CALIB_FIX_ASPECT_RATIO;


            calibrationPattern = NOT_EXISTING;
            if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
            if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
            if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
            if (calibrationPattern == NOT_EXISTING)
            {
                std::cerr << " Inexistent camera calibration mode: " << patternToUse << std::endl;
                goodInput = false;
            }
        }

    public:
        cv::Size boardSize;        // The size of the board -> Number of items by width and height
        Pattern calibrationPattern;// One of the Chessboard, circles, or asymmetric circle pattern
        float squareSize;          // The size of a square in your defined unit (point, millimeter,etc).
        float aspectRatio;         // The aspect ratio
        int delay;                 // Delay between frames
        bool bwritePoints;         //  Write detected feature points
        bool bwriteExtrinsics;     // Write extrinsic parameters
        bool calibZeroTangentDist; // Assume zero tangential distortion
        bool calibFixPrincipalPoint;// Fix the principal point at the center
        bool flipVertical;          // Flip the captured images around the horizontal axis
        std::string outputFileName; // The name of the file where to write
        bool showUndistorsed;       // Show undistorted images after calibration

        bool goodInput;
        int flag;

        std::string patternToUse;


    };


    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare a new task context (i.e., a component)

The corresponding C++ class can be edited in tasks/Task.hpp and
tasks/Task.cpp, and will be put in the camera_calib namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','camera_calib::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

        /** General setting file **/
        Settings s;
        cv::Mat view_left, view_right; //! Stream image in OpenCV format
        frame_helper::FrameHelper frameHelper; /** Frame helper **/
        std::vector<std::vector<cv::Point2f> > imagePoints;
        cv::Mat cameraMatrix, distCoeffs;
        cv::Size imageSize;
        clock_t prevTimestamp;

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "camera_calib::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        bool runCalibrationAndSave(Settings& s, cv::Size imageSize, cv::Mat&
                cameraMatrix, cv::Mat& distCoeffs,
                std::vector<std::vector<cv::Point2f> > imagePoints);

    private:
        void calcBoardCornerPositions(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/);

        double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                         const std::vector<std::vector<cv::Point2f> >& imagePoints,
                                         const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                         const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs,
                                         std::vector<float>& perViewErrors);

        bool runCalibration(Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                            std::vector<std::vector<cv::Point2f> > imagePoints, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
                            std::vector<float>& reprojErrs,  double& totalAvgErr);

    };

}

#endif

