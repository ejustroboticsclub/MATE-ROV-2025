///////////////////////////////////////////////////////////////////////////
//
// This sample code receives a GStreamer UDP stream (side-by-side stereo),
// processes the left image for stereo rectification and depth calculation,
// and displays a single window ("Video Stream").
// You can click on the stream to select two points; the code draws markers and
// a connecting line, then calculates and displays the 3D Euclidean distance
// between the two points. The distance calculation is improved by:
//   1) Applying a bilateral filter on the disparity map to reduce noise,
//   2) Smoothing the instantaneous measurements using a sliding window moving average.
//
// Note: Adjust the calibration file ("SN31223474.conf") and parameter settings as needed.
//
///////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>

// Include your calibration and stereo utility headers.
#include "calibration.hpp"
#include "stopwatch.hpp"
#include "stereo.hpp"
#include "ocv_display.hpp"

// Global container to store the selected points.
std::vector<cv::Point> g_selectedPoints;

// Global depth map (updated each frame).
cv::Mat g_depthMap;

// Global intrinsic parameters (from the left camera) for 3D reconstruction.
double g_fx, g_fy, g_cx, g_cy;

// Global variables for temporal smoothing of distance measurements.
std::vector<double> g_distanceBuffer;
double g_smoothedDistance_cm = 0.0;  // Smoothed distance (in cm)

//-----------------------------------------------------------------
// Mouse callback: on left-button click, store the clicked point.
// If already two points are stored, clear them to start fresh.
//-----------------------------------------------------------------
void onMouse(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        if (g_selectedPoints.size() >= 2)
            g_selectedPoints.clear();  // Reset if already two points.
        g_selectedPoints.push_back(cv::Point(x, y));
        if (g_selectedPoints.size() == 2)
        {
            std::cout << "Two points selected: ("
                      << g_selectedPoints[0].x << ", " << g_selectedPoints[0].y
                      << ") and ("
                      << g_selectedPoints[1].x << ", " << g_selectedPoints[1].y
                      << ")" << std::endl;
        }
    }
}

int main(int argc, char *argv[])
{
    // -------------------------------------------------------------
    // 1. Set up GStreamer Pipeline for the UDP Video Stream.
    // -------------------------------------------------------------
    // This pipeline receives an H264-encoded RTP stream on UDP port 5000.
    std::string pipeline =
        "udpsrc port=5000 ! "
        "application/x-rtp, encoding-name=H264, payload=96 ! "
        "rtph264depay ! avdec_h264 ! videoconvert ! appsink";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened())
    {
        std::cerr << "Error: Unable to open the camera stream." << std::endl;
        return -1;
    }

    // -------------------------------------------------------------
    // 2. Retrieve an initial frame and determine image dimensions.
    // -------------------------------------------------------------
    cv::Mat frame;
    if (!cap.read(frame))
    {
        std::cerr << "Error: Unable to read the first frame from the stream." << std::endl;
        return -1;
    }
    int frame_width = frame.cols;
    int frame_height = frame.rows;
    // Assuming a side-by-side stereo image: left image occupies half the width.
    int left_width = frame_width / 2;
    std::cout << "Stream opened. Frame size: " << frame_width << "x" << frame_height << std::endl;

    // -------------------------------------------------------------
    // 3. Load Calibration Data.
    // -------------------------------------------------------------
    // The calibration file is assumed to be stored locally.
    std::string calibration_file = "SN31223474.conf";
    cv::Mat map_left_x, map_left_y, map_right_x, map_right_y;
    cv::Mat cameraMatrix_left, cameraMatrix_right;
    double baseline = 0.0;

    // Initialize calibration for the left image.
    sl_oc::tools::initCalibration(calibration_file, cv::Size(left_width, frame_height),
                                  map_left_x, map_left_y, map_right_x, map_right_y,
                                  cameraMatrix_left, cameraMatrix_right, &baseline);

    // Extract intrinsic parameters from the left camera matrix.
    g_fx = cameraMatrix_left.at<double>(0, 0);
    g_fy = cameraMatrix_left.at<double>(1, 1);
    g_cx = cameraMatrix_left.at<double>(0, 2);
    g_cy = cameraMatrix_left.at<double>(1, 2);

    std::cout << "Camera Matrix L:\n" << cameraMatrix_left << std::endl;
    std::cout << "Camera Matrix R:\n" << cameraMatrix_right << std::endl;
    std::cout << "Baseline: " << baseline << " mm" << std::endl;

    // -------------------------------------------------------------
    // 4. Initialize the Stereo Matcher (using StereoSGBM).
    // -------------------------------------------------------------
    sl_oc::tools::StereoSgbmPar stereoPar;
    if (!stereoPar.load())
    {
        stereoPar.save(); // Save default parameters if none are found.
    }
    cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(
            stereoPar.minDisparity, stereoPar.numDisparities, stereoPar.blockSize);
    left_matcher->setMinDisparity(stereoPar.minDisparity);
    left_matcher->setNumDisparities(stereoPar.numDisparities);
    left_matcher->setBlockSize(stereoPar.blockSize);
    left_matcher->setP1(stereoPar.P1);
    left_matcher->setP2(stereoPar.P2);
    left_matcher->setDisp12MaxDiff(stereoPar.disp12MaxDiff);
    left_matcher->setMode(stereoPar.mode);
    left_matcher->setPreFilterCap(stereoPar.preFilterCap);
    left_matcher->setUniquenessRatio(stereoPar.uniquenessRatio);
    left_matcher->setSpeckleWindowSize(stereoPar.speckleWindowSize);
    left_matcher->setSpeckleRange(stereoPar.speckleRange);
    stereoPar.print();

    // -------------------------------------------------------------
    // 5. Create Display Window and Set Mouse Callback.
    // -------------------------------------------------------------
    cv::namedWindow("Video Stream", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Video Stream", onMouse, nullptr);

    // -------------------------------------------------------------
    // 6. Main Processing Loop.
    // -------------------------------------------------------------
    cv::Mat left_raw, left_rect;
    cv::Mat right_raw, right_rect;
    cv::Mat left_for_matcher, right_for_matcher;
    cv::Mat left_disp_half, left_disp_float;
    cv::Mat filteredDisp;   // To hold the filtered disparity map.
    cv::Mat left_depth_map;

    // Parameters for bilateral filtering (tweak these as necessary).
    int bilateralFilterDiameter = 9;
    double sigmaColor = 75.0;
    double sigmaSpace = 75.0;

    // Temporal smoothing window size.
    const int temporalWindow = 30;

    while (true)
    {
        if (!cap.read(frame))
        {
            std::cerr << "Error: Unable to retrieve frame from the stream." << std::endl;
            break;
        }

        // Extract left and right images from the side-by-side stereo frame.
        left_raw  = frame(cv::Rect(0, 0, left_width, frame.rows));
        right_raw = frame(cv::Rect(left_width, 0, left_width, frame.rows));

        // Rectify the images.
        cv::remap(left_raw, left_rect, map_left_x, map_left_y, cv::INTER_AREA);
        cv::remap(right_raw, right_rect, map_right_x, map_right_y, cv::INTER_AREA);

        // Downscale images for faster stereo matching.
        double resize_fact = 0.5;
        cv::resize(left_rect, left_for_matcher, cv::Size(), resize_fact, resize_fact, cv::INTER_AREA);
        cv::resize(right_rect, right_for_matcher, cv::Size(), resize_fact, resize_fact, cv::INTER_AREA);

        // Compute the disparity map.
        left_matcher->compute(left_for_matcher, right_for_matcher, left_disp_half);
        left_disp_half.convertTo(left_disp_float, CV_32FC1);
        cv::multiply(left_disp_float, 1.0 / 16.0, left_disp_float);  // Convert fixed-point to float

        // Upscale the disparity map to the original resolution.
        cv::multiply(left_disp_float, 2.0, left_disp_float);
        cv::resize(left_disp_float, left_disp_float, left_rect.size(), 0, 0, cv::INTER_LINEAR);

        // Apply a bilateral filter to reduce noise while preserving edges.
        cv::bilateralFilter(left_disp_float, filteredDisp, bilateralFilterDiameter, sigmaColor, sigmaSpace);

        // Compute the depth map using the filtered disparity:
        double numerator = g_fx * baseline;
        cv::divide(numerator, filteredDisp, left_depth_map);

        // Update the global depth map.
        g_depthMap = left_depth_map.clone();

        // Create a display image from the left rectified image.
        cv::Mat displayImg = left_rect.clone();

        // If one or two points are selected, mark them and compute distance.
        if (!g_selectedPoints.empty())
        {
            for (const auto &pt : g_selectedPoints)
                cv::circle(displayImg, pt, 5, cv::Scalar(0, 255, 0), -1);

            if (g_selectedPoints.size() == 2)
            {
                // Draw a line connecting the two points.
                cv::line(displayImg, g_selectedPoints[0], g_selectedPoints[1], cv::Scalar(255, 0, 0), 2);

                // Ensure both points are within the depth map bounds.
                int x1 = g_selectedPoints[0].x, y1 = g_selectedPoints[0].y;
                int x2 = g_selectedPoints[1].x, y2 = g_selectedPoints[1].y;
                if (x1 >= 0 && x1 < g_depthMap.cols && y1 >= 0 && y1 < g_depthMap.rows &&
                    x2 >= 0 && x2 < g_depthMap.cols && y2 >= 0 && y2 < g_depthMap.rows)
                {
                    // Get the depth (in mm) at the selected pixel positions.
                    float d1 = g_depthMap.at<float>(y1, x1);
                    float d2 = g_depthMap.at<float>(y2, x2);

                    // Reconstruct the 3D coordinates for both points:
                    // X = (u - cx) * depth / fx, Y = (v - cy) * depth / fy, Z = depth.
                    double X1 = (x1 - g_cx) * d1 / g_fx;
                    double Y1 = (y1 - g_cy) * d1 / g_fy;
                    double Z1 = d1;
                    double X2 = (x2 - g_cx) * d2 / g_fx;
                    double Y2 = (y2 - g_cy) * d2 / g_fy;
                    double Z2 = d2;

                    // Compute the instantaneous Euclidean distance between the 3D points (in mm).
                    double distance_mm = std::sqrt(std::pow(X1 - X2, 2) +
                                                   std::pow(Y1 - Y2, 2) +
                                                   std::pow(Z1 - Z2, 2));

                    // Add this distance to the temporal buffer.
                    g_distanceBuffer.push_back(distance_mm);
                    if (g_distanceBuffer.size() > temporalWindow)
                        g_distanceBuffer.erase(g_distanceBuffer.begin());  // Maintain a fixed-size sliding window.

                    // Compute the moving average of the distances.
                    double sum = 0.0;
                    for (double d : g_distanceBuffer)
                        sum += d;
                    double avgDistance_mm = sum / g_distanceBuffer.size();

                    // Convert from mm to cm and round to one decimal place.
                    g_smoothedDistance_cm = std::round((avgDistance_mm / 10.0) * 10.0) / 10.0;

                    // Display the smoothed distance (in cm) on the image.
                    std::ostringstream oss;
                    oss << "Distance: " << g_smoothedDistance_cm << " cm";
                    cv::putText(displayImg, oss.str(), cv::Point(50, 50),
                                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
                    std::cout << oss.str() << std::endl;
                }
            }
        }

        // Show the annotated video stream.
        cv::imshow("Video Stream", displayImg);
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q')
            break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
