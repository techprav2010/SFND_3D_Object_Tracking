#include <numeric>



#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType, Config3DObjectTrack &config3d, AuditLog &audit) {

    //    vector<string> matcherTypes = {"MAT_BF", "MAT_FLANN"};
    //    vector<string> matcherTypeMetrics = {"DES_BINARY", "DES_HOG"};
    //    vector<string> matcherTypeSelectors = {"SEL_NN", "SEL_KNN"};

    // configure matcher
    bool crossCheck = false;
    bool converted_CV_32F = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    double t = (double) cv::getTickCount();

    if (matcherType.compare("MAT_BF") == 0) {
        int normType = cv::NORM_HAMMING;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    } else if (matcherType.compare("MAT_FLANN") == 0) {
        // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
        descSource.convertTo(descSource, CV_32F);
        descRef.convertTo(descRef, CV_32F);
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0) { // nearest neighbor (best match)
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1

        //audit: long match_time, match_keypoints_size, match_removed_keypoints_size;
        audit.match_keypoints_size = matches.size();

    } else if (selectorType.compare("SEL_KNN") == 0) { // k nearest neighbors (k=2)
        vector<vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, 2);
        double minDescDistRatio = 0.8;
        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it) {
            if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance) {
                matches.push_back((*it)[0]);
            }
        }

        //audit: long match_time, match_keypoints_size, match_removed_keypoints_size;
        audit.match_keypoints_size = matches.size();
        audit.match_removed_keypoints_size = (knn_matches.size() - matches.size());
        cout << "# keypoints removed = " << knn_matches.size() - matches.size() << endl;
    }

    t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
    cout << selectorType << " with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
    //audit: time
    audit.match_time = 1000 * t / 1.0;
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType, Config3DObjectTrack &config3d, AuditLog &audit)
{
    // select appropriate descriptor
    double t = (double)cv::getTickCount();
    cv::Ptr<cv::DescriptorExtractor> extractor;
//    vector<string> descriptorTypes = {"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};

    if (descriptorType.compare("BRISK") == 0)
    {
        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        extractor = cv::xfeatures2d::SIFT::create();
    }
    else
    {
        //nothing
        return ; //raise error: not needed - so far
    }

    // perform feature description

    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;

    //audit:
    audit.desc_time = 1000 * t / 1.0;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, Config3DObjectTrack &config3d, AuditLog &audit, bool bVis)
{
    //    vector<string> detectorTypes = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};

    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {
        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    //audit:
    audit.detect_time = 1000 * t / 1.0;
    audit.detect_keypoints_size = keypoints.size();

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, Config3DObjectTrack &config3d, AuditLog &audit, bool bVis )
{
    //    vector<string> detectorTypes = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)
    double t = (double)cv::getTickCount();

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // visualize results
    if (bVis) {
        string windowName = "Harris Corner Detector Response Matrix";
        cv::namedWindow(windowName, 4);
        cv::imshow(windowName, dst_norm_scaled);
        cv::waitKey(0);
    }

    // STUDENTS NEET TO ENTER THIS CODE (C3.2 Atom 4)

    // Look for prominent corners and instantiate keypoints
//    vector<cv::KeyPoint> keypoints;
    double maxOverlap = 0.0; // max. permissible overlap between two features in %, used during non-maxima suppression
    for (size_t j = 0; j < dst_norm.rows; j++)
    {
        for (size_t i = 0; i < dst_norm.cols; i++)
        {
            int response = (int)dst_norm.at<float>(j, i);
            if (response > minResponse)
            { // only store points above a threshold

                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = response;

                // perform non-maximum suppression (NMS) in local neighbourhood around new key point
                bool bOverlap = false;
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                    if (kptOverlap > maxOverlap)
                    {
                        bOverlap = true;
                        if (newKeyPoint.response > (*it).response)
                        {                      // if overlap is >t AND response is higher for new kpt
                            *it = newKeyPoint; // replace old key point with new one
                            break;             // quit loop over keypoints
                        }
                    }
                }
                if (!bOverlap)
                {                                     // only add new key point if no overlap has been found in previous NMS
                    keypoints.push_back(newKeyPoint); // store new keypoint in dynamic list
                }
            }
        } // eof loop over cols
    }     // eof loop over rows

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "HARRIS detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
//    cout << " " << 1000 * t / 1.0;

    //audit:  long detect_time, detect_keypoints_size;
    audit.detect_time = 1000 * t / 1.0;
    audit.detect_keypoints_size = keypoints.size();

    // visualize keypoints
    if (bVis) {
        std::string windowName = "Harris Corner Detection Results";
        cv::namedWindow(windowName, 5);
        cv::Mat visImage = dst_norm_scaled.clone();
        cv::drawKeypoints(dst_norm_scaled, keypoints, visImage, cv::Scalar::all(-1),
                          cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, Config3DObjectTrack &config3d, AuditLog &audit, bool bVis ){

    //    vector<string> detectorTypes = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};

    cv::Ptr<cv::FeatureDetector> detector = NULL;
    double t = (double)cv::getTickCount();

    if (detectorType.compare("FAST") == 0)
    {
        int threshold = 30;
        bool bNMS = true;
        cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16;
        detector = cv::FastFeatureDetector::create(threshold, bNMS, type);
    }
    else if (detectorType.compare("BRISK") == 0)
    {
        detector = cv::BRISK::create();
    }
    else if (detectorType.compare("ORB") == 0)
    {
        detector = cv::ORB::create();
    }
    else if (detectorType.compare("AKAZE") == 0)
    {
        detector = cv::AKAZE::create();
    }
    else if (detectorType.compare("SIFT") == 0)
    {
        detector = cv::xfeatures2d::SIFT::create();
    }
    else
    {
        //nothing to do. should not come here. Already handled error in main.
        return;
    }

    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout <<   detectorType << " detection with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    //audit:  long detect_time, detect_keypoints_size;
    audit.detect_time = 1000 * t / 1.0;
    audit.detect_keypoints_size = keypoints.size();

    // visualize results
    if (bVis)
    {
        string windowName = " " + detectorType + " Detector Results";
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}
