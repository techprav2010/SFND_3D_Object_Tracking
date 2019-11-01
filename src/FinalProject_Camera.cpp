
/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"

using namespace std;


/////////////////////// audit log with multiple configurations /////////
Config3DObjectTrack getConfigListSingle(int use_test=1) {
    //int use_test = 1;
    vector<Config3DObjectTrack> configList;
    Config3DObjectTrack config;
    if (use_test == 1)
    {
        config.detectorType = "SHITOMASI";
        config.descriptorType = "BRIEF";
        config.matcherType = "MAT_FLANN";
        config.matcherTypeMetric = "DES_BINARY";
        config.matcherTypeSelector = "SEL_NN";
    }
    else if (use_test == 2)
    {
        config.detectorType = "AKAZE";
        config.descriptorType = "FREAK";
        config.matcherType = "MAT_FLANN";
        config.matcherTypeMetric = "DES_BINARY";
        config.matcherTypeSelector = "SEL_NN";
    }
    else if (use_test == 3)
    {
        config.detectorType = "SHITOMASI";
        config.descriptorType = "ORB";
        config.matcherType = "MAT_BF";
        config.matcherTypeMetric = "DES_BINARY";
        config.matcherTypeSelector = "SEL_NN";
    }
    else if (use_test == 4)
    {
        config.detectorType = "ORB";
        config.descriptorType = "ORB";
        config.matcherType = "MAT_BF";
        config.matcherTypeMetric = "DES_BINARY";
        config.matcherTypeSelector = "SEL_NN";
    }
    config.bVis = false;
    config.bLimitKpts = false;
    config.maxKeypoints = 50;
    config.bVisshow3DObjects = false;
    config.bWait3DObjects = false;
    config.bVisTTC = false;
    config.bSaveImg = true;

    return config;
}

vector<Config3DObjectTrack> getConfigListShort() {
    vector<Config3DObjectTrack> configList;
    vector<string> detectorTypes = { "FAST", "BRISK", "ORB", "AKAZE","SHITOMASI", "HARRIS", "SIFT"};
    vector<string> descriptorTypes = {"BRISK", "BRIEF", "ORB", "FREAK"};

    vector<string> matcherTypes = { "MAT_FLANN"};
    vector<string> matcherTypeMetrics = {"DES_BINARY"};
    vector<string> matcherTypeSelectors = {"SEL_KNN"};
    for (auto  descriptorType:descriptorTypes) {
        bool write_detector = false;

        for (auto detectorType:detectorTypes) // start
        {
            for (auto matcherType:matcherTypes) {
                for (auto matcherTypeMetric:matcherTypeMetrics) {
                    for (auto matcherTypeSelector:matcherTypeSelectors) {
                        Config3DObjectTrack config;
                        config.detectorType = detectorType;
                        config.descriptorType = descriptorType;
                        config.matcherType = matcherType;
                        config.matcherTypeMetric = matcherTypeMetric;
                        config.matcherTypeSelector = matcherTypeSelector;

                        configList.push_back(config);
                    }
                }
            }
        }
    }
    return configList;
}

vector<Config3DObjectTrack> getConfigListAll() {

    vector<Config3DObjectTrack> configList;
    //vector<string> detectorTypes = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
    vector<string> detectorTypes = { "FAST", "BRISK", "ORB", "AKAZE","SHITOMASI", "HARRIS", "SIFT"};
    vector<string> descriptorTypes = {"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};

    vector<string> matcherTypes = {"MAT_BF", "MAT_FLANN"};
    vector<string> matcherTypeMetrics = {"DES_BINARY", "DES_HOG"};
    vector<string> matcherTypeSelectors = {"SEL_NN", "SEL_KNN"};
    for (auto detectorType:detectorTypes) {
        bool write_detector = false;

        for (auto descriptorType:descriptorTypes) // start
        {
//                if (descriptorType.compare("AKAZE") == 0)
//                    continue;

            for (auto matcherType:matcherTypes) {
                for (auto matcherTypeMetric:matcherTypeMetrics) {
                    for (auto matcherTypeSelector:matcherTypeSelectors) {
                        Config3DObjectTrack config;
                        config.detectorType = detectorType;
                        config.descriptorType = descriptorType;
                        config.matcherType = matcherType;
                        config.matcherTypeMetric = matcherTypeMetric;
                        config.matcherTypeSelector = matcherTypeSelector;

                        configList.push_back(config);
                    }
                }
            }
        }
    }
    return configList;
}

std::string joinVec(vector<long> v)
{
    std::stringstream ss;
    for(size_t i = 0; i < v.size(); ++i)
    {
        if(i != 0)
            ss << ",";
        ss << v[i];
    }
    std::string s = ss.str();
    return s;
}
void log(ofstream &detector_file, AuditLog &audit) {

    cout << "joinVec(audit.ttc_camera)" << audit.ttc_camera   << endl;
    cout << "joinVec(audit.ttc_lidar)" << audit.ttc_lidar   << endl;

    detector_file << "{" << endl;

    detector_file << "'isError':'" << audit.isError << "'," << endl;
    detector_file << "'image_name':'" << audit.image_name  << "'," << endl;
    detector_file << "'lidar_file_name':'" << audit.lidar_file_name  << "'," << endl;
    detector_file << "'detectorType':'" << audit.config.detectorType << "'," << endl;
    detector_file << "'descriptorType':'" << audit.config.descriptorType  << "'," << endl;
    detector_file << "'matcherType':'" << audit.config.matcherType  << "'," << endl;

    detector_file << "'ttc_camera':'" <<  audit.ttc_camera  << "'," << endl;
    detector_file << "'ttc_lidar':'" <<  audit.ttc_lidar  << "'," << endl;

    detector_file << "'matcherTypeMetric':'" << audit.config.matcherTypeMetric << "'," << endl;
    detector_file << "'matcherTypeSelector':'" << audit.config.matcherTypeSelector  << "'," << endl;

    detector_file << "'detect_time_ms':" << audit.detect_time << "," << endl;
    detector_file << "'desc_time_ms':" << audit.desc_time << "," << endl;
    detector_file << "'match_time_ms':" << audit.match_time << "," << endl;

    detector_file << "'detect_keypoints_size':" << audit.detect_keypoints_size << "," << endl;
    detector_file << "'match_keypoints_size':" << audit.match_keypoints_size << "," << endl;
    detector_file << "'match_removed_keypoints_size':" << audit.match_removed_keypoints_size << "," << endl;


    detector_file << "'bVis':" << audit.config.bVis  << "," << endl;
    detector_file << "'bLimitKpts':" << audit.config.bLimitKpts  << "," << endl;
    detector_file << "'maxKeypoints':" << audit.config.maxKeypoints  << "," << endl;
    detector_file << "'bVisshow3DObjects':" << audit.config.bVisshow3DObjects  << "," << endl;

    detector_file << "}," << endl;
}

void log_audit_header(ofstream &detector_file) {
    detector_file << "error";

    detector_file << "," << "image_name";
    detector_file << "," << "lidar_file_name";
    detector_file << "," << "detectorType";
    detector_file << "," << "descriptorType";

    detector_file << "," << "matcherType";
    detector_file << "," << "matcherTypeMetric";
    detector_file << "," << "matcherTypeSelector";

    detector_file << "," << "ttc_camera";
    detector_file << "," << "ttc_lidar";

    detector_file << "," << "detect_time";
    detector_file << "," << "desc_time";
    detector_file << "," << "match_time";

    detector_file << "," << "detect_keypoints_size" ;
    detector_file << "," << "match_keypoints_size";
    detector_file << "," << "match_removed_keypoints_size";

    detector_file << "," << "bVis";
    detector_file << "," << "bLimitKpts";
    detector_file << "," << "maxKeypoints";
    detector_file << "," << "bVisshow3DObjects";

    detector_file  << endl;
}

void log_audit(ofstream &detector_file, AuditLog &audit) {

    detector_file << audit.isError;

    detector_file << "," << audit.image_name;
    detector_file << "," << audit.lidar_file_name;
    detector_file << "," << audit.config.detectorType;
    detector_file << "," << audit.config.descriptorType;
    detector_file << "," << audit.config.matcherType;
    detector_file << "," << audit.config.matcherTypeMetric;
    detector_file << "," << audit.config.matcherTypeSelector;

    detector_file << ",\"" <<  audit.ttc_camera  << "\""  ;
    detector_file << ",\"" <<  audit.ttc_lidar  << "\""  ;


    detector_file << "," << audit.detect_time;
    detector_file << "," << audit.desc_time;
    detector_file << "," << audit.match_time;

    detector_file << "," << audit.detect_keypoints_size ;
    detector_file << "," << audit.match_keypoints_size;
    detector_file << "," << audit.match_removed_keypoints_size;

    detector_file << "," << audit.config.bVis;
    detector_file << "," << audit.config.bLimitKpts;
    detector_file << "," << audit.config.maxKeypoints;
    detector_file << "," << audit.config.bVisshow3DObjects;

    detector_file   << endl;
}


/////////////////////// audit log END /////////


/////////////////////// run_3D_object_tracking  given a configuration /////////

int run_3D_object_tracking(Config3DObjectTrack &config3d, vector<AuditLog> audits, ofstream &detector_file_json, ofstream &detector_file) {
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";
    string output_folder = dataPath +"output/";

    if( mkdir(output_folder.c_str(),0777) == -1 ) {

    }
    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;   // last file index to load
    int imgStepWidth = 1;
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    string yoloBasePath = dataPath + "dat/yolo/";
    string yoloClassesFile = yoloBasePath + "coco.names";
    string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    string yoloModelWeights = yoloBasePath + "yolov3.weights";

    // Lidar
    string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    string lidarFileType = ".bin";

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector

    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;

    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;

    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;

    // misc
    double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
    int dataBufferSize =  2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex+=imgStepWidth)
    {
        //audit logs
        AuditLog audit;
        audit.config = config3d;
        audits.push_back(audit);
        cout << endl;
        cout << endl;
        cout << "START " << config3d.detectorType <<  " " <<   config3d.descriptorType << " " ;
        cout <<   config3d.matcherType << " " <<   config3d.matcherType << " " <<   config3d.matcherTypeMetric<< " " <<   config3d.matcherTypeSelector;
        cout << endl;
        try {

            /* LOAD IMAGE INTO BUFFER */

            // assemble filenames for current index
            ostringstream imgNumber;
            imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
            string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

            // load image from file
            cv::Mat img = cv::imread(imgFullFilename);

            //audit
            audit.image_name = imgFullFilename;

            // push image into data frame buffer
            DataFrame frame;
            frame.cameraImg = img;
            dataBuffer.push_back(frame);
            if (dataBuffer.size() > dataBufferSize) {
                //circular buffer - dataBufferSize=2
                dataBuffer.erase(dataBuffer.begin());
            }
            cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;


            /* DETECT & CLASSIFY OBJECTS */

            float confThreshold = 0.2;
            float nmsThreshold = 0.4;
            detectObjects((dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->boundingBoxes, confThreshold, nmsThreshold,
                          yoloBasePath, yoloClassesFile, yoloModelConfiguration, yoloModelWeights, bVis, config3d, audit);

            cout << "#2 : DETECT & CLASSIFY OBJECTS done" << endl;


            /* CROP LIDAR POINTS */

            // load 3D Lidar points from file
            string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
            std::vector<LidarPoint> lidarPoints;
            loadLidarFromFile(lidarPoints, lidarFullFilename);

            //audit
            audit.lidar_file_name = lidarFullFilename;

            // remove Lidar points based on distance properties
            float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
            cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);

            (dataBuffer.end() - 1)->lidarPoints = lidarPoints;

            cout << "#3 : CROP LIDAR POINTS done" << endl;


            /* CLUSTER LIDAR POINT CLOUD */

            // associate Lidar points with camera-based ROI
            float shrinkFactor = 0.10; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
            clusterLidarWithROI((dataBuffer.end()-1)->boundingBoxes, (dataBuffer.end() - 1)->lidarPoints, shrinkFactor, P_rect_00, R_rect_00, RT, config3d, audit);

            // Visualize 3D objects
            bVis = config3d.bVisshow3DObjects ;//true;
            if(bVis)
            {
                show3DObjects((dataBuffer.end()-1)->boundingBoxes, cv::Size(4.0, 20.0), cv::Size(2000, 2000), config3d, audit, config3d.bWait3DObjects);
            }
            bVis = false;

            cout << "#4 : CLUSTER LIDAR POINT CLOUD done" << endl;


            // REMOVE THIS LINE BEFORE PROCEEDING WITH THE FINAL PROJECT
            //continue; // skips directly to the next image without processing what comes beneath

            /* DETECT IMAGE KEYPOINTS */

            // convert current image to grayscale
            cv::Mat imgGray;
            cv::cvtColor((dataBuffer.end()-1)->cameraImg, imgGray, cv::COLOR_BGR2GRAY);

            // extract 2D keypoints from current image
            vector<cv::KeyPoint> keypoints; // create empty feature list for current image
            string detectorType = config3d.detectorType; //"SHITOMASI";

            //verify we have correct configuration arguments
            std::vector<string> detectors{"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
            if (std::find(detectors.begin(), detectors.end(), detectorType) != detectors.end()) {
                cout << "detectorType " << detectorType << endl;
            }
            else
            {
                cout << "Invalid detectorType " << detectorType << endl;
                return -1;
            }

            if (detectorType.compare("SHITOMASI") == 0)
            {
                detKeypointsShiTomasi(keypoints, imgGray, config3d, audit, false);
            }
            else if (detectorType.compare("HARRIS") == 0)
            {
                detKeypointsHarris(keypoints, imgGray, config3d, audit, false);
            }
            else
            {
                // FAST, BRISK, ORB, AKAZE, SIFT
                detKeypointsModern(keypoints, imgGray, detectorType, config3d, audit, false);
            }

            // optional : limit number of keypoints (helpful for debugging and learning)
            bool bLimitKpts =  config3d.bLimitKpts;//false;
            if (bLimitKpts)
            {
                int maxKeypoints = 50;

                if (detectorType.compare("SHITOMASI") == 0)
                { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                    keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                }
                cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                cout << " NOTE: Keypoints have been limited!" << endl;
            }

            // push keypoints and descriptor for current frame to end of data buffer
            (dataBuffer.end() - 1)->keypoints = keypoints;

            cout << "#5 : DETECT KEYPOINTS done =" << keypoints.size() << endl;


            /* EXTRACT KEYPOINT DESCRIPTORS */

            cv::Mat descriptors;
            string descriptorType =  config3d.descriptorType ;//"BRISK"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
            descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType, config3d, audit);

            // push descriptors for current frame to end of data buffer
            (dataBuffer.end() - 1)->descriptors = descriptors;

            cout << "#6 : EXTRACT DESCRIPTORS done =" << descriptors.size() << endl;


            if (dataBuffer.size() > 1) // wait until at least two images have been processed
            {

                /* MATCH KEYPOINT DESCRIPTORS */

                vector<cv::DMatch> matches;
                string matcherType = config3d.matcherType;//"MAT_BF";        // MAT_BF, MAT_FLANN
                string matchDescriptorType = config3d.matcherTypeMetric;//"DES_BINARY"; // DES_BINARY, DES_HOG
                string selectorType =  config3d.matcherTypeSelector;//"SEL_NN";       // SEL_NN, SEL_KNN

                matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                 (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                 matches, matchDescriptorType, matcherType, selectorType, config3d, audit);

                // store matches in current data frame
                (dataBuffer.end() - 1)->kptMatches = matches;

                cout << "#7 : MATCH KEYPOINT DESCRIPTORS done =" << matches.size() << endl;


                /* TRACK 3D OBJECT BOUNDING BOXES */

                //// STUDENT ASSIGNMENT
                //// TASK FP.1 -> match list of 3D objects (vector<BoundingBox>) between current and previous frame (implement ->matchBoundingBoxes)
                map<int, int> bbBestMatches;
                matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end()-2), *(dataBuffer.end()-1) , config3d, audit); // associate bounding boxes between current and previous frame using keypoint matches
                //// EOF STUDENT ASSIGNMENT

                // store matches in current data frame
                (dataBuffer.end()-1)->bbMatches = bbBestMatches;

                cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done =" << bbBestMatches.size() << endl;


                /* COMPUTE TTC ON OBJECT IN FRONT */

                // loop over all BB match pairs
                for (auto it1 = (dataBuffer.end() - 1)->bbMatches.begin(); it1 != (dataBuffer.end() - 1)->bbMatches.end(); ++it1)
                {
                    //cout << "it1->first =" << it1->first << " it1->second =" << it1->second << endl;
                    // find bounding boxes associates with current match
                    BoundingBox *prevBB, *currBB;
                    for (auto it2 = (dataBuffer.end() - 1)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 1)->boundingBoxes.end(); ++it2)
                    {
                        if (it1->second == it2->boxID) // check wether current match partner corresponds to this BB
                        {
                            // cout << "currBB FOUND it1->second =" << it1->second << " cur it2->boxID =" <<  it2->boxID ;
                            currBB = &(*it2);
                        }
                    }

                    for (auto it2 = (dataBuffer.end() - 2)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 2)->boundingBoxes.end(); ++it2)
                    {
                        if (it1->first == it2->boxID) // check wether current match partner corresponds to this BB
                        {
                            //cout << "prevBB FOUND it1->first =" << it1->first << " prev it2->boxID =" <<  it2->boxID ;
                            prevBB = &(*it2);
                        }
                    }
//                    cout << " currBB =" << currBB->lidarPoints.size()  ;
//                    cout << " prevBB =" << prevBB->lidarPoints.size() << endl;

                    // compute TTC for current match
                    if( currBB->lidarPoints.size()>0 && prevBB->lidarPoints.size()>0 ) // only compute TTC if we have Lidar points
                    {
                        cout << " currBB =" << currBB->lidarPoints.size()  ;
                        cout << " prevBB =" << prevBB->lidarPoints.size() << endl;
                        //// STUDENT ASSIGNMENT
                        //// TASK FP.2 -> compute time-to-collision based on Lidar data (implement -> computeTTCLidar)
                        double ttcLidar;
                        computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar, config3d, audit);
                        //// EOF STUDENT ASSIGNMENT

                        //// STUDENT ASSIGNMENT
                        //// TASK FP.3 -> assign enclosed keypoint matches to bounding box (implement -> clusterKptMatchesWithROI)
                        //// TASK FP.4 -> compute time-to-collision based on camera (implement -> computeTTCCamera)
                        double ttcCamera;
                        clusterKptMatchesWithROI(*currBB, (dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->kptMatches , config3d, audit);
                        if(currBB->kptMatches.size() > 0) {
                            computeTTCCamera((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                             currBB->kptMatches, sensorFrameRate, ttcCamera, config3d, audit);
                        }
                        else
                            ttcCamera = NAN;

                        //// EOF STUDENT ASSIGNMENT

                        cout << " TTC Lidar " << ttcLidar<< " TTC Camera " << ttcCamera << endl;

                        bVis = config3d.bVisTTC; //true;
                        bool bSave = config3d.bSaveImg; //true;
                        if ( bVis || bSave )
                        {
                            cv::Mat visImg = (dataBuffer.end() - 1)->cameraImg.clone();
                            showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
                            cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y), cv::Point(currBB->roi.x + currBB->roi.width, currBB->roi.y + currBB->roi.height), cv::Scalar(0, 255, 0), 2);

                            char str[200];
                            sprintf(str, "TTC Lidar : %f s, TTC Camera : %f s", ttcLidar, ttcCamera);
                            putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255));

                            if(bVis)
                            {
                                string windowName = "Final Results : TTC";
                                cv::namedWindow(windowName, 4);
                                cv::imshow(windowName, visImg);
                                cout << "Press key to continue to next frame" << endl;
                                cv::waitKey(0);
                            }

                            if (bSave)
                            {
                                string  test_prefix = config3d.detectorType +  "_" + config3d.descriptorType + "_"  +config3d.matcherType + "_" +config3d.matcherTypeSelector + "_" + config3d.matcherTypeMetric;
                                string saveImgFullFilename = output_folder  + "/" + test_prefix + "_" + imgNumber.str() + ".png";
                                const char *cp_saveImgFullFilename = saveImgFullFilename.c_str();
                                cv::imwrite(cp_saveImgFullFilename, visImg);
                            }

                        }
                        bVis = false;

                    } // eof TTC computation
                } // eof loop over all BB matches

            }

            //audit logs
            log(detector_file_json, audit);
            log_audit(detector_file, audit);

        } catch (...) {

            cout << "exception happened " << config3d.detectorType <<  " " <<   config3d.descriptorType << " " ;
            cout <<   config3d.matcherType << " " <<   config3d.matcherType << " " <<   config3d.matcherTypeMetric<< " " <<   config3d.matcherTypeSelector;

            audit.isError = true;
            log(detector_file_json, audit);
            log_audit(detector_file, audit);

            try
            {
                auto expPtr = std::current_exception();
                if(expPtr) std::rethrow_exception(expPtr);
            }
            catch(const std::exception& e) //it would not work if you pass by value
            {
                cout << "exception ??? " ;
                std::cout << e.what() << endl;
            }
        }
    } // eof loop over all images

    return 0;
}
/////////////////////// run_3D_object_tracking END /////////

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    // use singleTest &  singleTestConfig for one experiment
    bool singleTest = false;
   // bool singleTestConfig = 2; //change to use other single test
    bool shortTest = true;
    string file_prefix = "short";
    //if argument is passed should be = single/short/all
    if (argc > 0 ) {
        if(argv[0] == "single"){
            singleTest = true;
        }
        if(argv[0] == "short"){
            singleTest = false;
            shortTest = true;
        }
        if(argv[0] == "all"){
            singleTest = false;
            shortTest = false;
        }
    }
    // load configuration
    vector<Config3DObjectTrack> configList;  //shortTest=true/false . run all combination of all or shorter list
    if(singleTest)
    {
        //capture images for these 4 test cases
        configList.push_back(getConfigListSingle(1));
        configList.push_back(getConfigListSingle(2));
        configList.push_back(getConfigListSingle(3));
        configList.push_back(getConfigListSingle(4));
//        configList.push_back(getConfigListSingle(5));//default
        file_prefix = "one";
    }
    else if(shortTest)
    {
        configList = getConfigListShort();
        file_prefix = "short";
    }
    else
    {
        configList = getConfigListAll();
        file_prefix = "all";
    }

    //load audit logs
    ofstream detector_file;
    detector_file.open("../"+ file_prefix + "_results.csv");

    ofstream detector_file_json;
    detector_file_json.open("../"+ file_prefix + "_results.json");
    detector_file_json << "[" << endl;

    vector<AuditLog> audits;
    log_audit_header(detector_file);


    //run the test in loop
    for (auto config3d = configList.begin(); config3d != configList.end(); ++config3d) {
        try {
            //original main code is moved into this method.... so that we can run multiple test
            run_3D_object_tracking((*config3d), audits, detector_file_json, detector_file);
        } catch (...) {
            cout << "exception in main " ;
            try
            {
                auto expPtr = std::current_exception();
                if(expPtr) std::rethrow_exception(expPtr);
            }
            catch(const std::exception& e) //it would not work if you pass by value
            {
                cout << "real exception in main ??? " ;
                cout << e.what() ;
            }
            cout << endl;
        }
    }

//    log_audits(audits);
    detector_file_json << "]" << endl;
    detector_file.close();
    detector_file_json.close();
}
