
#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <map>
#include <opencv2/core.hpp>

struct LidarPoint { // single lidar point in space
    double x,y,z,r; // x,y,z in [m], r is point reflectivity
};

struct BoundingBox { // bounding box around a classified object (contains both 2D and 3D data)
    
    int boxID; // unique identifier for this bounding box
    int trackID; // unique identifier for the track to which this bounding box belongs
    
    cv::Rect roi; // 2D region-of-interest in image coordinates
    int classID; // ID based on class file provided to YOLO framework
    double confidence; // classification trust

    std::vector<LidarPoint> lidarPoints; // Lidar 3D points which project into 2D image roi
    std::vector<cv::KeyPoint> keypoints; // keypoints enclosed by 2D roi
    std::vector<cv::DMatch> kptMatches; // keypoint matches enclosed by 2D roi
};

struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
    std::vector<LidarPoint> lidarPoints;

    std::vector<BoundingBox> boundingBoxes; // ROI around detected objects in 2D image coordinates
    std::map<int,int> bbMatches; // bounding box matches between previous and current frame
};


// audit experiment results with multiple configurations
struct Config3DObjectTrack {
    std::string detectorType = "SHITOMASI";
    std::string descriptorType = "BRISK"; // BRIEF, ORB, FREAK, AKAZE, SIFT

    std::string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
    std::string matcherTypeMetric = "DES_BINARY"; // DES_BINARY, DES_HOG
    std::string matcherTypeSelector = "SEL_NN";       // SEL_NN, SEL_KNN

    bool bVis = false;
    bool bLimitKpts = false;
    int maxKeypoints = 50;

    bool bVisshow3DObjects= false;
    bool bWait3DObjects= false;

    bool bVisTTC= false;

};

// audit experiment results
struct AuditLog {
    Config3DObjectTrack config ;
    std::string image_name ="";
    std::string lidar_file_name ="";
    std::vector<long> ttc_camera;
    std::vector<long> ttc_lidar;

    bool isError = false;
    long match_time = 0;
    long match_keypoints_size = 0;
    long match_removed_keypoints_size = 0;
    long desc_time  = 0;
    long detect_time = 0;
    long detect_keypoints_size = 0;
};

#endif /* dataStructures_h */
