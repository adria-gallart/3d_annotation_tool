#ifndef RGBD_GRABBER_HH
#define RGBD_GRABBER_HH

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include "ros/ros.h"

#include <string>

class RGBDGrabber
{


public:
    RGBDGrabber();
    ~RGBDGrabber();

    void pointCloudCallback (const sensor_msgs::PointCloud2::ConstPtr& msg);
    void colorImageCallback (const sensor_msgs::Image::ConstPtr& img);
    void depthImageCallback (const sensor_msgs::Image::ConstPtr& img);

    void setSaveOneFrame(const bool&);
    void setSaveFrameSeq(const bool&);
    std::string getFolderName();

    bool createNewFolder();


    static std::string getDateTime();

private:
    volatile bool        m_bSaveOneFrame, m_bSaveFrameSequence, m_bCreateCVWindow;
    int                  m_iSequenceNumber;
    std::string          m_sCurrentFolder;
};

#endif // DATA_COMPRESION_NODE_HH
