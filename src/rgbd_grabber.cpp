#include "rgbd_grabber.h"

#include <sstream>
#include <stdio.h>
#include <time.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

RGBDGrabber::RGBDGrabber() : m_bSaveOneFrame(false), m_bSaveFrameSequence(false), m_bCreateCVWindow(false),
    m_iSequenceNumber(0)
{
    bool folderCreated = createNewFolder(); // to initialize m_sCurrentFolder (also to actually create the folder)
    if (!folderCreated)
    {
        m_sCurrentFolder = "";
    }
}

//---------------------------------------------------------------------------------------------
void RGBDGrabber::pointCloudCallback (const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    std::cout << "New colored point cloud received " << std::endl;

}
//---------------------------------------------------------------------------------------------
void RGBDGrabber::colorImageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{
//    std::cout << "New color camera image received " << std::endl;

    if (m_bSaveFrameSequence || m_bSaveOneFrame)
    {
        // save this frame
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(imgMsg, "bgr8");
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        char buffer[50];
        if (m_iSequenceNumber<10)
        {
            sprintf(buffer,"RGB000%d.png",m_iSequenceNumber);
        } else if (m_iSequenceNumber<100)
        {
            sprintf(buffer,"RGB00%d.png",m_iSequenceNumber);
        } else if (m_iSequenceNumber<1000)
        {
            sprintf(buffer,"RGB0%d.png",m_iSequenceNumber);
        } else if (m_iSequenceNumber<10000)
        {
            sprintf(buffer,"RGB%d.png",m_iSequenceNumber);
        } else {
            // saved the maximum number of images -> print an error and return
            cout << "ERROR - cannot save image, already saved the maximum number of images."<<endl;
            return;
        }

        string completeName = m_sCurrentFolder + string("/") + string(buffer);

        imwrite(completeName.c_str(),cv_ptr->image);
        std::cout<<"RGBDGrabber :: saved color file "<<buffer<<std::endl;

        m_bSaveOneFrame = false;
        m_iSequenceNumber++;
    }


}
//---------------------------------------------------------------------------------------------
void RGBDGrabber::depthImageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{
//    std::cout << "New depth camera image received " << std::endl;

    if (m_bSaveFrameSequence || m_bSaveOneFrame)
    {
        // save this frame
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(imgMsg, "32FC1");
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        imshow( "DepthWindow", cv_ptr->image );
        cv_ptr->image*=16;
        cv::waitKey(10);

        char buffer[50];
        if (m_iSequenceNumber<10)
        {
            sprintf(buffer,"Depth000%d.png",m_iSequenceNumber);
        } else if (m_iSequenceNumber<100)
        {
            sprintf(buffer,"Depth00%d.png",m_iSequenceNumber);
        } else if (m_iSequenceNumber<1000)
        {
            sprintf(buffer,"Depth0%d.png",m_iSequenceNumber);
        } else if (m_iSequenceNumber<10000)
        {
            sprintf(buffer,"Depth%d.png",m_iSequenceNumber);
        } else {
            // saved the maximum number of images -> print an error and return
            cout << "ERROR - cannot save image, already saved the maximum number of images."<<endl;
            return;
        }

        string completeName = m_sCurrentFolder + string("/") + string(buffer);

        imwrite(completeName.c_str(),cv_ptr->image);
        std::cout<<"RGBDGrabber :: saved depth file "<<buffer<<std::endl;

        m_bSaveOneFrame = false;
        m_iSequenceNumber++;
    }

}
//---------------------------------------------------------------------------------------------
void RGBDGrabber::setSaveOneFrame(const bool& saveOneFrame)
{
    m_bSaveOneFrame = saveOneFrame;
}

//---------------------------------------------------------------------------------------------
void RGBDGrabber::setSaveFrameSeq(const bool& saveFrameSeq)
{
    m_bSaveFrameSequence = saveFrameSeq;
}
//---------------------------------------------------------------------------------------------
RGBDGrabber::~RGBDGrabber()
{

}
//---------------------------------------------------------------------------------------------
bool RGBDGrabber::createNewFolder()
{
    string newFolderName = RGBDGrabber::getDateTime();

    struct stat st = {0};

    if (stat(newFolderName.c_str(), &st) == -1) {
        mkdir(newFolderName.c_str(), 0700);
        m_sCurrentFolder = newFolderName;
        m_iSequenceNumber = 0; // reset
        return true;
    }

    return false;
}
std::string RGBDGrabber::getFolderName()
{
    return m_sCurrentFolder;
}

//---------------------------------------------------------------------------------------------
std::string RGBDGrabber::getDateTime()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://www.cplusplus.com/reference/clibrary/ctime/strftime/
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}
//---------------------------------------------------------------------------------------------
