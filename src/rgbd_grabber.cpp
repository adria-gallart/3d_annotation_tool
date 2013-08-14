#include "rgbd_grabber.h"

#include <sstream>
#include <stdio.h>
#include <time.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

RGBDGrabber::RGBDGrabber() : m_bSaveOneFrame(false), m_bSaveFrameSequence(false), m_bCreateCVWindow(false), m_bLensCovered(true),
    m_iSequenceNumber(0), m_dLastTimestamp(0.0), m_iImageSyncQueueLength(10), m_dImageSyncTimout(2.0), m_dImageSyncBetweenFrames(0.02)
{
    bool folderCreated = createNewFolder(); // to initialize m_sCurrentFolder (also to actually create the folder)
    if (!folderCreated)
    {
        m_sCurrentFolder = "";
        m_fIndexFile.open("index.txt");
    }

    m_dAvgPixelValueWhenLensCovered = 16250.0;
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

    if ((m_bSaveFrameSequence || m_bSaveOneFrame) && (!m_bLensCovered))
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

        // identify the sequence number of this frame by matching its timestamp with the timestamps of previously saved Depth images
        double currentTimestamp = imgMsg->header.stamp.toSec();
        int frameSequenceNumber = -1;

        if (m_DepthSequenceAndTimestamp.size() == 0)
        {
            // first frame
            m_iSequenceNumber++;
            frameSequenceNumber = m_iSequenceNumber;
        } else if (currentTimestamp > m_DepthSequenceAndTimestamp.back().second)
        {
            // this frame is ahead of all the previously saved depth images
            if (abs(currentTimestamp - m_DepthSequenceAndTimestamp.back().second) < m_dImageSyncBetweenFrames)
            {
                // the difference between the previous frame and this frame is small, keep the same sequence number
                frameSequenceNumber = m_DepthSequenceAndTimestamp.back().first;
            } else{
                m_iSequenceNumber++;
                frameSequenceNumber = m_iSequenceNumber;
            }
        } else {
            // find the depth image which is closest to the current image (timestamp wise)
            double smallestDiff = m_dImageSyncTimout;
            int closestSequenceNumber = -1;
            for (int i=m_DepthSequenceAndTimestamp.size()-1; i>=0;i--)
            {
                double diff = abs(currentTimestamp - m_DepthSequenceAndTimestamp[i].second);
                if (diff < smallestDiff)
                {
                    smallestDiff = diff;
                    closestSequenceNumber = m_DepthSequenceAndTimestamp[i].first;
                }
            }

            if (smallestDiff <m_dImageSyncTimout)
            {
                frameSequenceNumber = closestSequenceNumber;
            } else {
                cout<<"ERROR - cannot save image as it is out of sync. Time difference > "<<m_dImageSyncTimout<<endl;
            }
        }


        if (frameSequenceNumber == -1)
        {
            cout<<"An error occured, cannot save depth image"<<endl;
        }

        char buffer[50];
        if (frameSequenceNumber<10)
        {
            sprintf(buffer,"RGB000%d.png",frameSequenceNumber);
        } else if (frameSequenceNumber<100)
        {
            sprintf(buffer,"RGB00%d.png",frameSequenceNumber);
        } else if (frameSequenceNumber<1000)
        {
            sprintf(buffer,"RGB0%d.png",frameSequenceNumber);
        } else if (frameSequenceNumber<10000)
        {
            sprintf(buffer,"RGB%d.png",frameSequenceNumber);
        } else {
            // saved the maximum number of images -> print an error and return
            cout << "ERROR - cannot save image, already saved the maximum number of images."<<endl;
            return;
        }

        string completeName = m_sCurrentFolder + string("/") + string(buffer);


        imwrite(completeName.c_str(),cv_ptr->image);
        m_fIndexFile<<buffer<<" "<<imgMsg->header.stamp<<"\n";

        std::cout<<"RGBDGrabber :: saved color file "<<buffer<<"   time stamp  "<<imgMsg->header.stamp<<std::endl;

        m_bSaveOneFrame = false;       
        m_RGBSequenceAndTimestamp.push_back(std::make_pair(m_iSequenceNumber,currentTimestamp));
        if (m_RGBSequenceAndTimestamp.size() > m_iImageSyncQueueLength)
        {
            m_RGBSequenceAndTimestamp.pop_front(); // remove the oldest element from the queue
        }

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
        cv_ptr->image*=16;


        // compute the pixel average, in order to detect whether the lens is covered or not
        double avg = 0.0;
        for (size_t i=0; i<cv_ptr->image.rows;i++)
            for (size_t j=0; j<cv_ptr->image.cols;j++)
                avg+=cv_ptr->image.at<int16_t>(i,j);

        avg/=(cv_ptr->image.rows*cv_ptr->image.cols);

        if (avg>m_dAvgPixelValueWhenLensCovered)
        {
            m_bLensCovered = true;
            // check whether we need to create a new folder
            cout<<"Lens covered; skipping image."<<endl;
            if (m_bSaveFrameSequence && (m_iSequenceNumber != 0))
            {
                createNewFolder(); // this also sets m_iSequenceNumber to 0
                cout<<"Lens covered; skipping image. Creating new folder."<<endl;
            }
            return;
        } else {
            m_bLensCovered = false;
        }

//        imshow( "DepthWindow", cv_ptr->image );
//        cv::waitKey(10);

        // identify the sequence number of this frame by matching its timestamp with the timestamps of previously saved Depth images
        double currentTimestamp = imgMsg->header.stamp.toSec();
        int frameSequenceNumber = -1;
        if (m_RGBSequenceAndTimestamp.size() == 0)
        {
            // first frame
            m_iSequenceNumber++;
            frameSequenceNumber = m_iSequenceNumber;
        } else if (currentTimestamp > m_RGBSequenceAndTimestamp.back().second)
        {
            // this frame is ahead of all the previously saved depth images
            if (abs(currentTimestamp - m_RGBSequenceAndTimestamp.back().second) < m_dImageSyncBetweenFrames)
            {
                // the difference between the previous frame and this frame is small, keep the same sequence number
                frameSequenceNumber = m_RGBSequenceAndTimestamp.back().first;
            } else{
                m_iSequenceNumber++;
                frameSequenceNumber = m_iSequenceNumber;
            }
        } else {
            // find the depth image which is closest to the current image (timestamp wise)
            double smallestDiff = m_dImageSyncTimout;
            int closestSequenceNumber = -1;
            for (int i=m_RGBSequenceAndTimestamp.size()-1; i>=0;i--)
            {
                double diff = abs(currentTimestamp - m_RGBSequenceAndTimestamp[i].second);
                if (diff < smallestDiff)
                {
                    smallestDiff = diff;
                    closestSequenceNumber = m_RGBSequenceAndTimestamp[i].first;
                }
            }

            if (smallestDiff <m_dImageSyncTimout)
            {
                frameSequenceNumber = closestSequenceNumber;
            } else {
                cout<<"ERROR - cannot save image as it is out of sync. Time difference > "<<m_dImageSyncTimout<<endl;
            }
        }

        if (frameSequenceNumber == -1)
        {
            cout<<"An error occured, cannot save depth image"<<endl;
        }

        char buffer[50];
        if (frameSequenceNumber<10)
        {
            sprintf(buffer,"Depth000%d.png",frameSequenceNumber);
        } else if (frameSequenceNumber<100)
        {
            sprintf(buffer,"Depth00%d.png",frameSequenceNumber);
        } else if (frameSequenceNumber<1000)
        {
            sprintf(buffer,"Depth0%d.png",frameSequenceNumber);
        } else if (frameSequenceNumber<10000)
        {
            sprintf(buffer,"Depth%d.png",frameSequenceNumber);
        } else {
            // saved the maximum number of images -> print an error and return
            cout << "ERROR - cannot save image, already saved the maximum number of images."<<endl;
            return;
        }

        string completeName = m_sCurrentFolder + string("/") + string(buffer);

        imwrite(completeName.c_str(),cv_ptr->image);
        m_fIndexFile<<buffer<<" "<<imgMsg->header.stamp<<"\n";

        std::cout<<"RGBDGrabber :: saved depth file "<<buffer<<" pixel average  "<<avg<<"   time stamp  "<<imgMsg->header.stamp<<std::endl;

        m_bSaveOneFrame = false;
        m_DepthSequenceAndTimestamp.push_back(make_pair(m_iSequenceNumber,imgMsg->header.stamp.toSec()));
        if (m_DepthSequenceAndTimestamp.size() > m_iImageSyncQueueLength)
        {
            m_DepthSequenceAndTimestamp.pop_front(); // remove the oldest element from the queue
        }

        m_dLastTimestamp = imgMsg->header.stamp.toSec();
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
        // create new folder
        mkdir(newFolderName.c_str(), 0700);
        m_sCurrentFolder = newFolderName;
        m_iSequenceNumber = 0; // reset

        // create new index file
        string indexFilename = m_sCurrentFolder+string("/")+string("index.txt");
        if (m_fIndexFile.is_open())
        {
            m_fIndexFile.close();
        }
        m_fIndexFile.open(indexFilename.c_str());

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
