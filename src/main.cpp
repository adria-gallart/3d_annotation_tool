#include "rgbd_grabber.h"

#include "ros/ros.h"

#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>


int keyPressed()
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    // don't echo and don't wait for ENTER
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);

    // make it non-blocking (so we can check without waiting)
    if (0 != fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK))
    {
        return 0;
    }

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    if (0 != fcntl(STDIN_FILENO, F_SETFL, oldf))
    {
        return 0;
    }

    if(ch != EOF)
    {
//        ungetc(ch, stdin);
        return ch;
    }

    return 0;
}

int main(int argc, char *argv[])
{
    // Create kinect node
    ros::init(argc, argv, "kinectListenner");

    RGBDGrabber grabber;

    ros::NodeHandle n;
//    ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 2, &DataCompressionNode::pointCloudCallback, &dataCompressor);
    ros::Subscriber sub2 = n.subscribe("/camera/rgb/image_color", 2, &RGBDGrabber::colorImageCallback, &grabber);
    ros::Subscriber sub3 = n.subscribe("/camera/depth_registered/image_rect", 2, &RGBDGrabber::depthImageCallback, &grabber);

    ros::Rate loop_rate(10);

    bool loop = true;
    std::cout<<"------------------------- RGBD_GRABBER started -----------------------------"<<std::endl;
    while (ros::ok() && loop)
    {
        int ch = keyPressed();

        switch (ch)
        {
            case 113: // key q
            {
                std::cout<<"------------------------- Exitting program -----------------------------"<<std::endl;
                loop = false;
                break;
            }

            case 115: // key s
            {
                grabber.setSaveOneFrame(true);
                std::cout<<"------------------------- Saving a frame -----------------------------"<<std::endl;
                break;
            }
            case 97: // key a
            {
                grabber.setSaveFrameSeq(true);
                std::cout<<"------------------------- Saving frame sequence-----------------------------"<<std::endl;
                break;
            }
            case 122: // key z
            {
                grabber.setSaveFrameSeq(false);
                std::cout<<"------------------------- Stopping saving -----------------------------"<<std::endl;
                break;
            }
            case 110: // key n
            {
                grabber.createNewFolder();
                std::string folderName = grabber.getFolderName();
                std::cout<<"------------------------- Created new folder "<<folderName<<"-----------------------------"<<std::endl;
                break;
            }

            case 0:
                break;
            default:
                   break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
