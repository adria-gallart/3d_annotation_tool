#include "qsr.h"

#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/common/angles.h>

qsr::qsr()
{
}

pcl::PointXYZ qsr::getCenterOfMass(object obj){
    pcl::PointXYZ centerOfMass;
    centerOfMass.x = obj.geometry.length/2;
    centerOfMass.y = obj.geometry.width/2;
    centerOfMass.z = obj.geometry.height/2;

    Eigen::Affine3f transformation = pcl::getTransformation (obj.geometry.pose.x,
                                                             obj.geometry.pose.y,
                                                             obj.geometry.pose.z,
                                                             obj.geometry.roll,
                                                             obj.geometry.pitch,
                                                             obj.geometry.yaw);

    centerOfMass = pcl::transformPoint(centerOfMass, transformation);

    return centerOfMass;
}

void qsr::calculatePointsLandmark(object landmark){
    // Firstly, get the transformation of the landmark
    Eigen::Affine3f transformationLandmark = pcl::getTransformation (landmark.geometry.pose.x,
                                                                     landmark.geometry.pose.y,
                                                                     landmark.geometry.pose.z,
                                                                     landmark.geometry.roll,
                                                                     landmark.geometry.pitch,
                                                                     landmark.geometry.yaw);
    // Calculate the points in the center of each face
    // Right center point
    _RCPoint.x = landmark.geometry.length;
    _RCPoint.y = landmark.geometry.width/2;
    _RCPoint.z = landmark.geometry.height/2;
    _RCPoint = pcl::transformPoint(_RCPoint, transformationLandmark);

    // Left center point
    _LCPoint.x = 0;
    _LCPoint.y = landmark.geometry.width/2;
    _LCPoint.z = landmark.geometry.height/2;
    _LCPoint = pcl::transformPoint(_LCPoint, transformationLandmark);

    // Front center point
    _FCPoint.x = landmark.geometry.length/2;
    _FCPoint.y = 0;
    _FCPoint.z = landmark.geometry.height/2;
    _FCPoint = pcl::transformPoint(_FCPoint, transformationLandmark);

    // Back center point
    _BCPoint.x = landmark.geometry.length/2;
    _BCPoint.y = landmark.geometry.width;
    _BCPoint.z = landmark.geometry.height/2;
    _BCPoint = pcl::transformPoint(_BCPoint, transformationLandmark);

    // Calculate the lower corners points
    // Front face, left lower point
    _FLDPoint.x = 0;
    _FLDPoint.y = 0;
    _FLDPoint.z = 0;
    _FLDPoint = pcl::transformPoint(_FLDPoint, transformationLandmark);

    // Front face, right lower point
    _FRDPoint.x = landmark.geometry.length;
    _FRDPoint.y = 0;
    _FRDPoint.z = 0;
    _FRDPoint = pcl::transformPoint(_FRDPoint, transformationLandmark);

    // Back face, left lower point
    _BLDPoint.x = landmark.geometry.length;
    _BLDPoint.y = landmark.geometry.width;
    _BLDPoint.z = 0;
    _BLDPoint = pcl::transformPoint(_BLDPoint, transformationLandmark);

    // Left face, right lower point
    _BRDPoint.x = 0;
    _BRDPoint.y = landmark.geometry.width;
    _BRDPoint.z = 0;
    _BRDPoint = pcl::transformPoint(_BRDPoint, transformationLandmark);
}

void qsr::calculatePointTrajector(object trajector){
    _centerOfMassTrajector = getCenterOfMass(trajector);
}

void qsr::calculateAnglesOnRight(){
    // Front face direction
    Eigen::Vector4f frontDirection4f(_FRDPoint.x - _FLDPoint.x, _FRDPoint.y - _FLDPoint.y, 0, 0);
    Eigen::Vector3f frontDirection3f = frontDirection4f.segment(0,3);

    // Line FRD and center of mass
    Eigen::Vector4f rightCornerCenterLine4f(_centerOfMassTrajector.x - _FRDPoint.x,
                                            _centerOfMassTrajector.y - _FRDPoint.y,
                                            0,
                                            0);

    Eigen::Vector3f rightCornerCenterLine3f = rightCornerCenterLine4f.segment(0,3);

    // Line BLD and center of mass
    Eigen::Vector4f leftCornerCenterLine4f(_centerOfMassTrajector.x - _BLDPoint.x,
                                           _centerOfMassTrajector.y - _BLDPoint.y,
                                           0,
                                           0);

    Eigen::Vector3f leftCornerCenterLine3f = leftCornerCenterLine4f.segment(0,3);

    // Cross product to know which is the orientation
    Eigen::Vector3f crossProductRight = frontDirection3f.cross(rightCornerCenterLine3f);

    // Calculate the angle and modify depending the orientation
    _angleRightOnRight = pcl::getAngle3D (rightCornerCenterLine4f, frontDirection4f);
    if(crossProductRight(2)<0) _angleRightOnRight = 2*M_PI-_angleRightOnRight;

    // Cross product to know which is the orientation
    Eigen::Vector3f crossProductLeft = frontDirection3f.cross(leftCornerCenterLine3f);
    // Calculate the angle and modify depending the orientation
    _angleLeftOnRight = pcl::getAngle3D (leftCornerCenterLine4f, frontDirection4f);
    if(crossProductLeft(2)>0) _angleLeftOnRight = 2*M_PI-_angleLeftOnRight;

    //    std::cout << "----------------- On right of -----------------" << std::endl;
    //    std::cout << "Angle Right: " << pcl::rad2deg(angleRight) << std::endl;
    //    std::cout << "Angle Left: " << pcl::rad2deg(angleLeft) << std::endl;

    //    if(angleRight < M_PI/2 && angleLeft < M_PI/2)
    //        std::cout << "The trajector can be on the right of the landmark.\n" << std::endl;
    //    else
    //        std::cout << "The trajector is not on the right of the landmark.\n" << std::endl;

}

void qsr::calculateAnglesOnLeft(){
    // Front face direction
    Eigen::Vector4f backDirection4f(_BRDPoint.x - _BLDPoint.x, _BRDPoint.y - _BLDPoint.y, 0, 0);
    Eigen::Vector3f backDirection3f = backDirection4f.segment(0,3);

    // Line BRD and center of mass
    Eigen::Vector4f rightCornerCenterLine4f(_centerOfMassTrajector.x - _BRDPoint.x,
                                            _centerOfMassTrajector.y - _BRDPoint.y,
                                            0,
                                            0);

    Eigen::Vector3f rightCornerCenterLine3f = rightCornerCenterLine4f.segment(0,3);

    // Line FLD and center of mass
    Eigen::Vector4f leftCornerCenterLine4f(_centerOfMassTrajector.x - _FLDPoint.x,
                                           _centerOfMassTrajector.y - _FLDPoint.y,
                                           0,
                                           0);

    Eigen::Vector3f leftCornerCenterLine3f = leftCornerCenterLine4f.segment(0,3);

    // Cross product to know which is the orientation
    Eigen::Vector3f crossProductRight = backDirection3f.cross(rightCornerCenterLine3f);
    // Calculate the angle and modify depending the orientation
    _angleRightOnLeft = pcl::getAngle3D (rightCornerCenterLine4f, backDirection4f);
    if(crossProductRight(2)<0) _angleRightOnLeft = 2*M_PI-_angleRightOnLeft;

    // Cross product to know which is the orientation
    Eigen::Vector3f crossProductLeft = backDirection3f.cross(leftCornerCenterLine3f);
    // Calculate the angle and modify depending the orientation
    _angleLeftOnLeft = pcl::getAngle3D (leftCornerCenterLine4f, backDirection4f);
    if(crossProductLeft(2)>0) _angleLeftOnLeft = 2*M_PI-_angleLeftOnLeft;

    //    std::cout << "----------------- On left of -----------------" << std::endl;
    //    std::cout << "Angle Right: " << pcl::rad2deg(angleRight) << std::endl;
    //    std::cout << "Angle Left: " << pcl::rad2deg(angleLeft) << std::endl;

    //    if(angleRight < M_PI/2 && angleLeft < M_PI/2)
    //        std::cout << "The trajector can be in the left of the landmark.\n" << std::endl;
    //    else
    //        std::cout << "The trajector is not in the left of the landmark.\n" << std::endl;
}

void qsr::calculateAnglesInFront(){
    // Left face direction
    Eigen::Vector4f leftDirection4f(_FLDPoint.x - _BRDPoint.x, _FLDPoint.y - _BRDPoint.y, 0, 0);
    Eigen::Vector3f leftDirection3f = leftDirection4f.segment(0,3);

    // Line FLD and center of mass
    Eigen::Vector4f rightCornerCenterLine4f(_centerOfMassTrajector.x - _FLDPoint.x,
                                            _centerOfMassTrajector.y - _FLDPoint.y,
                                            0,
                                            0);

    Eigen::Vector3f rightCornerCenterLine3f = rightCornerCenterLine4f.segment(0,3);

    // Line FRD and center of mass
    Eigen::Vector4f leftCornerCenterLine4f(_centerOfMassTrajector.x - _FRDPoint.x,
                                           _centerOfMassTrajector.y - _FRDPoint.y,
                                           0,
                                           0);

    Eigen::Vector3f leftCornerCenterLine3f = leftCornerCenterLine4f.segment(0,3);

    // Cross product to know which is the orientation
    Eigen::Vector3f crossProductRight = leftDirection3f.cross(rightCornerCenterLine3f);
    // Calculate the angle and modify depending the orientation
    _angleRightInFront = pcl::getAngle3D (rightCornerCenterLine4f, leftDirection4f);
    if(crossProductRight(2)<0) _angleRightInFront = 2*M_PI-_angleRightInFront;

    // Cross product to know which is the orientation
    Eigen::Vector3f crossProductLeft = leftDirection3f.cross(leftCornerCenterLine3f);
    // Calculate the angle and modify depending the orientation
    _angleLeftInFront = pcl::getAngle3D (leftCornerCenterLine4f, leftDirection4f);
    if(crossProductLeft(2)>0) _angleLeftInFront = 2*M_PI-_angleLeftInFront;

    //    std::cout << "----------------- In front of -----------------" << std::endl;
    //    std::cout << "Angle Right: " << pcl::rad2deg(angleRight) << std::endl;
    //    std::cout << "Angle Left: " << pcl::rad2deg(angleLeft) << std::endl;

    //    if(angleRight < M_PI/2 && angleLeft < M_PI/2)
    //        std::cout << "The trajector can be in front of the landmark.\n" << std::endl;
    //    else
    //        std::cout << "The trajector is not in front of the landmark.\n" << std::endl;
}

void qsr::calculateAnglesBehind(){
    // Left face direction
    Eigen::Vector4f rightDirection4f(_BLDPoint.x - _FRDPoint.x, _BLDPoint.y - _FRDPoint.y, 0, 0);
    Eigen::Vector3f rightDirection3f = rightDirection4f.segment(0,3);

    // Line BLD and center of mass
    Eigen::Vector4f rightCornerCenterLine4f(_centerOfMassTrajector.x - _BLDPoint.x,
                                            _centerOfMassTrajector.y - _BLDPoint.y,
                                            0,
                                            0);

    Eigen::Vector3f rightCornerCenterLine3f = rightCornerCenterLine4f.segment(0,3);

    // Line BRD and center of mass
    Eigen::Vector4f leftCornerCenterLine4f(_centerOfMassTrajector.x - _BRDPoint.x,
                                           _centerOfMassTrajector.y - _BRDPoint.y,
                                           0,
                                           0);

    Eigen::Vector3f leftCornerCenterLine3f = leftCornerCenterLine4f.segment(0,3);

    // Cross product to know which is the orientation
    Eigen::Vector3f crossProductRight = rightDirection3f.cross(rightCornerCenterLine3f);
    // Calculate the angle and modify depending the orientation
    _angleRightBehind = pcl::getAngle3D (rightCornerCenterLine4f, rightDirection4f);
    if(crossProductRight(2)<0) _angleRightBehind = 2*M_PI-_angleRightBehind;

    // Cross product to know which is the orientation
    Eigen::Vector3f crossProductLeft = rightDirection3f.cross(leftCornerCenterLine3f);
    // Calculate the angle and modify depending the orientation
    _angleLeftBehind = pcl::getAngle3D (leftCornerCenterLine4f, rightDirection4f);
    if(crossProductLeft(2)>0) _angleLeftBehind = 2*M_PI-_angleLeftBehind;

    //    std::cout << "----------------- Behind -----------------" << std::endl;
    //    std::cout << "Angle Right: " << pcl::rad2deg(angleRight) << std::endl;
    //    std::cout << "Angle Left: " << pcl::rad2deg(angleLeft) << std::endl;

    //    if(angleRight < M_PI/2 && angleLeft < M_PI/2)
    //        std::cout << "The trajector can be behind the landmark.\n" << std::endl;
    //    else
    //        std::cout << "The trajector is not behind the landmark.\n\n\n\n" << std::endl;
}

