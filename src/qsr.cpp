#include "qsr.h"

#include <math.h>
#include <iomanip>

#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/common/angles.h>

qsr::qsr(std::vector<object> objectList)
{
    _objectList = objectList;
    _distanceThreshold = 1.5;
}

void qsr::calculateQSROnRight(){

    float values[_objectList.size()][_objectList.size()];

    // Calculate values
    for(int i=0; i < _objectList.size(); i++){
        calculatePointsLandmark(_objectList[i]);
        for(int j=0; j < _objectList.size(); j++){
            if(j == i) values[j][i] = -1;
            else{
                calculatePointTrajector(_objectList[j]);
                values[j][i] = qsrOnRight();
            }
        }
    }

    // Print the table
    std::cout << "------------------------------------------------" << std::endl;
    std::cout << "----------On Right of---------------------------" << std::endl;
    std::cout << "------------------------------------------------\n" << std::endl;

    std::cout << std::setw(10) << "" << std::ends;
    for(int i = 0; i < _objectList.size(); i++){
        std::cout << std::setw(10) << _objectList[i].name.toStdString() << std::ends;
    }
    std::cout << std::endl;

    for(int i = 0; i < _objectList.size(); i++){
        std::cout << std::setw(10) << std::left << _objectList[i].name.toStdString() << std::ends;
        for(int j = 0; j < _objectList.size(); j++){
            if(i==j)std::cout << std::setw(10) << std::right << "X" << std::ends;
            else std::cout << std::setw(10) << std::right << std::setprecision(2)  << values[i][j] << std::ends;
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void qsr::calculateQSROnLeft(){

    float values[_objectList.size()][_objectList.size()];

    // Calculate values
    for(int i=0; i < _objectList.size(); i++){
        calculatePointsLandmark(_objectList[i]);
        for(int j=0; j < _objectList.size(); j++){
            if(j == i) values[j][i] = -1;
            else{
                calculatePointTrajector(_objectList[j]);
                values[j][i] = qsrOnLeft();
            }
        }
    }

    // Print the table
    std::cout << "------------------------------------------------" << std::endl;
    std::cout << "----------On Left of----------------------------" << std::endl;
    std::cout << "------------------------------------------------\n" << std::endl;

    std::cout << std::setw(10) << "" << std::ends;
    for(int i = 0; i < _objectList.size(); i++){
        std::cout << std::setw(10) << _objectList[i].name.toStdString() << std::ends;
    }
    std::cout << std::endl;

    for(int i = 0; i < _objectList.size(); i++){
        std::cout << std::setw(10) << std::left << _objectList[i].name.toStdString() << std::ends;
        for(int j = 0; j < _objectList.size(); j++){
            if(i==j)std::cout << std::setw(10) << std::right << "X" << std::ends;
            else std::cout << std::setw(10) << std::setprecision(2) << std::right << values[i][j] << std::ends;
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void qsr::calculateQSRInFront(){

    float values[_objectList.size()][_objectList.size()];

    // Calculate values
    for(int i=0; i < _objectList.size(); i++){
        calculatePointsLandmark(_objectList[i]);
        for(int j=0; j < _objectList.size(); j++){
            if(j == i) values[j][i] = -1;
            else{
                calculatePointTrajector(_objectList[j]);
                values[j][i] = qsrInFront();
            }
        }
    }

    // Print the table
    std::cout << "------------------------------------------------" << std::endl;
    std::cout << "----------In Front of---------------------------" << std::endl;
    std::cout << "------------------------------------------------\n" << std::endl;

    std::cout << std::setw(10) << "" << std::ends;
    for(int i = 0; i < _objectList.size(); i++){
        std::cout << std::setw(10) << _objectList[i].name.toStdString() << std::ends;
    }
    std::cout << std::endl;

    for(int i = 0; i < _objectList.size(); i++){
        std::cout << std::setw(10) << std::left << _objectList[i].name.toStdString() << std::ends;
        for(int j = 0; j < _objectList.size(); j++){
            if(i==j)std::cout << std::setw(10) << std::right << "X" << std::ends;
            else std::cout << std::setw(10) << std::right << std::setprecision(2)  << values[i][j] << std::ends;
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void qsr::calculateQSRBehind(){

    float values[_objectList.size()][_objectList.size()];

    // Calculate values
    for(int i=0; i < _objectList.size(); i++){
        calculatePointsLandmark(_objectList[i]);
        for(int j=0; j < _objectList.size(); j++){
            if(j == i) values[j][i] = -1;
            else{
                calculatePointTrajector(_objectList[j]);
                values[j][i] = qsrBehind();
            }
        }
    }

    // Print the table
    std::cout << "------------------------------------------------" << std::endl;
    std::cout << "----------Behind of-----------------------------" << std::endl;
    std::cout << "------------------------------------------------\n" << std::endl;

    std::cout << std::setw(10) << "" << std::ends;
    for(int i = 0; i < _objectList.size(); i++){
        std::cout << std::setw(10) << _objectList[i].name.toStdString() << std::ends;
    }
    std::cout << std::endl;

    for(int i = 0; i < _objectList.size(); i++){
        std::cout << std::setw(10) << std::left << _objectList[i].name.toStdString() << std::ends;
        for(int j = 0; j < _objectList.size(); j++){
            if(i==j)std::cout << std::setw(10) << std::right << "X" << std::ends;
            else std::cout << std::setw(10) << std::right << std::setprecision(2)  << values[i][j] << std::ends;
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
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

void qsr::calculateAnglesAndDistanceOnRight(){
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

    // Calculate the distance
    _distanceOnRight = pcl::euclideanDistance(_centerOfMassTrajector, _RCPoint);
}

void qsr::calculateAnglesAndDistanceOnLeft(){
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

    // Calculate the distance
    _distanceOnLeft = pcl::euclideanDistance(_centerOfMassTrajector, _LCPoint);
}

void qsr::calculateAnglesAndDistanceInFront(){
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

    // Calculate the distance
    _distanceInFront = pcl::euclideanDistance(_centerOfMassTrajector, _FCPoint);
}

void qsr::calculateAnglesAndDistanceBehind(){
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

    // Calculate the distance
    _distanceBehind = pcl::euclideanDistance(_centerOfMassTrajector, _BCPoint);
}

// Arreglar funcions al quadrat, etc.
float qsr::angleFunction(float angle){
    if(angle >= 0 && angle < M_PI_2) return 1.0;
    else if(angle >= M_PI_2 && angle < 2*M_PI/3) return 1-(36/(M_PI*M_PI))*(angle - M_PI_2)*(angle - M_PI_2);
    else if(angle >= 2*M_PI/3 && angle < 11*M_PI/6) return 0.0;
    else return 1 - (36/(M_PI*M_PI))*(angle - 2*M_PI)*(angle - 2*M_PI);
}

float qsr::distanceFunction(float distance, float perimeter){
    if(distance <= perimeter/2) return 1.0;
    else if(distance <= 3*perimeter/4) return 1-(8/pow(perimeter,2))*(pow(distance-perimeter/2,2));
    else return 0.5*pow((distance-3*perimeter/4)+1,-5);


    //    if x <= width/2
    //      y = 1;
    //    elseif x <= 3*width/4
    //        y = 1 - (8/(width^2))*((x-width/2)^2);
    //    else
    //      y = 0.5*((x-3*width/4)+1)^-0.3;
}

float qsr::qsrOnRight(){
    calculateAnglesAndDistanceOnRight();
    return angleFunction(_angleRightOnRight)*angleFunction(_angleLeftOnRight)*distanceFunction(_distanceOnRight, _distanceThreshold);
}

float qsr::qsrOnLeft(){
    calculateAnglesAndDistanceOnLeft();
    return angleFunction(_angleRightOnLeft)*angleFunction(_angleLeftOnLeft)*distanceFunction(_distanceOnLeft, _distanceThreshold);
}

float qsr::qsrInFront(){
    calculateAnglesAndDistanceInFront();
    return angleFunction(_angleRightInFront)*angleFunction(_angleLeftInFront)*distanceFunction(_distanceInFront, _distanceThreshold);
}

float qsr::qsrBehind(){
    calculateAnglesAndDistanceBehind();
    return angleFunction(_angleRightBehind)*angleFunction(_angleLeftBehind)*distanceFunction(_distanceBehind, _distanceThreshold);
}

void qsr::test(){
    //    float angle2;
    //    for(float angle=0.0; angle <= 2*M_PI; angle+=M_PI_4/30){
    //        if(pcl::rad2deg(angle) < 180) std::cout << pcl::rad2deg(angle) << " " << angleFunction(angle) << std::endl;
    //        else std::cout << pcl::rad2deg(angle) - 360 << " " << angleFunction(angle) << std::endl;
    //    }

    for(float distance = 0.0; distance <= 2; distance+=0.01){
        std::cout << distance << " " << distanceFunction(distance, 0.6) << std::endl;
    }
}
