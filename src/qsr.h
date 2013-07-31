#ifndef QSR_H
#define QSR_H

#include <pcl/point_types.h>
#include "objectsinformation.h"

class qsr
{
public:
    qsr();

private:
    pcl::PointXYZ getCenterOfMass(object obj);

    void calculatePointsLandmark(object landmark);

    void calculatePointTrajector(object trajector);

    void calculateAnglesOnRight();

    void calculateAnglesOnLeft();

    void calculateAnglesInFront();

    void calculateAnglesBehind();


    pcl::PointXYZ _FLDPoint, _FRDPoint, _BRDPoint, _BLDPoint;
    pcl::PointXYZ _FCPoint, _RCPoint, _BCPoint, _LCPoint;
    pcl::PointXYZ _centerOfMassTrajector;

    float _angleRightOnRight, _angleLeftOnRight;
    float _angleRightOnLeft, _angleLeftOnLeft;
    float _angleRightInFront, _angleLeftInFront;
    float _angleRightBehind, _angleLeftBehind;
};

#endif // QSR_H
