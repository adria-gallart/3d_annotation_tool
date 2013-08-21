#ifndef QSR_H
#define QSR_H

#include <pcl/point_types.h>
#include "objectsinformation.h"

class qsr
{
public:
    qsr(std::vector<object> objectList);

    void calculateQSROnRight();

    void calculateQSROnLeft();

    void calculateQSRInFront();

    void calculateQSRBehind();

    void test();

private:
    pcl::PointXYZ getCenterOfMass(object obj);

    void calculatePointsLandmark(object landmark);

    void calculatePointTrajector(object trajector);

    void calculateAnglesAndDistanceOnRight();

    void calculateAnglesAndDistanceOnLeft();

    void calculateAnglesAndDistanceInFront();

    void calculateAnglesAndDistanceBehind();

    float angleFunction(float angle);

    float distanceFunction(float distance, float perimeter);

    float qsrOnRight();

    float qsrOnLeft();

    float qsrInFront();

    float qsrBehind();

    pcl::PointXYZ _FLDPoint, _FRDPoint, _BRDPoint, _BLDPoint;
    pcl::PointXYZ _FCPoint, _RCPoint, _BCPoint, _LCPoint;
    pcl::PointXYZ _centerOfMassTrajector;

    float _angleRightOnRight, _angleLeftOnRight, _distanceOnRight;
    float _angleRightOnLeft, _angleLeftOnLeft, _distanceOnLeft;
    float _angleRightInFront, _angleLeftInFront, _distanceInFront;
    float _angleRightBehind, _angleLeftBehind, _distanceBehind;
    float _distanceThreshold;

    std::vector<object> _objectList;
};

#endif // QSR_H
