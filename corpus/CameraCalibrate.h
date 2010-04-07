#ifndef _CameraCalibrate_h_DEFINED
#define _CameraCalibrate_h_DEFINED

#include <boost/numeric/ublas/matrix.hpp>
#include "CoordFrame.h"

class CameraCalibrate{

public:
    enum CameraCalibrateConsts {
        CALIBRATE_CAMERA_ROLL_ANGLE = 0,
        CALIBRATE_CAMERA_PITCH_ANGLE,
        CALIBRATE_CAMERA_PAN_ANGLE,
        CALIBRATE_CAMERA_OFF_X,
        CALIBRATE_CAMERA_OFF_Y,
        CALIBRATE_CAMERA_OFF_Z,
        CALIBRATE_HEAD_PAN,
        CALIBRATE_HEAD_PITCH,
        CALIBRATE_NECK_OFFSET_Z
        };

    static const int NUM_PARAMS = 9;

    static float CAMERA_CALIBRATE[NUM_PARAMS];
    static float CALIBRATE_HEAD_MDH_ANGLES[2];
    static boost::numeric::ublas::matrix <float> CALIBRATE_HEAD_BASE_TRANSFORMS[1];
    static boost::numeric::ublas::matrix <float> CALIBRATE_HEAD_END_TRANSFORMS[6];

    static void UpdateWithParams(float params[]);

    };


#endif