#include "CameraCalibrate.h"

//TODO: introduce an unified configure file system

#ifdef ROBOT_NAME_trillian
float CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::NUM_PARAMS] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
#else
#ifdef ROBOT_NAME_marvin
float CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::NUM_PARAMS] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
#else
float CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::NUM_PARAMS] = {0, 0, 0, 0, 0, 0.1, 0, 0, 0};
#endif
#endif

float CameraCalibrate::CALIBRATE_HEAD_MDH_ANGLES[2] = 
    {CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::CALIBRATE_HEAD_PAN],
     CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::CALIBRATE_HEAD_PITCH]};

boost::numeric::ublas::matrix <float> CameraCalibrate::CALIBRATE_HEAD_BASE_TRANSFORMS[1]
    = {CoordFrame4D::translation4D( 0.0f,
       0.0f,
       CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::CALIBRATE_NECK_OFFSET_Z])};
       
boost::numeric::ublas::matrix <float> CameraCalibrate::CALIBRATE_HEAD_END_TRANSFORMS[6]
    = { boost::numeric::ublas::identity_matrix <float> (4),
        boost::numeric::ublas::identity_matrix <float> (4),
        CoordFrame4D::translation4D(CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::CALIBRATE_CAMERA_OFF_X],
                                    CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::CALIBRATE_CAMERA_OFF_Y],
                                    CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::CALIBRATE_CAMERA_OFF_Z]),
        CoordFrame4D::rotation4D(CoordFrame4D::Z_AXIS,
                                 CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::CALIBRATE_CAMERA_PAN_ANGLE]),
        CoordFrame4D::rotation4D(CoordFrame4D::Y_AXIS,
                                 CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::CALIBRATE_CAMERA_PITCH_ANGLE]),
        CoordFrame4D::rotation4D(CoordFrame4D::X_AXIS,
                                 CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::CALIBRATE_CAMERA_ROLL_ANGLE])};

void CameraCalibrate::UpdateWithParams(float params[]){
    //update the parameter array
    for (int i = 0; i < CameraCalibrate::NUM_PARAMS; i++)
        CameraCalibrate::CAMERA_CALIBRATE[i] = params[i];
    //then update all of the transforms based on the new params
    CameraCalibrate::CALIBRATE_HEAD_MDH_ANGLES[0] = 
        CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::CALIBRATE_HEAD_PAN];
    CameraCalibrate::CALIBRATE_HEAD_MDH_ANGLES[1] = 
        CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::CALIBRATE_HEAD_PITCH];
     
     CameraCalibrate::CALIBRATE_HEAD_BASE_TRANSFORMS[0] = 
        CoordFrame4D::translation4D(0.0f, 0.0f, 
            CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::CALIBRATE_NECK_OFFSET_Z]);
    
    CameraCalibrate::CALIBRATE_HEAD_END_TRANSFORMS[2] =
        CoordFrame4D::translation4D(CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::CALIBRATE_CAMERA_OFF_X],
                                    CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::CALIBRATE_CAMERA_OFF_Y],
                                    CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::CALIBRATE_CAMERA_OFF_Z]);
    CameraCalibrate::CALIBRATE_HEAD_END_TRANSFORMS[3] =
        CoordFrame4D::rotation4D(CoordFrame4D::Z_AXIS,
                                 CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::CALIBRATE_CAMERA_PAN_ANGLE]);
    CameraCalibrate::CALIBRATE_HEAD_END_TRANSFORMS[4] =
        CoordFrame4D::rotation4D(CoordFrame4D::Y_AXIS,
                                 CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::CALIBRATE_CAMERA_PITCH_ANGLE]);
    CameraCalibrate::CALIBRATE_HEAD_END_TRANSFORMS[5] =
        CoordFrame4D::rotation4D(CoordFrame4D::X_AXIS,
                                 CameraCalibrate::CAMERA_CALIBRATE[CameraCalibrate::CALIBRATE_CAMERA_ROLL_ANGLE]);
}
