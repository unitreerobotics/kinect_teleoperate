#pragma once

#include <cmath>

// This file contains the retargeting function from the skeleton joint angles to the robot motor joint angles.
// ':=' means that the item on the left hand side is being defined to be what is on the right hand side.
// L := Left, R := Right, S := Shoulder, E := Elbow, C := Chest

double LS_mappingCameraYaw2RobotRadians(double camera_yaw) {
    if(camera_yaw <= 0.0)
        return camera_yaw + M_PI;
    else
        return camera_yaw - M_PI;
}
double LS_mappingCameraRoll2RobotRadians(double camera_roll) {
    return camera_roll;
}
double LS_mappingCameraPitch2RobotRadians(double camera_pitch) {
    if(camera_pitch <= 0.0)
        return camera_pitch + M_PI;
    else
        return camera_pitch - M_PI;
}

double RS_mappingCameraYaw2RobotRadians(double camera_yaw) {
    return camera_yaw;

}
double RS_mappingCameraPitch2RobotRadians(double camera_pitch) {
    if(camera_pitch <= 0.0)
        return std::abs(camera_pitch) - M_PI;
    else
        return M_PI - camera_pitch;
}
double RS_mappingCameraRoll2RobotRadians(double camera_roll) {
    return -camera_roll;
}

double LE_mappingCameraYaw2RobotRadians(double camera_yaw) {
    return (M_PI / 2.0) - camera_yaw;
}
double RE_mappingCameraYaw2RobotRadians(double camera_yaw) {
    return (M_PI / 2.0) - camera_yaw;
}

// double SC_mappingCameraPitch2RobotTorso(double camera_pitch){
//     return ((M_PI / 2.0) - std::abs(camera_pitch));
// }
// Use a power function to smooth the steering angle near 0, making the spine chest turn look more natural visually?
double SC_mappingCameraPitch2RobotTorso(double camera_pitch) {
    double center = M_PI / 2.0;
    double offset = std::abs(camera_pitch) - center;
    double exponent = 10.0;
    double sign = offset >= 0 ? -1.0 : 1.0;
    double normalized_offset = std::pow(std::abs(offset) / center, exponent) * sign;
    return normalized_offset * (M_PI / 2.0);
}