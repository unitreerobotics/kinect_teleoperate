#pragma once

#include <k4abttypes.h>
#include <chrono>
#include <iostream>

using namespace std::chrono;

// To detect the start and stop actions of the teleoperated robot
class StartEndPoseDetector {
public:
    StartEndPoseDetector() : state(false), last_check_time(high_resolution_clock::now()), pose_duration(0), loop_counter(0) {}

    bool isStartEndPose(double ls_r, double ls_p, double ls_y,
                        double rs_r, double rs_p, double rs_y,
                        double le_y, double re_y) {

        ++loop_counter;
        if (loop_counter % 1000 == 0) {
            auto now = high_resolution_clock::now();
            auto elapsed_time = duration_cast<microseconds>(now - last_check_time).count();
            last_check_time = now;

            bool in_range = (ls_r > -0.5 && ls_r < 0.5) && (ls_p > -0.5 && ls_p < 0.5) && (ls_y > -0.5 && ls_y < 0.5) &&
                            (rs_r > -0.5 && rs_r < 0.5) && (rs_p > -0.5 && rs_p < 0.5) && (rs_y > -0.5 && rs_y < 0.5) &&
                            (le_y > -0.5 && le_y < 0.5) && (re_y > -0.5 && re_y < 0.5);

            if (in_range) {
                pose_duration += elapsed_time;
                if (pose_duration > 3000000) {
                    flipState();
                }
            } else {
                resetTime();
            }

            loop_counter = 0;
        }

        return state;
    }

private:
    bool state;
    time_point<high_resolution_clock> last_check_time;
    long long pose_duration;
    int loop_counter;

    void flipState() {
        state = !state;
        resetTime();
        std::cout << (state ? " START." : " END.") << std::endl;
    }

    void resetTime() {
        pose_duration = 0;
        last_check_time = high_resolution_clock::now();
    }
};