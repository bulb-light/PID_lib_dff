#ifndef __PID_H__
#define __PID_H__

#include <Arduino.h>

// Digital implementation of the PID controller
// using a zero order hold
class PID {
    private:
        // pid errors
        float _error_1prev = 0.0f;
        float _error_2prev = 0.0f;
        float _error_current = 0.0f;
        float _pid_out_1prev = 0.0f;
        float _T = 0.0f; // sampling time seconds

        // limits
        float _max_pid = INFINITY;
        float _min_pid = -INFINITY;

    public:
        // kp, ki and kd are from continuos pid
        PID(float kp, float ki, float kd, float sampling_time, float min_limit, float max_limit);
        ~PID();

        // pid params
        float _kp = 0.0f;
        float _ki = 0.0f;
        float _kd = 0.0f;

        // methods
        float pid_out(float error);
        void reset();
};

#endif