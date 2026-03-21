// SPDX-FileCopyrightText: 2025 David Chirme Sisa ([dff-laoise](https://github.com/bulb-light)) 
// SPDX-License-Identifier: MIT

#include "PID.h"

PID::PID(float Kp, float Ti, float Td, float sampleTimeSec, float minOutput, float maxOutput)
    :   mKp(0.0f), mTi(0.0001f), mTd(0.0f),
        mSampleTimeSec(0.01f), mMinOutput(0.0f), mMaxOutput(255.0f), mWasSaturated(false),
        mKpd(0.0f), mKid(0.0f), mKdd(0.0f),
        mLastError1(0.0f),
        mIntegralAccumulator(0.0f)
{
    setSampleTime(sampleTimeSec);
    setPIDParams(Kp, Ti, Td);
    setOutputLimits(minOutput, maxOutput);

    updatePIDCoefficients();

    resetStates();
}

PID::~PID()
{
}

void PID::setSampleTime(float sampleTimeSec)
{
    // check and set new sample time, if invalid set to default 10ms
    if (sampleTimeSec <= 0.0f)
    {
        mSampleTimeSec = 0.01f; // default to 10ms if invalid
    } else {
        mSampleTimeSec = sampleTimeSec;
    }

    updatePIDCoefficients();
}

void PID::setPIDParams(float Kp, float Ti, float Td)
{
    // set new PID parameters
    mKp = Kp;

    // check and set Ti to avoid division by zero
    // if Ti == 0, set to small value 0.1ms
    if (Ti == 0.0f)
    {
        mTi = 0.0001f; // small value to avoid division by zero 0.1ms
    } else {
        mTi = Ti;
    }

    mTd = Td;

    updatePIDCoefficients();
}

void PID::setOutputLimits(float minOutput, float maxOutput)
{
    // check and set new output limits
    // if minOutput >= maxOutput, set to default (0.0, 255.0)
    if (minOutput >= maxOutput)
    {
        mMinOutput = 0.0f;
        mMaxOutput = 255.0f; // default limits
    } else {
        mMinOutput = minOutput;
        mMaxOutput = maxOutput;
    }
}

void PID::updatePIDCoefficients()
{
    float Ts = mSampleTimeSec;

    // discrete PID cofficients
    mKpd = mKp;
    mKid = (mKp * Ts) / (2 * mTi);
    mKdd = (mKp * mTd) / Ts;
}

float PID::computePIDOut(float error)
{
    // Discrete PID controller using trapezoidal approximation for integral
    // and backward-difference for derivative

    // Compute PID output
    // this->integralAccumulator +=  (this->lastError1 + error);
    float predictedAccumulator = mIntegralAccumulator + (mLastError1 + error);

    float output = mKpd * error
                 + mKid * predictedAccumulator
                 + mKdd * (error - mLastError1);

    // Anti-windup: Check for saturation (clamp output)
    if (output > mMaxOutput)
    {
        output = mMaxOutput;
        mWasSaturated = true;
    }
    else if (output < mMinOutput)
    {
        output = mMinOutput;
        mWasSaturated = true;
    }
    else
    {
        mWasSaturated = false;
    }

    // only update integral accumulator if not saturated
    if (!mWasSaturated)
    {
        mIntegralAccumulator = predictedAccumulator;
    }

    // Update internal states
    mLastError1 = error;
    // mLastOutput1 = output;

    return output; // Placeholder return value
}

void PID::resetStates()
{
    // reset internal states
    mLastError1 = 0.0f;
    // mLastOutput1 = 0.0f;

    // this->mIntegralAccumulator = 0.0f;
}