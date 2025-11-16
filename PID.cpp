// SPDX-FileCopyrightText: 2025 David Chirme Sisa ([dff-laoise](https://github.com/bulb-light)) 
// SPDX-License-Identifier: MIT

#include "PID.h"

PID::PID(float Kp, float Ti, float Td, float sampleTimeSec, float minOutput, float maxOutput)
    :   mKp(0.0f), mTi(0.0001f), mTd(0.0f),
        mSampleTimeSec(0.01f), mMinOutput(0.0f), mMaxOutput(255.0f), mWasSaturated(false),
        Kpd(0.0f), Kid(0.0f), Kdd(0.0f),
        lastError1(0.0f), lastOutput1(0.0f),
        integralAccumulator(0.0f)
{
    this->setSampleTime(sampleTimeSec);
    this->setPIDParams(Kp, Ti, Td);
    this->setOutputLimits(minOutput, maxOutput);

    this->updatePIDCoefficients();

    this->resetStates();
}

PID::~PID()
{
}

void PID::setSampleTime(float sampleTimeSec)
{
    // check and set new sample time, if invalid set to default 10ms
    if (sampleTimeSec <= 0.0f)
    {
        this->mSampleTimeSec = 0.01f; // default to 10ms if invalid
    } else {
        this->mSampleTimeSec = sampleTimeSec;
    }

    this->updatePIDCoefficients();
}

void PID::setPIDParams(float Kp, float Ti, float Td)
{
    // set new PID parameters
    this->mKp = Kp;

    // check and set Ti to avoid division by zero
    // if Ti == 0, set to small value 0.1ms
    if (Ti == 0.0f)
    {
        this->mTi = 0.0001f; // small value to avoid division by zero 0.1ms
    } else {
        this->mTi = Ti;
    }

    this->mTd = Td;

    this->updatePIDCoefficients();
}

void PID::setOutputLimits(float minOutput, float maxOutput)
{
    // check and set new output limits
    // if minOutput >= maxOutput, set to default (0.0, 255.0)
    if (minOutput >= maxOutput)
    {
        this->mMinOutput = 0.0f;
        this->mMaxOutput = 255.0f; // default limits
    } else {
        this->mMinOutput = minOutput;
        this->mMaxOutput = maxOutput;
    }
}

void PID::updatePIDCoefficients()
{
    float Ts = this->mSampleTimeSec;

    // discrete PID cofficients
    this->Kpd = this->mKp;
    this->Kid = (this->mKp * Ts) / (2 * this->mTi);
    this->Kdd = (this->mKp * this->mTd) / Ts;
}

float PID::computePIDOut(float error)
{
    // Discrete PID controller using trapezoidal approximation for integral
    // and backward-difference for derivative

    // Compute PID output
    // this->integralAccumulator +=  (this->lastError1 + error);
    float predictedAccumulator = this->integralAccumulator + (this->lastError1 + error);

    float output = this->Kpd * error
                 + this->Kid * predictedAccumulator
                 + this->Kdd * (error - this->lastError1);

    // Anti-windup: Check for saturation (clamp output)
    if (output > this->mMaxOutput)
    {
        output = this->mMaxOutput;
        this->mWasSaturated = true;
    }
    else if (output < this->mMinOutput)
    {
        output = this->mMinOutput;
        this->mWasSaturated = true;
    }
    else
    {
        this->mWasSaturated = false;
    }

    // only update integral accumulator if not saturated
    if (!this->mWasSaturated)
    {
        this->integralAccumulator = predictedAccumulator;
    }

    // Update internal states
    this->lastError1 = error;
    this->lastOutput1 = output;

    return output; // Placeholder return value
}

void PID::resetStates()
{
    // reset internal states
    this->lastError1 = 0.0f;
    this->lastOutput1 = 0.0f;

    // this->integralAccumulator = 0.0f;
}