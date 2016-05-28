/*
Arduino-MAX30100 oximetry / heart rate integrated sensor library
Copyright (C) 2016  OXullo Intersecans <x@brainrapers.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PULSEDETECTOR_H
#define PULSEDETECTOR_H

#include <stdint.h>

#define PULSEDETECTOR_INIT_HOLDOFF                2000    // in ms, how long to wait before counting
#define PULSEDETECTOR_MASKING_HOLDOFF             200     // in ms, non-retriggerable window after beat detection
#define PULSEDETECTOR_BPFILTER_ALPHA              0.5     // EMA factor for the beat period value
#define PULSEDETECTOR_MIN_THRESHOLD               30      // minimum threshold (filtered) value
#define PULSEDETECTOR_MAX_THRESHOLD               500     // maximumg threshold (filtered) value
#define PULSEDETECTOR_THRESHOLD_FALLOFF_TARGET    0.3     // thr chasing factor of the max value when beat
#define PULSEDETECTOR_THRESHOLD_DECAY_FACTOR      0.99    // thr chasing factor when no beat
#define PULSEDETECTOR_INVALID_READOUT_DELAY       2000    // in ms, no-beat time to cause a reset
#define PULSEDETECTOR_SAMPLES_PERIOD              10      // in ms, 1/Fs


typedef enum PulseDetectorState {
    PULSEDETECTOR_STATE_INIT,
    PULSEDETECTOR_STATE_WAITING,
    PULSEDETECTOR_STATE_FOLLOWING_SLOPE,
    PULSEDETECTOR_STATE_MASKING
} PulseDetectorState;


class PulseDetector
{
public:
    PulseDetector();
    void addSample(float sample);
    float getHeartRate();
    float getCurrentThreshold();

private:
    void checkForBeat(float value);
    void decreaseThreshold();

    PulseDetectorState state;
    float threshold;
    float beatPeriod;
    float lastMaxValue;
    uint32_t tsLastPulse;
};

#endif
