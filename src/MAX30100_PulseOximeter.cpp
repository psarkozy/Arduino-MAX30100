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

#include <Arduino.h>

#include "MAX30100_PulseOximeter.h"


PulseOximeter::PulseOximeter() :
    state(PULSEOXIMETER_STATE_INIT),
    tsFirstBeatDetected(0),
    tsLastBeatDetected(0),
    tsLastBiasCheck(0),
    tsLastCurrentAdjustment(0),
    redLedCurrentIndex((uint8_t)RED_LED_CURRENT_START),
    irLedCurrent(DEFAULT_IR_LED_CURRENT),
    onBeatDetected(NULL)
{
}

bool PulseOximeter::begin(PulseOximeterDebuggingMode debuggingMode_, int SCLpin, int SDApin)
{
    debuggingMode = debuggingMode_;

    bool ready = hrm.begin(SCLpin,SDApin);

    if (!ready) {
        if (debuggingMode != PULSEOXIMETER_DEBUGGINGMODE_NONE) {
            Serial.println("Failed to initialize the HRM sensor");
        }
        return false;
    }

    hrm.setMode(MAX30100_MODE_SPO2_HR);
    hrm.setLedsCurrent(irLedCurrent, (LEDCurrent)redLedCurrentIndex);

    irDCRemover = DCRemover(DC_REMOVER_ALPHA);
    redDCRemover = DCRemover(DC_REMOVER_ALPHA);

    state = PULSEOXIMETER_STATE_IDLE;

    return true;
}

uint8_t PulseOximeter::update()
{
    uint8_t num_samples = hrm.update();

    checkSample();
    checkCurrentBias();
    return num_samples;
}

float PulseOximeter::getHeartRate()
{
    return beatDetector.getRate();
}

uint8_t PulseOximeter::getSpO2()
{
    return spO2calculator.getSpO2();
}

float PulseOximeter::getacSqRatio()
{
    return spO2calculator.getacSqRatio();
}

uint8_t PulseOximeter::getRedLedCurrentBias()
{
    return redLedCurrentIndex;
}

void PulseOximeter::setOnBeatDetectedCallback(void (*cb)())
{
    onBeatDetected = cb;
}

void PulseOximeter::setIRLedCurrent(LEDCurrent irLedNewCurrent)
{
    irLedCurrent = irLedNewCurrent;
    hrm.setLedsCurrent(irLedCurrent, (LEDCurrent)redLedCurrentIndex);
}

void PulseOximeter::shutdown()
{
    hrm.shutdown();
}

void PulseOximeter::resume()
{
    hrm.resume();
}

uint32_t PulseOximeter::discard(){
    uint16_t rawIRValue = 0;
    uint16_t rawRedValue = 0;
    uint32_t timeout = millis() + 100;
    uint32_t discarded_samples = 0;
    // Dequeue all available samples, they're properly timed by the HRM
    discarded_samples = hrm.readFifoData();
    while (hrm.getRawValues(&rawIRValue, &rawRedValue) || (millis() > timeout)){
    };
    if (debuggingMode != PULSEOXIMETER_DEBUGGINGMODE_NONE){
        Serial.print("POX Discarded ");
        Serial.println(discarded_samples);
    }
    return discarded_samples;
}

void PulseOximeter::checkSample()
{
    uint16_t rawIRValue, rawRedValue;
    float irACValue = 0;
    float redACValue = 0;
    float filteredPulseValue = 0;
    bool beatDetected = false;
    int numvaluesread = 0;

    // Dequeue all available samples, they're properly timed by the HRM
    while (hrm.getRawValues(&rawIRValue, &rawRedValue)) {
        numvaluesread ++;
        irACValue = irDCRemover.step(rawIRValue);
        redACValue = redDCRemover.step(rawRedValue);
        // The signal fed to the beat detector is mirrored since the cleanest monotonic spike is below zero
        filteredPulseValue = lpf.step(-irACValue);
        beatDetected = beatDetector.addSample(filteredPulseValue);


        spO2calculator.update(irACValue, redACValue, beatDetected);
        if (beatDetector.getRate() > 0) {
            state = PULSEOXIMETER_STATE_DETECTING;
        } else if (state == PULSEOXIMETER_STATE_DETECTING) {
            //state = PULSEOXIMETER_STATE_IDLE;
            //spO2calculator.reset();
        }

        switch (debuggingMode) {
            case PULSEOXIMETER_DEBUGGINGMODE_RAW_VALUES:
                Serial.print("R:");
                Serial.print(rawIRValue);
                Serial.print(",");
                Serial.println(rawRedValue);
                break;

            case PULSEOXIMETER_DEBUGGINGMODE_AC_VALUES:
                Serial.print("R:");
                Serial.print(irACValue);
                Serial.print(",");
                Serial.println(redACValue);
                break;

            case PULSEOXIMETER_DEBUGGINGMODE_PULSEDETECT:
                Serial.print("R:");
                Serial.print(filteredPulseValue);
                Serial.print(",");
                Serial.println(beatDetector.getCurrentThreshold());
                break;
            case PULSEOXIMETER_DEBUGGINGMODE_FULL:
                Serial.print("R\t");
                Serial.print((int)rawIRValue);
                Serial.print("\t");
                Serial.print((int)rawRedValue);
                Serial.print("\t");
                Serial.print((int)irACValue);
                Serial.print("\t");
                Serial.print((int)redACValue);
                Serial.print("\t");
                Serial.print((int)filteredPulseValue);
                Serial.print("\t");
                Serial.print((int)beatDetector.getCurrentThreshold());
                Serial.print("\t");
                Serial.println((int) 100* spO2calculator.getacSqRatio());
                break;

            default:
                break;
        }

        if (beatDetected && onBeatDetected) {
            onBeatDetected();
        }
    }
}

void PulseOximeter::checkCurrentBias()
{
    // Follower that adjusts the red led current in order to have comparable DC baselines between
    // red and IR leds. The numbers are really magic: the less possible to avoid oscillations

    if (millis() - tsLastBiasCheck > CURRENT_ADJUSTMENT_PERIOD_MS) {
        bool changed = false;
        if (irDCRemover.getDCW() - redDCRemover.getDCW() > 70000 && redLedCurrentIndex < MAX30100_LED_CURR_50MA) {
            ++redLedCurrentIndex;
            changed = true;
        } else if (redDCRemover.getDCW() - irDCRemover.getDCW() > 70000 && redLedCurrentIndex > 0) {
            --redLedCurrentIndex;
            changed = true;
        } 
        if (redDCRemover.getlongtermDC()<4048 && redLedCurrentIndex <15 ) {
            redLedCurrentIndex++;
            changed = true;
        }
        if (redDCRemover.getlongtermDC()>60000 && redLedCurrentIndex > 1 ) {
            redLedCurrentIndex--;
            changed = true;
        }
        if (irDCRemover.getlongtermDC()<4048 && ((uint8_t) irLedCurrent) < 15) {
            irLedCurrent = (LEDCurrent) (((uint8_t) irLedCurrent)+1);
            changed = true;
        }
        if (irDCRemover.getlongtermDC()>60000 && ((uint8_t) irLedCurrent) > 1) {
            irLedCurrent = (LEDCurrent) (((uint8_t) irLedCurrent)-1);
            changed = true;
        }
    

        if (changed) {
            spO2calculator.reset();
            hrm.setLedsCurrent(irLedCurrent, (LEDCurrent)redLedCurrentIndex);
            tsLastCurrentAdjustment = millis();

            if (debuggingMode != PULSEOXIMETER_DEBUGGINGMODE_NONE) {
                Serial.print("Changing LED current to RED_dc:");
                Serial.print(redLedCurrentIndex );
                Serial.print(" IR_dc:");
                Serial.print(irLedCurrent );
                Serial.print(" redDCRemover.getDCW():");
                Serial.print(redDCRemover.getDCW() );
                Serial.print(" irDCRemover.getDCW():");
                Serial.println(irDCRemover.getDCW() );
            }
        }

        tsLastBiasCheck = millis();
    }
}
