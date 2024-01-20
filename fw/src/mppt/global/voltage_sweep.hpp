/**
 * @file voltage_sweep.hpp
 * @author Afnan Mir (afnanmir@utexas.edu)
 * @brief Voltage Sweep MPPT algorithm driver.
 * @version 0.2.0
 * @date 2024-01-20
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once
#include "../mppt.hpp"
#include "../local/pando.hpp"

#define MAX_INP_VOLT 70.0
#define STRIDE 0.1
#define WINDOW_SIZE 10

class VoltageSweep final : public MPPT {
    public:
        VoltageSweep(void) : MPPT() {
            prev_array_voltage = 0.0;
            prev_array_power = 0.0;
            sweeping = false;
            increasing = false;
            index = 0;
            for (int i = 0; i < WINDOW_SIZE; i++) {
                voltage_peaks[i] = 0.0;
                power_peaks[i] = 0.0;
            }
        }

        void input_context(void *args) {
            #define INPUT_VOLTAGE ((float *) args)[0]
            #define INPUT_CURRENT ((float *) args)[1]
            #define OUTPUT_VOLTAGE ((float *) args)[2]
            #define OUTPUT_CURRENT ((float *) args)[3]

            array_voltage = INPUT_VOLTAGE;
            array_current = INPUT_CURRENT;
            battery_voltage = OUTPUT_VOLTAGE;
            battery_current = OUTPUT_CURRENT;
            pandO->input_context(args);

            #undef INPUT_VOLTAGE
            #undef INPUT_CURRENT
            #undef OUTPUT_VOLTAGE
            #undef OUTPUT_CURRENT
        }

        void step_algorithm(void) {
            // if sweeping
            if (sweeping) {
                sweep();
                reference_voltage += STRIDE;
                if (reference_voltage > MAX_INP_VOLT) {
                    sweeping = false;
                    reference_voltage = get_starting_point();
                }
            } else {
                reference_voltage = pandO->step_algorithm();
            }
            prev_array_power = array_voltage * array_current;
            prev_array_voltage = array_voltage;
        }

        void reset_state(void) {
            reference_voltage = 0.0;
            prev_array_voltage = 0.0;
            prev_array_power = 0.0;
            sweeping = false;
            increasing = false;
            index = 0;
            for (int i = 0; i < WINDOW_SIZE; i++) {
                voltage_peaks[i] = 0.0;
                power_peaks[i] = 0.0;
            }
            pandO->reset_state();
        }
        
    protected:
        /** Required inputs. */
        float array_voltage;
        float array_current;
        float battery_voltage;
        float battery_current;

        /** Saved internal data. */
        float prev_array_voltage;
        float prev_array_power;
        float voltage_peaks[WINDOW_SIZE];
        float power_peaks[WINDOW_SIZE];
        int index;
        
        /** Algorithm properties */
        bool sweeping;
        bool increasing;
        PandO* pandO;

    private:
        /** Private helper functions. */
        void sweep(void) {
            float array_power = array_voltage * array_current;

            if (array_power < prev_array_power && increasing) {
                increasing = false;
                voltage_peaks[index] = prev_array_voltage;
                power_peaks[index] = prev_array_power;
                index += 1;
            } else if (array_power >= prev_array_power && !increasing) {
                increasing = true;
            }
        }
        float get_starting_point(void) {
            float max_power = 0.0;
            float max_voltage = 0.0;
            for (int i = 0; i < WINDOW_SIZE; i++) {
                if (power_peaks[i] > max_power) {
                    max_power = power_peaks[i];
                    max_voltage = voltage_peaks[i];
                }
            }
            return max_voltage;
        }
};
