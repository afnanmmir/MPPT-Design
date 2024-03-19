/**
 * @file simualted_annealing.hpp
 * @author Praneel Murali (praneelm@utexas.edu)
 * @brief Simulated Annealing MPPT algorithm driver.
 * @version 0.2.0
 * @date 2024-01-20
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once
#include "../mppt.hpp"
#include "../local/pando.hpp"
#include <cmath>
#include <random>

#define MAX_INP_VOLT 70.0
#define INITIAL_TEMP 10.0
#define COOLING_RATE 0.99
#define NUM_ITERATIONS 100

class SimulatedAnnealing final : public MPPT {
public:
    SimulatedAnnealing(void) : MPPT() {
        prev_array_voltage = 0.0;
        prev_array_power = 0.0;
        temperature = INITIAL_TEMP;
        pandO = new PandO();
        gen = std::mt19937(std::random_device{}());
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
        if (temperature > 0.1) {
            float current_power = array_voltage * array_current;
            float best_power = current_power;
            float best_voltage = array_voltage;

            for (int i = 0; i < NUM_ITERATIONS; i++) {
                float new_voltage = array_voltage + get_random_value(-1.0, 1.0);
                new_voltage = std::max(0.0f, std::min(new_voltage, MAX_INP_VOLT));
                float new_power = new_voltage * array_current;

                if (new_power > best_power) {
                    best_power = new_power;
                    best_voltage = new_voltage;
                } else {
                    float acceptance_probability = std::exp((new_power - current_power) / temperature);
                    if (get_random_value(0.0, 1.0) < acceptance_probability) {
                        best_power = new_power;
                        best_voltage = new_voltage;
                    }
                }
            }

            reference_voltage = best_voltage;
            temperature *= COOLING_RATE;
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
        temperature = INITIAL_TEMP;
        pandO->reset_state();
    }

protected:
    float array_voltage;
    float array_current;
    float battery_voltage;
    float battery_current;

    float prev_array_voltage;
    float prev_array_power;
    float temperature;
    PandO* pandO;
    std::mt19937 gen;

private:
    float get_random_value(float min, float max) {
        std::uniform_real_distribution<float> dist(min, max);
        return dist(gen);
    }
};