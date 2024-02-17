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
#include "math.h"

class SimulatedAnnealing final : public MPPT {
public:
    SimulatedAnnealing(void) : MPPT(), 
        engine(std::random_device{}()), 
        distribution(-max_voltage_step, max_voltage_step) {
        current_temperature = initial_temperature;
    }

    void input_context(void *args) override {
        float* argArray = static_cast<float*>(args);
        array_voltage = argArray[0];
        array_current = argArray[1];
    }

    void step_algorithm(void) override {
        if (current_temperature > final_temperature) {
            float new_voltage = generate_new_voltage(reference_voltage);
            float new_power = calculate_power(new_voltage, array_current);

            if (new_power > max_power) {
                max_power = new_power;
                reference_voltage = new_voltage;
            } else {
                float delta_power = new_power - max_power;
                if (exp(delta_power / current_temperature) > distribution(engine)) {
                    reference_voltage = new_voltage;
                }
            }

            current_temperature *= cooling_rate;
        }
    }

    void reset_state(void) override {
        MPPT::reset_state();
        current_temperature = initial_temperature;
        max_power = 0.0;
    }

protected:
    float array_voltage;
    float array_current;
    float max_power = 0.0;

    float initial_temperature = 25.0;
    float final_temperature = 0.4;
    float cooling_rate = 0.75;
    float current_temperature;

    std::mt19937 engine;
    std::uniform_real_distribution<float> distribution;

private:
    float generate_new_voltage(float current_voltage) {
        float voltage_variation = distribution(engine);
        return current_voltage + voltage_variation;
    }

    float calculate_power(float voltage, float current) {
        return voltage * current;
    }

    static constexpr float max_voltage_step = 0.5;
};