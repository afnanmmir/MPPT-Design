/**
 * @file firefly.hpp
 * @author Praneel Murali (praneelm@utexas.edu)
 * @brief firefly MPPT algorithm driver.
 * @version 0.2.0
 * @date 2024-01-20
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#pragma once
#include "../mppt.hpp"
#include <vector>
#include <random>
#include <algorithm>

class FireflyAlgorithm : public MPPT {
public:
    FireflyAlgorithm(unsigned int populationSize = 25, float gamma = 1.0f, float beta0 = 2.0f, float alpha = 0.3f)
    : populationSize(populationSize), gamma(gamma), beta0(beta0), alpha(alpha), distribution(-1.0f, 1.0f) {
        engine.seed(std::random_device{}());
        initialize_population();
    }

    void input_context(void *args) override {
        float* argArray = static_cast<float*>(args);
        array_voltage = argArray[0];
        array_current = argArray[1];
        current_power = array_voltage * array_current;
    }

    void step_algorithm(void) override {
        for (size_t i = 0; i < population.size(); ++i) {
            for (size_t j = 0; j < population.size(); ++j) {
                if (population[j].intensity > population[i].intensity) {
                    move_firefly(i, j);
                }
            }
        }

        for (auto& firefly : population) {
            float step = distribution(engine) * alpha;
            firefly.voltage += step;
            firefly.intensity = calculate_intensity(firefly.voltage);
        }
    }

    void reset_state(void) override {
        initialize_population();
    }

protected:
    struct Firefly {
        float voltage;
        float intensity;

        Firefly(float v, float i) : voltage(v), intensity(i) {}
    };

    std::vector<Firefly> population;
    unsigned int populationSize;

    float gamma;
    float beta0;
    float alpha;

    std::mt19937 engine;
    std::uniform_real_distribution<float> distribution;

    float array_voltage;
    float array_current;
    float current_power;

private:
    void initialize_population() {
        population.clear();
        for (unsigned int i = 0; i < populationSize; ++i) {
            float voltage = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 100.0f;
            float intensity = calculate_intensity(voltage);
            population.emplace_back(voltage, intensity);
        }
    }

    float calculate_intensity(float voltage) {
        float power = voltage * array_current;
        return power;
    }

    void move_firefly(int i, int j) {
        float distance = std::abs(population[i].voltage - population[j].voltage);
        float attractiveness = beta0 * std::exp(-gamma * distance * distance);
        population[i].voltage += attractiveness * (population[j].voltage - population[i].voltage) + alpha * (distribution(engine) - 0.5);
        population[i].intensity = calculate_intensity(population[i].voltage);
    }
};
