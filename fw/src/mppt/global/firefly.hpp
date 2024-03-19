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
#include "../local/pando.hpp"
#include <cmath>
#include <random>
#include <vector>

#define MAX_INP_VOLT 70.0
#define NUM_FIREFLIES 20
#define ALPHA 1.0       // Attractiveness coefficient
#define BETA 1.0        // Attractiveness update coefficient
#define GAMMA 1.0       // Absorption coefficient
#define NUM_ITERATIONS 100

class FireflyAlgorithm final : public MPPT {
public:
   FireflyAlgorithm(void) : MPPT() {
       prev_array_voltage = 0.0;
       prev_array_power = 0.0;
       pandO = new PandO();
       gen = std::mt19937(std::random_device{}());
       initialize_fireflies();
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
       float global_best_power = -1.0f;
       float global_best_voltage = 0.0f;

       for (int iter = 0; iter < NUM_ITERATIONS; iter++) {
           update_firefly_brightness();

           for (int i = 0; i < NUM_FIREFLIES; i++) {
               for (int j = 0; j < NUM_FIREFLIES; j++) {
                   if (fireflies[j].brightness > fireflies[i].brightness) {
                       float distance = std::abs(fireflies[j].voltage - fireflies[i].voltage);
                       float attractiveness = fireflies[j].brightness * std::exp(-GAMMA * distance * distance);
                       float rand_val = get_random_value(0.0, 1.0);
                       float new_voltage = fireflies[i].voltage + (attractiveness * rand_val) * (fireflies[j].voltage - fireflies[i].voltage);
                       new_voltage = std::max(0.0f, std::min(new_voltage, MAX_INP_VOLT));
                       float new_power = new_voltage * array_current;

                       if (new_power > fireflies[i].power) {
                           fireflies[i].voltage = new_voltage;
                           fireflies[i].power = new_power;
                       }
                   }
               }
           }

           update_global_best(global_best_power, global_best_voltage);
       }

       reference_voltage = global_best_voltage;
       prev_array_power = array_voltage * array_current;
       prev_array_voltage = array_voltage;
   }

   void reset_state(void) {
       reference_voltage = 0.0;
       prev_array_voltage = 0.0;
       prev_array_power = 0.0;
       initialize_fireflies();
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
   PandO* pandO;
   std::mt19937 gen;

   struct Firefly {
       float voltage;
       float power;
       float brightness;
   };

   std::vector<Firefly> fireflies;

private:
   /** Private helper functions. */
   float get_random_value(float min, float max) {
       std::uniform_real_distribution<float> dist(min, max);
       return dist(gen);
   }

   void initialize_fireflies() {
       fireflies.clear();
       for (int i = 0; i < NUM_FIREFLIES; i++) {
           float voltage = get_random_value(0.0, MAX_INP_VOLT);
           float power = voltage * array_current;
           fireflies.push_back({voltage, power, power});
       }
   }

   void update_firefly_brightness() {
       float max_power = -1.0f;
       for (auto& firefly : fireflies) {
           max_power = std::max(max_power, firefly.power);
       }

       for (auto& firefly : fireflies) {
           firefly.brightness = firefly.power / max_power;
       }
   }

   void update_global_best(float& global_best_power, float& global_best_voltage) {
       for (const auto& firefly : fireflies) {
           if (firefly.power > global_best_power) {
               global_best_power = firefly.power;
               global_best_voltage = firefly.voltage;
           }
       }
   }
};