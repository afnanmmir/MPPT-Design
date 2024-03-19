/**
 * @file global_algorithms.hpp
 * @author Praneel Murali (praneelm@utexas.edu)
 * @brief Default Global MPPT algorithm driver.
 * @version 0.2.0
 * @date 2024-03-18
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include "../mppt.hpp"

// Global Algorithm Selection
#define GLOBAL_ALGO 0
// 0: Default (Local Algorithm Only)
// 1: Voltage Sweep
// 2: Trapezoidal Sum Optimization
// 3: Simulated Annealing
// 4: Firefly Algorithm

// Local Algorithm Selection
#define LOCAL_ALGO 0
// 0: Perturb and Observe (PandO)
// 1: Incremental Conductance (IC)
// 2: Fuzzy Logic
// 3: Feedback Control (FC)

#if LOCAL_ALGO == 0
#include "../local/pando.hpp"
#elif LOCAL_ALGO == 1
#include "../local/ic.hpp"
#elif LOCAL_ALGO == 2
#include "../local/fuzzy.hpp"
#elif LOCAL_ALGO == 3
#include "../local/fc.hpp"
#endif

class DefaultGlobal final : public MPPT {
public:
    DefaultGlobal(void) : MPPT() {
        #if LOCAL_ALGO == 0
        localMPPT = new PandO();
        #elif LOCAL_ALGO == 1
        localMPPT = new IC();
        #elif LOCAL_ALGO == 2
        localMPPT = new Fuzzy();
        #elif LOCAL_ALGO == 3
        localMPPT = new FC();
        #endif
    }

    void input_context(void *args) {
        localMPPT->input_context(args);
    }

    void step_algorithm(void) {
        localMPPT->step_algorithm();
    }

    float get_reference(void) {
        return localMPPT->get_reference();
    }

    void reset_state(void) {
        localMPPT->reset_state();
    }

protected:
    MPPT* localMPPT;
};

#if GLOBAL_ALGO == 0
using GlobalMPPT = DefaultGlobal;
#elif GLOBAL_ALGO == 1
#include "voltage_sweep.hpp"
using GlobalMPPT = VoltageSweep;
#elif GLOBAL_ALGO == 2
#include "trapeziummethod.hpp"
using GlobalMPPT = TrapeziumMethod;
#elif GLOBAL_ALGO == 3
#include "simulated_annealing.hpp"
using GlobalMPPT = SimulatedAnnealing;
#elif GLOBAL_ALGO == 4
#include "firefly_algorithm.hpp"
using GlobalMPPT = FireflyAlgorithm;
#endif