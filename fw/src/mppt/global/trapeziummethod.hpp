/**
 * @file trapeziummethod.hpp
 * @author Afnan Mir (afnanmir@utexas.edu) & Nash Wu
 * @brief Implementation of Trapezoidal Sum Optimization GlobalMPPTAlgortihm.
 * @version 0.2.0
 * @date 2024-01-20
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once
#include "../mppt.hpp"
#include "../local/pando.hpp"


#define DV 0.5

class TrapeziumMethod final : public MPPT {
    public:
        TrapeziumMethod(void) : MPPT() {
            reference_voltage = 0.0;
            prev_array_voltage = 0.0;
            prev_array_power = 0.0;
            prev_array_current = 0.0;

            prev_area = 0.0;

            findingTrapezoids = true;
            startLocal = true;
            kick = true;

        }

        float getReferenceVoltage(void) {
            float reference_voltage = array_voltage;
            float array_power = array_voltage * array_current;

            float currentArea = (DV*0.5) * (array_power + prev_array_power);
            float DA = currentArea-prev_area;
            float DP = array_power-prev_array_power;

            float aref = currentArea;
            if(findingTrapezoids) {
                if (DA>0) {
                    aref = currentArea;
                    pref = prev_array_power;
                    reference_voltage = prev_array_voltage;

                    if(currentArea >= aref && array_power >= pref) {
                        pref = array_power;
                        reference_voltage = array_voltage;
                    }
                    
                }

                else {
                        if(currentArea>=aref && array_power >= pref){

                            pref = array_power;
                            reference_voltage = array_voltage;

                        }
                }

                reference_voltage = reference_voltage+DV;
                prev_array_voltage = array_voltage;
                prev_array_power = array_power;
                prev_area = currentArea;

                if(reference_voltage > max_voltage) {
                    findingTrapezoids = false;
                    return reference_voltage;
                }
                
                return reference_voltage;

            }
                

            else {
                if (startLocal) {
                    reference_voltage = reference_voltage; // help
                    startLocal = false;
                    // TODO: self._model.setup(self.reference_voltage, 0, GlobalMPPTAlgorithm.MAX_VOLTAGE)

                }

                else if (kick){
                    reference_voltage = array_voltage + 0.02;
                    kick = false;
                    prev_array_voltage = array_voltage;
                    prev_array_power = array_current*array_voltage;
                    //TODO: self._model._strideModel.vOld = arrVoltage
                    //      self._model._strideModel.pOld = self.pOld
                    prev_array_current = array_current;
                }

                else {
                    // TODO: reference_voltage = _model.getReferenceVoltage(array_voltage,array_current,irradiance,temperature);
                    //       needsChange = self.checkEnvironmentalChanges(irradiance)
                    /*
                    if(needsChange) {
                        reference_voltage = 0;
                        pref = 0;
                        aref = 0;
                        reference_voltage = 0;;
                        aold = 0;
                        findingtrapezoids = True;
                        startLocal = True  ;
                        kick = True;
                        runningHistory.clear();
                        return reference_voltage;
                    }



                    */
                }
            }


            return reference_voltage;
            
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



        void reset_state(void) {
            reference_voltage = 0.0;
            prev_array_voltage = 0.0;
            prev_array_power = 0.0;
            prev_area = 0.0;
            findingTrapezoids = true;
            startLocal = true;
            kick = true;

            pandO->reset_state();
        }




    protected: 
        /** Required inputs. */
        float array_voltage;
        float array_current;
        float battery_voltage;
        float battery_current;
        float pref;
        float max_voltage;


        /** Saved internal data. */
        float prev_array_voltage;
        float prev_array_power;
        float prev_array_current;
        float prev_area;

        /* Algorithm properties*/
        bool findingTrapezoids;
        bool startLocal;
        bool kick;
        PandO* pandO;


    private:
        /** Private helper functions*/
        
        

};