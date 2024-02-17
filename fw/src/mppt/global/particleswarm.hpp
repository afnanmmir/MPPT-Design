/**
 * @file particleswarm.hpp
 * @author Afnan Mir (afnanmir@utexas.edu) & Nash Wu
 * @brief Implementation of Particle Swarm Optimization GlobalMPPTAlgortihm.
 * @version 0.2.0
 * @date 2024-01-27
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <random>

#pragma once
#include "../mppt.hpp"
#include "../local/pando.hpp"

#define    NUM_AGENTS 4
#define    C1 0.02
#define    C2 0.5
#define    W 0.35

class Particle final {

    public:

        Particle();

        Particle(float xPos, float vel) {
            personalBest = 0.0;
            personalBestVolt = 0.0;
            this->xPos = xPos;
            this->vel = vel;

        }

        // created particle class because to call smth.get the smth must be a class type (particle), otherwise ur just saying int.get or wtvr
        // accesses the class to get the val, the class stores some vals like above
        /*
        * 
        *
        *
        */
        
        void changeXPos(float vel) {
            xPos += vel;
        }

        void changeVel(float newVel) {
            vel = newVel;
        }

        float getPBest(void) {
            return personalBest;
        }

        float getVoltBest(void) {
            return personalBestVolt;
        }

        float getXPos(void) {
            return xPos;
        }

        float getVel(void) {
            return vel;
        }

        void changePBest(float newBest, float newBestVolt) {
            personalBest = newBest;
            personalBestVolt = newBestVolt;
        }


    private:

        /** Required inputs. */
        float xPos;
        float vel;

        /** Saved internal data. */
        float personalBest;
        float personalBestVolt;
        float newVel;

        /* Algorithm properties*/


};



class ParticleSwarm : public MPPT {
    public:
    
        ParticleSwarm(void) : MPPT() {
            
            gBest = 0.0;
            gBestVolt = 0.0;

            goForward = true;
            _setup = true;
            startLocal = true;
            kick = true;
            totalCycle = 0;
            
            _setup = true;
            interval = max_voltage / 5;


            for (int i=0; i<NUM_AGENTS; i++) {

                agents[i] = Particle(((double) rand() / (RAND_MAX) * interval) + (interval * i), 0.0);

            }

            cycle = -1;




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

        float setUp(void) {
            agentPower = array_voltage* array_current;

            if(cycle == -1) {
                cycle = 0;
                totalCycle += 1;
                reference_voltage = agents[0].getXPos();
            }

            else if(cycle == 0 && _setup) {
                if(agentPower > agents[0].getPBest()) {
                    agents[0].changePBest(agentPower, array_voltage);
                }

                if (agentPower > gBest){
                gBest = agentPower;
                gBestVolt = array_voltage;
                cycle += 1;
                totalCycle += 1;
                reference_voltage = agents[1].getXPos();

                }

            }
            
            else if(cycle == 1 && _setup){
                if (agentPower > agents[1].getPBest()){
                    agents[1].changePBest(agentPower, array_voltage);
                }
                if (agentPower > gBest){
                    gBest = agentPower;
                    gBestVolt = array_voltage;
                }
                totalCycle += 1;
                cycle += 1;
                reference_voltage = agents[2].getXPos();
                
            }

            else if (cycle == 2 && _setup) {
            if (agentPower > agents[2].getPBest()){
                agents[2].changePBest(agentPower, array_voltage);
            }
            if (agentPower > gBest) {
                gBest = agentPower;
                gBestVolt = array_voltage;
            }
            cycle += 1;
            totalCycle += 1;
            reference_voltage = agents[3].getXPos();
            }

            else if (cycle == 3 && _setup) {
            

                if (agentPower > agents[3].getPBest()) {
                    agents[3].changePBest(agentPower, array_voltage);
                }

                if (agentPower > gBest) {
                    gBest = agentPower;
                    gBestVolt = array_voltage;
                }

                totalCycle += 1;
                cycle = 2;
                _setup = false;
                goForward = false;
                reference_voltage = agents[2].getXPos();
            }


            return reference_voltage;


        }

        void reset_state(void) {
            
            reference_voltage = 0;
            prev_array_voltage = 0;
            prev_array_power = 0;
            prev_array_current = 0;
            prev_area = 0;
            cycle = 0;
            totalCycle = 0;

            goForward = true;
            _setup = true;
            startLocal = true;
            kick = true;

            interval = max_voltage / 5;

            for (int i=0; i<NUM_AGENTS; i++) {

                agents[i] = Particle(((double) rand() / (RAND_MAX) * interval) + (interval * i), 0.0);

            }
            
            cycle = -1;


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
            short cycle;
            int totalCycle;
            Particle agents[NUM_AGENTS]; // creates NUM AGENTS particles, where pb = 0 pbv = 0 xpos = xpos passedin vel = vel passedin

            float gBest;
            float gBestVolt;
            float interval;
            float agentPower;
            int newVel;


            /* Algorithm properties*/   
            bool findingTrapezoids;
            bool startLocal;
            bool kick;
            bool _setup;
            bool goForward;
            PandO* pandO;

    private:

        int getVelocityVector (Particle agent) {

            double r1 = ((double) rand() / (RAND_MAX));
            double r2 = ((double) rand() / (RAND_MAX));

            int newVel = (W * agent.getVel() 
            + C1 * r1 * (agent.getVoltBest() - agent.getXPos())
            + C2 * r2 * (gBestVolt - agent.getXPos())); 
        
            return newVel;

        }

        float agentUpdate () {
            reference_voltage = array_voltage;
            if(_setup) {
                reference_voltage = setUp();
            }

            else {
                reference_voltage = array_voltage;
                agentPower = array_voltage * array_current;
                if (cycle == 0) {
                    if (agentPower > agents[0].getPBest()) {
                        agents[0].changePBest(agentPower, array_voltage);
                    }
                    if (agentPower > gBest) {
                        gBest = agentPower;
                        gBestVolt = array_voltage;
                    }
                    newVel = getVelocityVector(agents[0]);
                    agents[0].changeXPos(newVel);
                    agents[0].changeVel(newVel);
                    goForward = true;
                    cycle = 1;
                    totalCycle += 1;
                    reference_voltage = agents[1].getXPos();
                }
                else if (cycle % 4 == 1) {
                    if (agentPower > agents[1].getPBest()) {
                        agents[1].changePBest(agentPower, array_voltage);
                    }
                    if (agentPower > gBest) {
                        gBest = agentPower;
                        gBestVolt = array_voltage;
                    }
                    newVel = getVelocityVector(agents[1]);
                    agents[1].changeXPos(newVel); 
                    agents[1].changeVel(newVel);
                    if (goForward) {
                        reference_voltage = agents[2].getXPos();
                        cycle = 2;
                    }
                    else {
                        reference_voltage = agents[0].getXPos();
                        cycle = 0;
                    }
                    totalCycle += 1;
                }
                else if (cycle % 4 == 2){
                    if (agentPower > agents[2].getPBest()) {
                        agents[2].changePBest(agentPower, array_voltage);
                    }
                    if (agentPower > gBest) {
                        gBest = agentPower;
                        gBestVolt = array_voltage;
                    }
                    newVel = getVelocityVector(agents[2]);
                    agents[2].changeXPos(newVel);
                    agents[2].changeVel(newVel);
                    if (goForward) {
                        reference_voltage = agents[3].getXPos();
                        cycle = 3;
                    }
                    else {
                        reference_voltage = agents[1].getXPos();
                        cycle = 1;
                    }
                    totalCycle += 1;
                }
                else if (cycle % 4 == 3) {
                    if (agentPower > agents[3].getPBest()) {
                        agents[3].changePBest(agentPower, array_voltage);
                    }
                    if (agentPower > gBest) {
                        gBest = agentPower;
                        gBestVolt = array_voltage;
                    }
                    newVel = getVelocityVector(agents[3]);
                    agents[3].changeXPos(newVel);
                    agents[3].changeVel(newVel);
                    reference_voltage = agents[2].getXPos();
                    cycle = 2;
                    totalCycle += 1;
                    goForward = false;
                }

            }
        return reference_voltage;



        }



};

