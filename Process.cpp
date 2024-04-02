#include <iostream>
#include <math.h>
#include <vector>
#include "Process.h"

// // Associate each index with a letter of the alphabet
const std::vector<char> alphabet = {' ','A','B','C','D','E','F','G','H','I','J','K',
'L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z'};

Process::Process() {
    id = -1;
    numBurst = 0;
    currBurst = 0;
    currTime = 0;
    cpuTime = 0;
    switches = 0;
    preemption = 0;
    arrivalTime = 0;
    tau = 0;
    currCPUBurstTime = 0;
    done = false;
    remTau = 0;
}


Process::Process(int id, int upperBound, float lambda) {

    // initialize everything
    this->id = id;
    this->currTime = 0;
    this->currBurst = 0;
    this->cpuTime = 0;
    this->switches = 0;
    this->preemption = 0;
    this->tau = 1.0/lambda;
    this->done = false;
    this->remTau = 1.0/lambda;

    // generate arrival time
    float r = nextExp(upperBound);
    this->arrivalTime = (int) floor(-log( r ) / lambda);

    // generate the number of CPU Bursts
    this->numBurst = ceil(nextExp(upperBound) * 100.0);

    // loop and generate bursts for each CPU and IO Burst
    for (int i = 0; i < this->numBurst; i++) {
        int cpuBurst = (int) ceil(-log(nextExp(upperBound))/lambda);
        this->cpuBurstTime.push_back(cpuBurst);
        this->cpuBurstTimeCopy.push_back(cpuBurst);

        if (i != (this->numBurst-1)){
            int ioBurst = (int) ceil(-log(nextExp(upperBound))/lambda) * 10.0;
            this->ioBurstTime.push_back(ioBurst);
        }
        this->waitTime.push_back(0);
        this->turnaroundTime.push_back(0);
        this->startTime.push_back(0);
    } 

    this->currCPUBurstTime = cpuBurstTime[0];
}

// Process::~Process(){
//     //empty cause no dynamic memory used
//     this->cpuBurstTime.clear();
//     this->ioBurstTime.clear();
//     this->waitTime.clear();
//     this->turnaroundTime.clear();
// }

// void Process::copy(const Process& p) {
//     this->id = p.id;
//     this->numBurst = p.numBurst;
//     this->currBurst = p.currBurst;
//     this->currTime = p.currTime;
//     this->cpuTime = p.cpuTime;
//     this->switches = p.switches;
//     this->preemption = p.preemption;
//     this->tau = p.tau;
//     this->done = p.done;
//     for (unsigned int i = 0; i < p.cpuBurstTime.size(); i++) {
//         this->cpuBurstTime.push_back(p.cpuBurstTime[i]);
//         if ((int)i != p.numBurst-1) {
//             this->ioBurstTime.push_back(p.ioBurstTime[i]);
//         }
//     }
//     for (unsigned int i = 0; i < p.waitTime.size(); i++) {
//         this->waitTime.push_back(p.waitTime[i]);
//     }
//     for(unsigned int i = 0; i < p.turnaroundTime.size(); i++) {
//         this->turnaroundTime.push_back(p.turnaroundTime[i]);
//     }
// }

// Process& Process::operator=(const Process& p) {
//     if (this != &p) {
//         // delete current data
//         this->cpuBurstTime.clear();
//         this->ioBurstTime.clear();
//         this->waitTime.clear();
//         this->turnaroundTime.clear();

//         this->copy(p);
//     }
//     return *this;
// }

float Process::nextExp(int upperBound){
    double next = drand48();
    while (next >= upperBound) {
        next = drand48();
    }
    return (float)next;
}

void Process::reset(){
    this->switches = 0;
    this->preemption = 0;
    for (unsigned int i = 0 ; i < waitTime.size(); i++) {
        this->waitTime[i] = 0;
        this->turnaroundTime[i] = 0;
    }
}

/**
 * @brief increments the current burst member variable
 * 
 * @return true if currBurst was successfully incremented
 * @return false if currBurst is equal to the number of bursts
 */
bool Process::addCurrBurst() {
    if (currBurst == numBurst) {
        return false;
    }
    this->currCPUBurstTime = this->cpuBurstTimeCopy[currBurst];
    this->currBurst++;
    return true;
}

void Process::print() {
    std::cout << "Process "<< alphabet[id] <<": " 
              << "arrival time " << arrivalTime << "ms; "
              << "tau " << tau << "ms; ";
    if (numBurst == 1) {
        std::cout << numBurst << " CPU burst:" << std::endl;
    } else {
        std::cout << numBurst << " CPU bursts:" << std::endl;
    }
    for (int i = 0; i < numBurst; i++) {
        std::cout << "--> CPU burst " << cpuBurstTime[i] << "ms";
        if (i != numBurst - 1) {
            std::cout << " --> I/O burst " << ioBurstTime[i] << "ms";
        }
        std::cout << std::endl;
    }
}

float Process::tauNext(float alpha, int currBurstTime) {
    //std::cout << "Inside tauNext" << std::endl;
    //std::cout << "alpha is " << alpha << " currBurstTime is " << currBurstTime << " currBurst num for "<< std::endl;
    this->tau =  ceil((alpha * currBurstTime) + (1 - alpha) * this->tau);
    return this->tau;
}

bool operator<(const Process &p1, const Process &p2) {
    if (p1.getRemTau() == p2.getRemTau()) {
        return p1.getId() > p2.getId();
    }
    return p1.getRemTau() > p2.getRemTau();
}