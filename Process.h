#ifndef Process_h
#define Process_h
#include <vector>

class Process {
    public:
        // Constructors
        Process();
        Process(int id, int upperBound, float lambda);
        // Process& operator=(const Process& p);
        // ~Process();

        // Returns the next random number in the sequence
        float nextExp(int upperBound);

        // Getters
        int getId() const {return id;}
        int getNumBurst() const {return numBurst;}
        int getWaitTime(int burstNum) const {return waitTime[burstNum];}
        int getTurnaroundTime(int burstNum) const {return turnaroundTime[burstNum];}
        int getSwitches() const {return switches;}
        int getPreemp() const {return preemption;}
        int getArrival() const {return arrivalTime;}
        int getCpuTime() const {return cpuTime;}
        float getTau() const {return tau;}
        bool getDone() const {return done;}
        int getCurrTime() const {return currTime;}
        int getRemainingBurst() const {return numBurst - currBurst;}
        int getRemainingTime() const {return cpuBurstTime[currBurst] - currTime;}
        int getCpuBurst() const {return cpuBurstTime[currBurst];}
        int getCpuBurstCopy() const {return cpuBurstTimeCopy[currBurst];}
        int getCpuBurstAt(int burstNum) {return cpuBurstTime[burstNum];}
        //int getCpuBurstCopyAt(int burstNum) {return cpuBurstTime[burstNum];}
        int getPrevIOBurst() const {return ioBurstTime[currBurst-1];}
        int getCurrIOBurst() const {return ioBurstTime[currBurst];}
        int getCurrBurst() const {return currBurst;}
        int getStartTime() const {return startTime[currBurst];}
        int getFinishedCPUBurstTime() const {return currCPUBurstTime;}
        int getRemTau() const {return remTau;}

        // Setters
        //int setWaitTime(int waitTime) {this->waitTime = waitTime;}
        //int setTurnaroundTime(int turnaroundTime) {this->turnaroundTime = turnaroundTime;}
        void addSwitches() {this->switches++;}
        void addPreemp() {this->preemption++;}
        bool addCurrBurst();
        void addCurrTime() {this->currTime++;}
        void addCpuTime() {this->cpuTime++;}
        void addWaitTime() {this->waitTime[currBurst]++;}
        void addTurnaroundTime() {this->turnaroundTime[currBurst]++;}
        void setCurrTime(int setTime) {this->currTime = setTime;}
        void decrementCPUBurstTime() {(this->cpuBurstTime[currBurst])--;}
        void decrementIOBurstTime() {(this->ioBurstTime[currBurst-1])--;}
        //void setCpuBurstTime(int setTime, int numBurst) {this->cpuBurstTime[numBurst] = setTime;}
        void setIOBurstTime(int setTime) {this->ioBurstTime[currBurst] = setTime;}
        void isDone() {this->done = true;}
        void addSwitchTimeToIO(int seconds) {this->ioBurstTime[currBurst-1] += seconds;}
        void setStartTime(int setTime) {this->startTime[currBurst] = setTime;}
        void setWaitTime(int setTime) {this->waitTime[currBurst] = setTime;}
        void setTurnaroundTime(int setTime) {this->turnaroundTime[currBurst] = setTime;}
        void setVectorCpuBurst() {this->cpuBurstTime = this->cpuBurstTimeCopy;} 
        void setRemTau(int tau) {this->remTau = tau;}
        //void setTau(int tau) {this->tau = tau;}

        // Modifiers
        // leaveCPU(int time);
        void reset();
        float tauNext(float alpha, int currBurstTime);
        void print();

    private:
        // void copy(const Process& p);

        int id;
        int numBurst;
        int currBurst;
        int currTime;
        int cpuTime;
        int currCPUBurstTime;
        std::vector <int> cpuBurstTime;
        std::vector <int> cpuBurstTimeCopy;
        std::vector <int> ioBurstTime;
        std::vector <int> startTime;
        std::vector <int> waitTime; // reset this
        std::vector <int> turnaroundTime; // reset this
        int switches; // reset this
        int preemption; // reset this
        int arrivalTime;
        float tau;
        bool done;
        float remTau;
};
bool operator<(const Process &p1, const Process &p2);
#endif