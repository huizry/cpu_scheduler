#include <iostream> // cin, cout, cerr
#include <fstream> // fstream
#include <iomanip> // Setprecision
#include <vector> // vector
#include <queue> // queue
#include <algorithm> // sort
#include <climits> // INT_MAX
#include <math.h>
#include <cmath>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include "Process.h"

// Print the vector ready queue
void printQueue(std::vector<Process> processes);
// Print the priority queue ready queue
void printQueue(std::priority_queue<Process> processes);
// Compare by ascending arrival time and id
bool compare(Process i, Process j);
// Compare by ascending id
bool compareId(Process i, Process j);
// Print statistics for each simulated algorithm
void simout(std::fstream& myfile, std::vector<Process> processes, int simTime, int algo);
// Algorithms
void fcfs(std::vector<Process> processes, int timeContextSwitch, std::fstream& myfile, std::fstream& simfile);
void sjf(std::vector<Process> processes, float alpha, int halfContextSwitchTime, std::fstream& myfile, std::fstream& simfile);
void srt(std::vector<Process> processes, float alpha, int halfContextSwitchTime, std::fstream& myfile, std::fstream& simfile);
void rr(std::vector<Process> processes, int timeSlice, int timeContextSwitch, bool rr_check, std::fstream& myfile, std::fstream& simfile);

// Index corresponding to each letter of the alphabet
const std::vector<char> alphabet = 
{' ','A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S',
'T','U','V','W','X','Y','Z'};

// Print statistics for each simulated algorithm
void simout(std::fstream& myfile, std::vector<Process> processes, int simTime, int algo) {
    //myfile << "time " << simTime << std::endl;
    int CpuBurstTime = 0; // sum of cpu burst time for each process
    int WaitTime = 0; // sum of wait time for each process
    int totalCpuBurst = 0; // total # of cpu burst for all of the processes
    int turnaroundTime = 0; // sum of turnaround time for each process
    int totalContextSwitch = 0; // total # of context switch for all of the process
    int totalPreemption = 0; // total # of Preemption for all of the process
    // Iterate through processes
    for(unsigned int i = 0; i < processes.size(); ++i) {
        totalContextSwitch += processes[i].getSwitches();
        totalPreemption += processes[i].getPreemp();
        totalCpuBurst += processes[i].getNumBurst();
        // Iterate through the bursts of each processes
        for(int j = 0; j < processes[i].getNumBurst(); j++) {
            CpuBurstTime += processes[i].getCpuBurstAt(j);
            WaitTime += processes[i].getWaitTime(j);
            turnaroundTime += processes[i].getTurnaroundTime(j);
        }    
    }
    WaitTime -= totalPreemption * 4;
    // Write to file
    myfile << "Algorithm ";
    if(algo == 0) {
        myfile << "FCFS\n";
    }
    else if(algo == 1) {
        myfile << "SJF\n";
    }
    else if(algo == 2) {
        myfile << "SRT\n";
    }
    else {
        myfile << "RR\n";
    }
    myfile.setf(std::ios::fixed,std::ios::floatfield);
    myfile.precision(3);
    
    myfile << "-- average CPU burst time: " << ceil((CpuBurstTime / (float) totalCpuBurst) * 1000.0)/1000.0 << " ms" << std::endl;
    myfile << "-- average wait time: " << ceil(WaitTime / (float) totalCpuBurst* 1000.0)/1000.0 << " ms" << std::endl;
    myfile << "-- average turnaround time: " << ceil(turnaroundTime / (float) totalCpuBurst* 1000.0)/1000.0 <<" ms" << std::endl;
    myfile << "-- total number of context switches: " << totalContextSwitch << std::endl;
    myfile << "-- total number of preemptions: " << totalPreemption << std::endl;
    myfile << "-- CPU utilization: " << ceil(CpuBurstTime / (float) simTime * 100.0 * 1000.0)/1000.0 << "%" << std::endl;
}

// Print the vector ready queue
void printQueue(std::vector<Process> processes, std::fstream& file) {
    if (processes.empty()) {
        file << "[Q: empty";
    } 
    else {
        file << "[Q:";
    }
    // Print the id of each processes
    for (unsigned int i = 0; i < processes.size(); i++) {
        file << " " << alphabet[processes[i].getId()]; 
    }
    file << "]" << std::endl;
}

// Print the priority queue ready queue
void printQueue(std::priority_queue<Process> processes, std::fstream& file) {
    // Create a copy of processes so we don't modify it
    std::priority_queue<Process> temp = processes;
    if (processes.empty()) {
        file << "[Q: empty";
    } else {
        file << "[Q:";
    }
    // Print the id of each processes
    while (!temp.empty()) {
        file << " " << alphabet[temp.top().getId()];
        temp.pop();
    }
    file << "]" << std::endl;
}

// Compare by ascending arrival time and id
bool compare(Process i, Process j) {
    // If arrival time is equivalent, compare by ascending id
    if (i.getArrival() == j.getArrival()) {
        return (i.getId() < j.getId());
    }
    // Else, compare by ascending arrival time
    return (i.getArrival() < j.getArrival());
}

// Compare by ascending id
bool compareId(Process i, Process j) {
    return (i.getId() < j.getId());
}

// First-come-first-served (FCFS)
void fcfs(std::vector<Process> processes, int timeContextSwitch, std::fstream& myfile, std::fstream& simfile) {
    // FCFS is RR with an “infinite” time slice
    int max = INT_MAX;
    rr(processes, max, timeContextSwitch, false, myfile, simfile);
}

// Shortest job first (SJF)
void sjf(std::vector<Process> processes, float alpha, int halfContextSwitchTime, std::fstream& myfile, std::fstream& simfile) {
    // Initial number of processes
    unsigned int n = processes.size();

    // Current time
    int time = 0;

    bool printStop = true;

    // Running state
    Process cpu;
    // Waiting state
    std::vector<Process> waitingState;
    // Ready state
    std::priority_queue<Process> readyQueue;
    // Completed processes
    std::vector<Process> Done;
    
    Process noProcess;
    Process nextProcess;
    int currBurstTime = 0;
    bool blockCpu = true;
    int switchOut = 0;
    int switchIn = 0;

    myfile << "time " << time << "ms: Simulator started for SJF ";
    printQueue(readyQueue, myfile); 

    // While all processes are not terminated
    while (Done.size() != n) {
        if (time == 1000) {
            printStop = false;
        }
        // If cpu is non-empty, decrement cpu burst time
        if(cpu.getId() != -1) {
            cpu.decrementCPUBurstTime();
        }
        // Check for CPU burst completion
        if (cpu.getId() != -1 && cpu.getCpuBurst() == 0) {
            int turnaroundTime = time - cpu.getStartTime() + halfContextSwitchTime;
            cpu.setTurnaroundTime(turnaroundTime);
            cpu.setWaitTime(turnaroundTime - cpu.getCpuBurstCopy() - (2*halfContextSwitchTime));

            //std::cout << time - cpu.getStartTime() << std::endl;

            // Increment the burst completed for a process
            cpu.addCurrBurst();
            // If there are no more burst for a process (i.e. process terminating)
            if (cpu.getRemainingBurst() == 0) {
                // Print
                myfile << "time " << time << "ms: Process " << alphabet[cpu.getId()]
                << " terminated ";
                printQueue(readyQueue, myfile);

                // Add the completed process to done
                Done.push_back(cpu);
                // Remove completed process from cpu
                cpu = noProcess;
                // Block other processes from using cpu
                blockCpu = true;
                // Add half context switch time for removing process from cpu
                switchOut += halfContextSwitchTime+1;
            }
            // Else if there are more burst for a process (i.e. process not terminating)
            else {
                // Print
                if(printStop) {
                    myfile << "time " << time << "ms: Process " << alphabet[cpu.getId()]
                    << " (tau " << cpu.getTau()  << "ms) completed a CPU burst; ";
                }
                // If process in cpu has more than 1 remaining burst
                if (cpu.getRemainingBurst() > 1) {
                    if(printStop) {
                        myfile << cpu.getRemainingBurst() << " bursts to go ";
                    }
                }
                // If process in cpu has exactly 1 remaining burst
                else {
                    if(printStop) {
                        myfile << "1 burst to go ";
                    }
                }
                if(printStop) {
                    printQueue(readyQueue, myfile);
                }
                // Update tau for the process in the CPU
                int oldTau = ceil(cpu.getTau());
                cpu.tauNext(alpha, currBurstTime);
                cpu.setRemTau(cpu.getTau());

                // Print
                if(printStop) {
                    myfile << "time " << time << "ms: Recalculated tau for process " 
                    << alphabet[cpu.getId()] << ": old tau " << oldTau << "ms; new tau " 
                    << cpu.getTau() << "ms "; 
                    printQueue(readyQueue, myfile);
                }

                // Switch the process into the waitingState and block cpu for haldContextSwitchTime
                switchOut += halfContextSwitchTime;
                
                // Print
                if(printStop) {
                    myfile << "time " << time << "ms: Process " << alphabet[cpu.getId()]
                    << " switching out of CPU; will block on I/O until time " 
                    << (cpu.getPrevIOBurst() + time + switchOut) << "ms ";
                    printQueue(readyQueue, myfile);
                }
                switchOut += 1;
                cpu.addSwitchTimeToIO(switchOut);
                waitingState.push_back(cpu);
                cpu = noProcess;
                blockCpu = true;
            }
            // if the ready queue is not empty, we can switchin the next process
            // so add that time here
            if (readyQueue.size() > 0) {
                switchIn += halfContextSwitchTime;
            }
        }
        // Corner Case: The ready queue was empty when the finished using the cpu
        // If no process in cpu but there is one ready, blockcpu for switch in time
        if (cpu.getId() == -1 && readyQueue.size() > 0 && nextProcess.getId() == -1 && switchIn == 0 ) {
            switchIn += halfContextSwitchTime;
            blockCpu = true;
        }
        // Blocks Cpu to allow for context switch
        if (blockCpu) {
            
            if (switchOut > 0) {
                switchOut--;
                if (switchIn == 0 && switchOut == 0) {
                    blockCpu = false;
                }
            } else if (switchIn > 0) {
                // Get the next process ready when we start waiting for switchIn time
                if (switchIn == halfContextSwitchTime && switchOut == 0 && (readyQueue.size() > 0 || nextProcess.getId() != -1)) {
                    nextProcess = readyQueue.top();
                    currBurstTime = nextProcess.getCpuBurst();
                    readyQueue.pop();
                }
                switchIn--;
                if (switchIn == 0) {
                    blockCpu = false;
                }
            }
            if (switchIn == 0 && switchOut == 0) {
                blockCpu = false;
            }
        }
        // Add next process to cpu
        if (!blockCpu && cpu.getId() == -1 && (readyQueue.size() > 0 || nextProcess.getId() != -1)) {
            // if the next proccess was not set, give the cpu the next process in readyQueuec
            if (nextProcess.getId() == -1) {
                cpu = readyQueue.top();
                currBurstTime = cpu.getCpuBurst();
                readyQueue.pop();
            } else {
                cpu = nextProcess;
            }
            // To count the number of context switches, you should count the 
            // number of times a process starts using the CPU
            cpu.addSwitches();

            nextProcess = noProcess;
            if (halfContextSwitchTime == 0) {
                    time--;
            }
            //time 35ms: Process A started using the CPU for 207ms burst [Q: empty]
            if(printStop) {
                myfile << "time " << time << "ms: Process " << alphabet[cpu.getId()]
                << " (tau " << cpu.getTau()  << "ms) started using the CPU for " 
                << currBurstTime << "ms burst "; 
                printQueue(readyQueue, myfile); 
            }
        }
        // Iterate through IO processes
        if (waitingState.size() > 1) {
            sort(waitingState.begin(), waitingState.end(), compareId);
        }
        for (std::vector<Process>::iterator waitStateItr= waitingState.begin(); 
        waitStateItr != waitingState.end();) {
            // Decrement IO burst time for each process
            waitStateItr->decrementIOBurstTime();

            // If IO burst complete, add to ready queue, remove from waiting state 
            if (waitStateItr->getPrevIOBurst() == 0) {
                waitStateItr->setStartTime(time);

                // Add to ready queue
                readyQueue.push(*waitStateItr);

                // Print
                if(printStop) {
                    myfile << "time " << time << "ms: Process " << alphabet[waitStateItr->getId()]
                    << " (tau " << waitStateItr->getTau()  << "ms) completed I/O; added to ready queue "; 
                    printQueue(readyQueue, myfile); 
                }

                // Remove from waiting state
                waitStateItr = waitingState.erase(waitStateItr);
            }
            // Continue to the next IO process
            else {
                waitStateItr++;
            }
        }
        // When new process arrive, add to ready queue
        while (processes.size() != 0 && processes[0].getArrival() == time) {
            // Add to ready queue
            processes[0].setStartTime(time);
            readyQueue.push(processes[0]);

            // Print
            if(printStop) {
                myfile << "time " << time << "ms: Process " << alphabet[(processes[0].getId())]
                << " (tau " << processes[0].getTau()  << "ms) arrived; added to ready queue ";
            }

            // Remove from vector of processes
            processes.erase(processes.begin());
            
            if(printStop) {
                // Print the process in the ready queue
                printQueue(readyQueue, myfile);
            }
        }
        time++;
    }
    // Print
    myfile << "time " << time+halfContextSwitchTime-1 << "ms: Simulator ended for SJF ";
    printQueue(readyQueue, myfile); 

    for (unsigned int i = 0; i < Done.size(); i++) {
        // switch CPU burst copy and CPU bust
        Done[i].setVectorCpuBurst();
    }

    time++;
    myfile << std::endl;
    // Generate an output file called simout.txt that contains statistics for SJF
    simout(simfile, Done, time, 1);
}

void srt(std::vector<Process> processes, float alpha, int halfContextSwitchTime, std::fstream& myfile, std::fstream& simfile) {
    // Initial number of processes
    unsigned int n = processes.size();

    // Current time
    int time = 0;

    // Running state
    Process cpu;
    // Waiting state
    std::vector<Process> waitingState;
    // Ready state
    std::priority_queue<Process> readyQueue;
    // Completed processes
    std::vector<Process> Done;
    
    Process noProcess;
    Process nextProcess;
    int nextBurstTime = 0;
    nextBurstTime += 0;
    bool preempt = false;
    bool blockCpu = true;
    int switchOut = 0;
    int switchIn = 0;

    int remaining = 0;
    int usingCpuStart = 0;
    bool printing = true;

    myfile << "time " << time << "ms: Simulator started for SRT ";
    printQueue(readyQueue, myfile);  

    // While all processes are not terminated
    while (Done.size() != n) {
        if (time == 1000) {
            printing = false;
        }
        // If cpu is non-empty, decrement cpu burst time
        if(cpu.getId() != -1) {
            cpu.decrementCPUBurstTime();
            cpu.addCurrTime();
        }
        // Check for CPU burst completion
        if (cpu.getId() != -1 && cpu.getCpuBurst() == 0) {
            int turnaroundTime = time - cpu.getStartTime() + halfContextSwitchTime;
            cpu.setTurnaroundTime(turnaroundTime);
            cpu.setWaitTime(turnaroundTime - cpu.getCpuBurstCopy() - (2*halfContextSwitchTime));

            //myfile << time - cpu.getStartTime() << std::endl;

            // Increment the burst completed for a process
            cpu.addCurrBurst();
            // If there are no more burst for a process (i.e. process terminating)
            if (cpu.getRemainingBurst() == 0) {
                // Print
                myfile << "time " << time << "ms: Process " << alphabet[cpu.getId()]
                << " terminated ";
                printQueue(readyQueue, myfile);
               

                // Add the completed process to done
                Done.push_back(cpu);
                // Remove completed process from cpu
                cpu = noProcess;
                // Block other processes from using cpu
                blockCpu = true;
                // Add half context switch time for removing process from cpu
                switchOut += halfContextSwitchTime+1;
                
            }
            // Else if there are more burst for a process (i.e. process not terminating)
            else {
                // Print
                if (printing) {
                    myfile << "time " << time << "ms: Process " << alphabet[cpu.getId()]
                    << " (tau " << cpu.getTau()  << "ms) completed a CPU burst; ";
                    // If process in cpu has more than 1 remaining burst
                    if (cpu.getRemainingBurst() > 1) {
                        myfile << cpu.getRemainingBurst() << " bursts to go ";
                    }
                    // If process in cpu has exactly 1 remaining burst
                    else {
                        myfile << "1 burst to go ";
                    }
                    printQueue(readyQueue, myfile);
                }

                // Update tau for the process in the CPU
                int oldTau = ceil(cpu.getTau());
                cpu.tauNext(alpha, cpu.getFinishedCPUBurstTime());
                cpu.setRemTau(cpu.getTau());

                // Print
                if (printing) {
                    myfile << "time " << time << "ms: Recalculated tau for process " 
                    << alphabet[cpu.getId()] << ": old tau " << oldTau << "ms; new tau " 
                    << cpu.getTau() << "ms "; 
                    printQueue(readyQueue, myfile);
                }

                // Switch the process into the waitingState and block cpu for haldContextSwitchTime
                switchOut += halfContextSwitchTime;
                
                // Print
                if(printing) {
                    myfile << "time " << time << "ms: Process " << alphabet[cpu.getId()]
                    << " switching out of CPU; will block on I/O until time " 
                    << (cpu.getPrevIOBurst() + time + switchOut) << "ms ";
                    printQueue(readyQueue, myfile);
                }
                
                switchOut += 1;
                cpu.addSwitchTimeToIO(switchOut);
                cpu.setCurrTime(0);
                waitingState.push_back(cpu);
                cpu = noProcess;
                blockCpu = true;
            }
            // if the ready queue is not empty, we can switchin the next process
            // so add that time here
            if (readyQueue.size() > 0) {
                switchIn += halfContextSwitchTime;
            }
        }
        // // Corner Case: The ready queue was empty when the finished using the cpu
        // If no process in cpu but there is one ready, blockcpu for switch in time
        if (cpu.getId() == -1 && readyQueue.size() > 0 && nextProcess.getId() == -1 && switchIn == 0 ) {
            switchIn += halfContextSwitchTime;
            blockCpu = true;
        }

        // Blocks Cpu to allow for context switch
        if (blockCpu) {
            
            if (switchOut > 0) {
                switchOut--;
                if (switchIn == 0 && switchOut == 0) {
                    blockCpu = false;
                }
            } else if (switchIn > 0) {
                // Get the next process ready when we start waiting for switchIn time
                if (switchIn == halfContextSwitchTime && switchOut == 0 && (readyQueue.size() > 0 || nextProcess.getId() != -1)) {
                    nextProcess = readyQueue.top();
                    nextBurstTime = nextProcess.getCpuBurst();
                    readyQueue.pop();
                }
                switchIn--;
                if (switchIn == 0) {
                    blockCpu = false;
                }
            }

            if (switchIn == 0 && switchOut == 0) {
                blockCpu = false;
            }
        }

        // Add next process to cpu
        if (!blockCpu && cpu.getId() == -1 && (readyQueue.size() > 0 || nextProcess.getId() != -1)) {
            // if the next proccess was not set, give the cpu the next process in readyQueuec
            if (nextProcess.getId() == -1) {
                cpu = readyQueue.top();
                //currBurstTime = cpu.getCpuBurst();
                readyQueue.pop();
            }else {
                cpu = nextProcess;
                //currBurstTime = nextBurstTime;
            }

            // To count the number of context switches, you should count the 
            // number of times a process starts using the CPU
            cpu.addSwitches();
            usingCpuStart = time;

            nextProcess = noProcess;
            
            if (cpu.getCurrTime() == 0) {
                if (halfContextSwitchTime == 0) {
                    time--;
                }
                //time 35ms: Process A started using the CPU for 207ms burst [Q: empty]
                if (printing) {
                    myfile << "time " << time << "ms: Process " << alphabet[cpu.getId()]
                    << " (tau " << cpu.getTau()  << "ms) started using the CPU for " 
                    << cpu.getCpuBurstCopy() << "ms burst "; 
                }
                remaining = 0;
            } else {
                //time 1166ms: Process F (tau 100ms) started using the CPU for remaining 58ms of 106ms burst [Q: G A]
                if (printing) {
                    myfile << "time " << time << "ms: Process " << alphabet[cpu.getId()]
                    << " (tau " << cpu.getTau()  << "ms) started using the CPU for remaining " 
                    << cpu.getCpuBurstCopy() - cpu.getCurrTime() << "ms of " << cpu.getCpuBurstCopy() << "ms burst "; 
                }
                remaining = cpu.getCpuBurstCopy() - cpu.getCurrTime();
            }
            if (printing) {
                printQueue(readyQueue, myfile);
            }
            
            if (preempt && switchIn == 0) {
                if (!readyQueue.empty() && cpu.getId() != -1) {
                    int subtract = 0;
                    int timeSpent = time - usingCpuStart;
                    if (cpu.getCurrTime() != timeSpent) {
                        subtract = cpu.getCpuBurstCopy() - remaining;
                    }
                    if (readyQueue.top().getTau() < (cpu.getTau() - subtract - timeSpent)) {
                        //Process D (tau 71ms) will preempt H
                        if (printing) {
                            myfile << "time " << time << "ms: Process " << alphabet[readyQueue.top().getId()] << " (tau " <<
                            readyQueue.top().getTau() << "ms) will preempt " << alphabet[cpu.getId()] << " ";
                            printQueue(readyQueue, myfile);
                        }
                        // Add preempted process back to ready queue
                        cpu.setRemTau(cpu.getTau() - subtract);
                        cpu.addPreemp();
                        readyQueue.push(cpu);

                        // Add half context switch for removing element from cpu
                        switchOut += halfContextSwitchTime;

                        // Add 1st prpcess from ready queue to cpu
                        /*cpu = readyQueue.top();
                        readyQueue.pop();*/
                        cpu = noProcess;
                        blockCpu = true;

                        // Add half context switch for adding element to cpu
                        switchIn += halfContextSwitchTime;

                        preempt = false;
                    }
                }
            }  
        }
        // Iterate through IO processes
        if (waitingState.size() > 1) {
            sort(waitingState.begin(), waitingState.end(), compareId);
        }

        for (std::vector<Process>::iterator waitStateItr= waitingState.begin(); 
        waitStateItr != waitingState.end();) {
            // Decrement IO burst time for each process
            waitStateItr->decrementIOBurstTime();

            // If IO burst complete, add to ready queue, remove from waiting state 
            if (waitStateItr->getPrevIOBurst() == 0) {
                waitStateItr->setStartTime(time);

                // Add to ready queue
                readyQueue.push(*waitStateItr);

                // Print
                if (printing) {
                    myfile << "time " << time << "ms: Process " << alphabet[waitStateItr->getId()]
                    << " (tau " << waitStateItr->getTau()  << "ms) completed I/O; ";
                }
                
                preempt = true;
                if (preempt) {
                    if (!readyQueue.empty() && cpu.getId() != -1) {
                        int subtract = 0;
                        int timeSpent = time - usingCpuStart;
                        if (cpu.getCurrTime() != timeSpent) {
                            subtract = cpu.getCpuBurstCopy() - remaining;
                        }
                        if (readyQueue.top().getTau() < (cpu.getTau() - subtract - timeSpent)) {
                            if (printing) {
                                myfile << "preempting " << alphabet[cpu.getId()] << " ";
                                printQueue(readyQueue, myfile);
                            }

                            // Add preempted process back to ready queue
                            cpu.setRemTau(cpu.getTau() - timeSpent - subtract);
                            cpu.addPreemp();
                            readyQueue.push(cpu);

                            // Add half context switch for removing element from cpu
                            switchOut += halfContextSwitchTime;

                            // Add 1st prpcess from ready queue to cpu
                            /*cpu = readyQueue.top();
                            readyQueue.pop();*/
                            cpu = noProcess;
                            blockCpu = true;

                            // Add half context switch for adding element to cpu
                            switchIn += halfContextSwitchTime;

                            preempt = false;
                        } else {
                            preempt = false;
                            if (printing) {
                                myfile << "added to ready queue ";
                                printQueue(readyQueue, myfile);
                            }
                        }
                    }
                    else if (switchIn > 0) {
                        preempt = true;
                        if (printing) {
                            myfile << "added to ready queue ";
                            printQueue(readyQueue, myfile);
                        }
                    }
                    else {
                        preempt = false;
                        if (printing) {
                            myfile << "added to ready queue ";
                            printQueue(readyQueue, myfile);
                        }
                    }
                }
                // Remove from waiting state
                waitStateItr = waitingState.erase(waitStateItr);
            }
            // Continue to the next IO process
            else {
                waitStateItr++;
            }
        }
        // When new process arrive, add to ready queue
        while (processes.size() != 0 && processes[0].getArrival() == time) {
            // Add to ready queue
            processes[0].setStartTime(time);
            
            readyQueue.push(processes[0]);

            // Print
            if (printing) {
                myfile << "time " << time << "ms: Process " << alphabet[(processes[0].getId())]
                << " (tau " << processes[0].getTau()  << "ms) arrived; ";
            }
            
            preempt = true;
            if (preempt) {
                if (!readyQueue.empty() && cpu.getId() != -1) {
                    int subtract = 0;
                    int timeSpent = time - usingCpuStart;
                    if (cpu.getCurrTime() != timeSpent) {
                        subtract = cpu.getCpuBurstCopy() - remaining;
                    }
                    if (readyQueue.top().getTau() < (cpu.getTau() - subtract - timeSpent)) {
                        if (printing) {
                            myfile << "preempting " << alphabet[cpu.getId()] << " ";
                            printQueue(readyQueue, myfile);
                        }
                        
                        // Add preempted process back to ready queue
                        cpu.setRemTau(cpu.getTau() - timeSpent);
                        cpu.addPreemp();
                        readyQueue.push(cpu);

                        // Add half context switch for removing element from cpu
                        switchOut += halfContextSwitchTime;

                        // Add 1st prpcess from ready queue to cpu
                        /*cpu = readyQueue.top();
                        readyQueue.pop();*/
                        cpu = noProcess;
                        blockCpu = true;

                        // Add half context switch for adding element to cpu
                        switchIn += halfContextSwitchTime;

                        preempt = false;
                    } 
                    else {
                        preempt = false;
                        if (printing) {
                            myfile << "added to ready queue ";
                            printQueue(readyQueue, myfile);
                        }
                    }
                }
                else {
                    preempt = false;
                    if (printing) {
                        myfile << "added to ready queue ";
                        printQueue(readyQueue, myfile);
                    }
                }
            }
            // Remove from vector of processes
            processes.erase(processes.begin());
        }
        // ReadyQueue is not empty, cpu is not empty, 1st process in ready queue 
        // has shorter remaining burst time than process in cpu
        time++;
    }
    // Print
    myfile << "time " << time+halfContextSwitchTime-1 << "ms: Simulator ended for SRT ";
    printQueue(readyQueue, myfile); 

    for (unsigned int i = 0; i < Done.size(); i++) {
        // switch CPU burst copy and CPU bust
        Done[i].setVectorCpuBurst();
    }
    time++;
    myfile << std::endl;
    // Generate an output file called simout.txt that contains statistics for SJF
    simout(simfile, Done, time, 2);
}

// Round robin (RR)
void rr(std::vector<Process> processes, int timeSlice, int timeContextSwitch, bool rr_check, std::fstream& myfile, std::fstream& simfile) {
    // what is in the queue for CPU Burst
    std::vector<Process> ready;
    std::vector<Process> wait;
    std::vector<Process> done;
    
    unsigned int processIndex = 0;
    unsigned int finished = 0;
    int time = 0;
    bool usage = false;
    int currSlice = timeSlice;
    int conSwitch = 0;
    Process currProcess;
    Process prevProcess;
    int addTime = 0;
    //int blockTime = 0; <= not used
    bool block = false;

    bool printing = true;

    // check if anything is in the ready queue at time 0
    while (!processes.empty() && processes[processIndex].getArrival() == time && processIndex < processes.size()) {
        // add to ready queue
        ready.push_back(processes[processIndex]);
        processIndex++;
    }
    
    if (rr_check)
        myfile << "time 0ms: Simulator started for RR with time slice " << timeSlice << "ms ";
    else
        myfile << "time 0ms: Simulator started for FCFS ";
    printQueue(ready, myfile);

    // none of the processes can run.. all the processes just arrive
    if (timeSlice <= 0) {
        for (unsigned int i = 0; i < processes.size(); i++) {
            if (time == 1000) {
                printing = false;
            }
            ready.push_back(processes[i]);
            if (printing) {
                myfile << "time " << processes[i].getArrival() << "ms: Process " << alphabet[processes[i].getId()] << 
                " arrived; added to ready queue ";
                printQueue(ready, myfile);
            }
            time = processes[i].getArrival();
        }
    }

    // check if all processes have finished
    while (finished != processes.size() && timeSlice > 0) {
        if (time == 1000) {
            printing = false;
        }
        if (addTime == 0 && block) {
            if (prevProcess.getId() != -1) {
                ready.push_back(prevProcess);
                Process temp;
                prevProcess = temp;
            }
        }
       
        // CPU can be utilizied when there is no context switch time
        if (conSwitch == 0) {
            // if no process is using the CPU
            if (!usage) {
                // new process
                if (currProcess.getId() == -1 && !ready.empty()) {
                    currProcess = ready[0];
                    // process just started
                    if (printing) {
                        if (currProcess.getCurrTime() == 0) {
                            myfile << "time "<< time << "ms: Process " << alphabet[currProcess.getId()] << 
                            " started using the CPU for " << currProcess.getCpuBurst() <<"ms burst ";
                        } else {
                            myfile << "time "<< time << "ms: Process " << alphabet[currProcess.getId()] << 
                            " started using the CPU for remaining " << currProcess.getRemainingTime() << "ms of " 
                            << currProcess.getCpuBurst() << "ms burst ";
                        }
                    }
                    
                    currProcess.addSwitches();
                    ready.erase(ready.begin());
                    if (printing) {
                        printQueue(ready, myfile);
                    }
                    
                    usage = true;
                } // next process has been determined
                else if (currProcess.getId() > 0) {
                    if (printing) {
                        if (currProcess.getCurrTime() == 0) {
                            myfile << "time "<< time << "ms: Process " << alphabet[currProcess.getId()] << 
                            " started using the CPU for " << currProcess.getCpuBurst() <<"ms burst ";
                        } else {
                            myfile << "time "<< time << "ms: Process " << alphabet[currProcess.getId()] << 
                            " started using the CPU for remaining " << currProcess.getRemainingTime() << "ms of " 
                            << currProcess.getCpuBurst() << "ms burst ";
                        }
                    }
                    
                    // remove if not already removed...
                    currProcess.addSwitches();
                    if (!ready.empty() && currProcess.getId() == ready[0].getId()) {
                        ready.erase(ready.begin());
                    }
                    if (printing) {
                        printQueue(ready, myfile);
                    }
                    
                    usage = true;
                }
            }

            // finishes a CPU Burst
            if (currProcess.getId() != -1 && currProcess.getCurrTime() == currProcess.getCpuBurst()) {

                // if we are on last burst, we don't do I/O, we are done                
                if (currProcess.getCurrBurst() == currProcess.getNumBurst() - 1) {
                    finished++;
                    currProcess.isDone();
                    int turnaroundTime = time - currProcess.getStartTime() + timeContextSwitch/2;
                    int waitTime = turnaroundTime - currProcess.getCpuBurst() - timeContextSwitch;
                    currProcess.setTurnaroundTime(turnaroundTime);
                    currProcess.setWaitTime(waitTime);
                    myfile << "time " << time << "ms: Process " << alphabet[currProcess.getId()] << 
                    " terminated ";
                    printQueue(ready, myfile);
                    done.push_back(currProcess);
                    
                    // added when process finishes
                    conSwitch += timeContextSwitch/2;
                    Process temp;
                    currProcess = temp;
                    usage = false;

                    // something in the ready queue
                    if (!ready.empty()) {
                        conSwitch += timeContextSwitch/2;
                        currProcess = ready[0];
                    }
                
                // not on the last burst
                } else {
                    // print finish
                    int rem = currProcess.getRemainingBurst() - 1;
                    int turnaroundTime = time - currProcess.getStartTime() + timeContextSwitch/2;
                    int waitTime = turnaroundTime - currProcess.getCpuBurst() - timeContextSwitch;
                    currProcess.setTurnaroundTime(turnaroundTime);
                    currProcess.setWaitTime(waitTime);
                    if (printing) {
                        myfile << "time " << time << "ms: Process " << alphabet[currProcess.getId()] << 
                        " completed a CPU burst; " << rem;
                        if (rem == 1)
                            myfile << " burst to go ";
                        else
                            myfile << " bursts to go ";
                        printQueue(ready, myfile);
                    }
                    
                    // block on I/O
                    if (printing) {
                        myfile << "time " << time << "ms: Process " << alphabet[currProcess.getId()] << 
                        " switching out of CPU; will block on I/O until time ";
                    }

                    conSwitch += timeContextSwitch/2;
                    currProcess.setIOBurstTime(time + timeContextSwitch/2 + currProcess.getCurrIOBurst());
                    if (printing) {
                        myfile << currProcess.getCurrIOBurst() << "ms ";
                        printQueue(ready, myfile);
                    }
                    
                    // reset the currentTime for the next CPU Burst, increment burst and add to wait
                    currProcess.setCurrTime(0);
                    currProcess.addCurrBurst();
                    wait.push_back(currProcess);

                    Process temp;
                    currProcess = temp;
                    usage = false;
            
                    if (!ready.empty()) {
                        conSwitch += timeContextSwitch/2;
                        currProcess = ready[0];
                        addTime += timeContextSwitch/2;
                        block = true;
                    }
                }
                currSlice = timeSlice;
            // if it reaches the time slice
            } else if (currProcess.getId() != -1 && currProcess.getCurrTime() != 0 && currSlice == 0) {
                if (printing) {
                    myfile << "time " << time << "ms: Time slice expired; ";
                }
                // if queue is empty
                if (ready.empty()) {
                    if (printing) {
                        myfile << "no preemption because ready queue is empty [Q: empty]" << std::endl;
                    }
                // queue isn't empty
                } else {
                    if (printing) {
                        myfile << "process "<< alphabet[currProcess.getId()] << " preempted with "<< 
                        currProcess.getRemainingTime() <<"ms remaining ";
                        printQueue(ready, myfile);
                    }
                    
                    currProcess.addPreemp();
                    prevProcess = currProcess;
                    conSwitch += timeContextSwitch/2;

                    // added when process finishes
                    Process temp;
                    currProcess = temp;
                    usage = false;

                    // something in the ready queue
                    if (!ready.empty()) {
                        conSwitch += timeContextSwitch/2;
                        addTime += timeContextSwitch/2;
                        currProcess = ready[0];
                        block = true;
                    }
                }
                currSlice = timeSlice;
            }

            if (currProcess.getId() != -1 && usage) {
                // add time
                currProcess.addCpuTime();
                currProcess.addCurrTime();
                currSlice--;
            }
        }

        // process can always enter the ready queue, finish from I/O or arrival
        
        // check if a process needs to enter the queue via arrival time
        while (processes[processIndex].getArrival() == time && processIndex < processes.size()) {
            // add to ready queue
            processes[processIndex].setStartTime(time);
            ready.push_back(processes[processIndex]);
            if (printing) {
                myfile << "time " << time << "ms: Process " << alphabet[processes[processIndex].getId()] << 
                " arrived; added to ready queue ";
                printQueue(ready, myfile);
            }

            // context switch occurs when there is no current process running.
            if (!usage && currProcess.getId() == -1 && conSwitch == 0) {
                conSwitch += timeContextSwitch/2;
                currProcess = ready[0];
            }

            processIndex++;
        }

        int added = false;
        if (wait.size() > 1) {
            sort(wait.begin(), wait.end(), compareId);
        }
        // check if a process needs to enter the ready queue via finishing I/O
        for (unsigned int i = 0; i < wait.size(); i++) {
            // if no time left, add it back to the ready queue
            if (wait[i].getPrevIOBurst() == time) {
                wait[i].setStartTime(time);
                ready.push_back(wait[i]);
                if (printing) {
                    myfile << "time " << time << "ms: Process " << alphabet[wait[i].getId()] << 
                    " completed I/O; added to ready queue ";
                    printQueue(ready, myfile);
                }
                wait.erase(wait.begin()+i);
                i--;
                added = true;
            }
        }
        // context switch occurs when there is no current process running.
        if (added && !usage && currProcess.getId() == -1) {
            conSwitch += timeContextSwitch/2;
            currProcess = ready[0];
            ready.erase(ready.begin());
        }
        
        // remove if process already chosen
        if (addTime == 0 && block) {
            if (!usage && !ready.empty() && currProcess.getId() == ready[0].getId()) {
                ready.erase(ready.begin());
                block = false;
            }
        }
        if (addTime > 0) {
            addTime--;
        }
        if (conSwitch > 0) {
            conSwitch--;
        }
        time++;
    }
    time += timeContextSwitch/2 - 1;
    if (rr_check) {
        myfile << "time " << time << "ms: Simulator ended for RR [Q: empty]" << std::endl;
        simout(simfile, done, time, 3);
    }
    else {
        myfile << "time " << time << "ms: Simulator ended for FCFS [Q: empty]" << std::endl;
        myfile << std::endl;
        simout(simfile, done, time, 0);
    }
}

void readfile(std::fstream& file, std::fstream& simout, bool isSimout) {
    // set file pointer to the beginning of the file
    file.seekg ( 0, std::ios::beg );

    if (isSimout) {
        std::string line;
        while(getline(file, line)) {
            simout << line << std::endl;
        }
    } 
    else {
        std::string line;
        while(getline(file, line)) {
            std::cout << line << std::endl;
        }
    }
}

int main(int argc, char* argv[]) {
    // If improper command-line arguments are given, report an error message to 
    // stderr and abort further program execution
    if(argc != 8) {
        std::cerr <<  "Usage: ./a.out <Number Processes> <Seed> <lamda> " << 
        "<Upper Bound> <Time Context Switch> <Alpha> <Time Slice>" << std::endl; 
        abort();
    }

    // Number of processes to simulate
    int n = std::stoi(*(argv+1));

    // Serves as the seed for the pseudo-random number sequence
    int seed = std::stoi(*(argv+2));

    // Use for exponential distribution
    float lambda = std::stof(*(argv+3));

    // Represents the upper bound for valid pseudo-random numbers. As an example, 
    // if this is set to 3000, all generated values above 3000 should be skipped
    int upperBound = std::stoi(*(argv+4));
    if (upperBound <= 0) {
        std::cerr << "Upper bound must be a positive number greater than 0" << std::endl;
        return EXIT_FAILURE;
    }

    // Define t_cs as the time, in milliseconds, that it takes to perform a 
    // context switch
    int timeContextSwitch = std::stoi(*(argv+5));

    // Use for exponential averaging
    float alpha = std::stof(*(argv+6));

    // Time slice measured in milliseconds that is used in the RR algorithm.
    int timeSlice = std::stoi(*(argv+7));

    // Use srand48() with a given seed before simulating each scheduling algorithm
    srand48(seed);

    // Generate n processes
    std::vector<Process> processes;
    for(int i = 0; i < n; i++) {
        processes.push_back(Process(i+1, upperBound, lambda));
        processes[i].print();
    }
    std::cout << std::endl;

    // Sort processes by ascending arrival time and id
    sort(processes.begin(), processes.end(), compare);

    // Create file for write
    std::fstream myfile ("simout.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
    if(!myfile.is_open()) {
        std::cerr << "Error: cannot open simout.txt for writing" << std::endl;
    }
    std::fstream file1 ("FCFS.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
    if(!myfile.is_open()) {
        std::cerr << "Error: cannot open FCFS.txt for writing" << std::endl;
    }
    std::fstream file2 ("SJF.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
    if(!myfile.is_open()) {
        std::cerr << "Error: cannot open SJF.txt for writing" << std::endl;
    }
    std::fstream file3 ("SRT.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
    if(!myfile.is_open()) {
        std::cerr << "Error: cannot open SRT.txt for writing" << std::endl;
    }
    std::fstream file4 ("RR.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
    if(!myfile.is_open()) {
        std::cerr << "Error: cannot open RR.txt for writing" << std::endl;
    }
    std::fstream file5 ("FCFS_simout.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
    if(!myfile.is_open()) {
        std::cerr << "Error: cannot open RR.txt for writing" << std::endl;
    }
    std::fstream file6 ("SJF_simout.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
    if(!myfile.is_open()) {
        std::cerr << "Error: cannot open RR.txt for writing" << std::endl;
    }
    std::fstream file7 ("SRT_simout.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
    if(!myfile.is_open()) {
        std::cerr << "Error: cannot open RR.txt for writing" << std::endl;
    }
    std::fstream file8 ("RR_simout.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
    if(!myfile.is_open()) {
        std::cerr << "Error: cannot open RR.txt for writing" << std::endl;
    }
        
    pid_t pids[4];
    for (int i = 0; i < 4; i++) {
        pids[i] = fork();
        if (pids[i] == -1) {
            perror("fork() failed");
            return EXIT_FAILURE;
        }
          
        // child
        if (pids[i] == 0) {
            //std::cout << "i is " << i << " pid of this child is " << getpid() << std::endl;
            if (i == 0) {
                fcfs(processes, timeContextSwitch, file1, file5);
            } else if (i == 1) {
                sjf(processes, alpha, timeContextSwitch/2, file2, file6);
            } else if (i == 2) {
                srt(processes, alpha, timeContextSwitch/2, file3, file7);
            } else{
                rr(processes, timeSlice, timeContextSwitch, true, file4, file8);
            }
            return EXIT_SUCCESS;
        }
    }
    for (int j = 0; j < 4; j++) {
        waitpid(pids[j], NULL, 0);
    }

    readfile(file1, myfile, false);
    readfile(file2, myfile, false);
    readfile(file3, myfile, false);
    readfile(file4, myfile, false);
    readfile(file5, myfile, true);
    readfile(file6, myfile, true);
    readfile(file7, myfile, true);
    readfile(file8, myfile, true);
    
    // Run each algorithm
    // fcfs(processes, timeContextSwitch, myfile);
    // std::cout << std::endl;
    // sjf(processes, alpha, timeContextSwitch/2, myfile);
    // std::cout << std::endl;
    // srt(processes, alpha, timeContextSwitch/2, myfile);
    // std::cout << std::endl;
    // rr(processes, timeSlice, timeContextSwitch, true, myfile);

    // Close file
    myfile.close();
    file1.close();
    file2.close();
    file3.close();
    file4.close();

    return EXIT_SUCCESS;
}