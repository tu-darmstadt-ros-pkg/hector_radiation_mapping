#ifndef RADIATION_MAPPING_CLOCK_CPU_H
#define RADIATION_MAPPING_CLOCK_CPU_H

class ClockCPU {
public:

    /**
     * @brief Start the clock
     */
    void tick() {
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu_timespec_);
        start_ = (cpu_timespec_.tv_sec + 1e-9 * cpu_timespec_.tv_nsec);
        usage_ = {};
    }

    /**
     * @brief Stop the clock
     * @param print  Print the elapsed time and peak memory usage
     * @return  Elapsed time in seconds
     */
    double tock(bool print = false) {
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu_timespec_);
        end_ = (cpu_timespec_.tv_sec + 1e-9 * cpu_timespec_.tv_nsec);
        double time = end_ - start_;
        getrusage(RUSAGE_SELF, &usage_);

        double ramMB = 0.001024 * usage_.ru_maxrss;
        ROS_INFO_STREAM("Elapsed CPU time: " << time << " s || Peak memory usage: " << ramMB << " MB");
        return time;
    }

private:
    timespec cpu_timespec_ = {};
    double start_ = {};
    double end_ = {};
    rusage usage_ = {};
};

#endif //RADIATION_MAPPING_CLOCK_CPU_H
