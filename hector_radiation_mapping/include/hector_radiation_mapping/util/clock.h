#ifndef RADIATION_MAPPING_CLOCK_H
#define RADIATION_MAPPING_CLOCK_H

class Clock {
public:

    /**
     * @brief Start the clock
     */
    void tick() {
        end_ = std::chrono::steady_clock::time_point{};
        start_ = std::chrono::steady_clock::now();
    }

    /**
     * @brief Stop the clock
     * @param print  Print the elapsed time
     * @return  Elapsed time in seconds
     */
    long tock(bool print = false) {
        end_ = std::chrono::steady_clock::now();
        long time = std::chrono::duration_cast<std::chrono::milliseconds>(end_ - start_).count();
        if (print)
            ROS_INFO_STREAM("Time elapsed(ms)=" << time);
        return time;
    }

private:
    std::chrono::steady_clock::time_point start_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end_ = {};
};

#endif //RADIATION_MAPPING_CLOCK_H