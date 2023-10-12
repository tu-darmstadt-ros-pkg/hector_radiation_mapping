#include "util/tests.h"
#include "maps/gridmap.h"
#include "util/util.h"
#include "util/print.h"
#include "util/color.h"
#include "util/clock_cpu.h"

void testColor(){
    Eigen::Matrix<double, 3, 3> colorMap = (Eigen::Matrix<double, 3, 3>() << 0, 0, 0,1,1,1,2,2,2).finished();
    Print::print(colorMap);

    Vector test1(3);
    test1 << 0.0, 0.5, 1.0;
    Eigen::MatrixX4d res1 = Color::applyColorMap(colorMap, test1);
    Print::print(res1);
    ROS_INFO_STREAM("\n");

    Vector test2(3);
    test2 << -1.0, 0.0 ,1.0;
    Eigen::MatrixX4d res2 = Color::applyColorMap(colorMap, test2);
    Print::print(res2);
}

void Tests::runTests() {
    testColor();
}