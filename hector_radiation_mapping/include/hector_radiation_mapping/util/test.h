#ifndef RADIATION_MAPPING_TEST_H
#define RADIATION_MAPPING_TEST_H

class Test{
public:
    static bool calculateA12(double &I1, double &I2, double &res_x1, double &res_y1, double &res_x2, double &res_y2, const double r1, const double r2, const double r3, const double x1, const double x2, const double x3, const double y1, const double y2, const double y3) {
        //ROS_INFO_STREAM("TriRSD: A12" << r1 << " " << r2 << " " << r3 << " " << x1 << " " << x2 << " " << x3 << " " << y1 << " " << y2 << " " << y3);
        double r1_2 = r1*r1;
        double r2_2 = r2*r2;
        double r3_2 = r3*r3;
        double x1_2 = x1*x1;
        double x1_3 = x1_2*x1;
        double x1_4 = x1_2*x1_2;
        double x2_2 = x2*x2;
        double x2_3 = x2_2*x2;
        double x2_4 = x2_2*x2_2;
        double x3_2 = x3*x3;
        double x3_3 = x3_2*x3;
        double x3_4 = x3_2*x3_2;
        double y1_2 = y1*y1;
        double y1_3 = y1_2*y1;
        double y1_4 = y1_2*y1_2;
        double y2_2 = y2*y2;
        double y2_3 = y2_2*y2;
        double y2_4 = y2_2*y2_2;
        double y3_2 = y3*y3;
        double y3_3 = y3_2*y3;
        double y3_4 = y3_2*y3_2;

        double r1r2_2 = r1_2 * r2_2;
        double r1r3_2 = r1_2 * r3_2;
        double r2r3_2 = r2_2 * r3_2;
        double r1r2r3_2 = r1_2 * r2_2 * r3_2;

        double base = pow(x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2, 2);
        double radicand = (-(r1r2_2*x1_4 - 4*r1r2_2*x1_3*x2 + 6*r1r2_2*x1_2*x2_2 + 2*r1r2_2*x1_2*y1_2 - 4*r1r2_2*x1_2*y1*y2 + 2*r1r2_2*x1_2*y2_2 - 4*r1r2_2*x1*x2_3 - 4*r1r2_2*x1*x2*y1_2 + 8*r1r2_2*x1*x2*y1*y2 - 4*r1r2_2*x1*x2*y2_2 + r1r2_2*x2_4 + 2*r1r2_2*x2_2*y1_2 - 4*r1r2_2*x2_2*y1*y2 + 2*r1r2_2*x2_2*y2_2 + r1r2_2*y1_4 - 4*r1r2_2*y1_3*y2 + 6*r1r2_2*y1_2*y2_2 - 4*r1r2_2*y1*y2_3 + r1r2_2*y2_4 - 2*r1_2*r2*r3*x1_4 + 4*r1_2*r2*r3*x1_3*x2 + 4*r1_2*r2*r3*x1_3*x3 - 2*r1_2*r2*r3*x1_2*x2_2 - 8*r1_2*r2*r3*x1_2*x2*x3 - 2*r1_2*r2*r3*x1_2*x3_2 - 4*r1_2*r2*r3*x1_2*y1_2 + 4*r1_2*r2*r3*x1_2*y1*y2 + 4*r1_2*r2*r3*x1_2*y1*y3 - 2*r1_2*r2*r3*x1_2*y2_2 - 2*r1_2*r2*r3*x1_2*y3_2 + 4*r1_2*r2*r3*x1*x2_2*x3 + 4*r1_2*r2*r3*x1*x2*x3_2 + 4*r1_2*r2*r3*x1*x2*y1_2 - 8*r1_2*r2*r3*x1*x2*y1*y3 + 4*r1_2*r2*r3*x1*x2*y3_2 + 4*r1_2*r2*r3*x1*x3*y1_2 - 8*r1_2*r2*r3*x1*x3*y1*y2 + 4*r1_2*r2*r3*x1*x3*y2_2 - 2*r1_2*r2*r3*x2_2*x3_2 - 2*r1_2*r2*r3*x2_2*y1_2 + 4*r1_2*r2*r3*x2_2*y1*y3 - 2*r1_2*r2*r3*x2_2*y3_2 - 2*r1_2*r2*r3*x3_2*y1_2 + 4*r1_2*r2*r3*x3_2*y1*y2 - 2*r1_2*r2*r3*x3_2*y2_2 - 2*r1_2*r2*r3*y1_4 + 4*r1_2*r2*r3*y1_3*y2 + 4*r1_2*r2*r3*y1_3*y3 - 2*r1_2*r2*r3*y1_2*y2_2 - 8*r1_2*r2*r3*y1_2*y2*y3 - 2*r1_2*r2*r3*y1_2*y3_2 + 4*r1_2*r2*r3*y1*y2_2*y3 + 4*r1_2*r2*r3*y1*y2*y3_2 - 2*r1_2*r2*r3*y2_2*y3_2 + r1r3_2*x1_4 - 4*r1r3_2*x1_3*x3 + 6*r1r3_2*x1_2*x3_2 + 2*r1r3_2*x1_2*y1_2 - 4*r1r3_2*x1_2*y1*y3 + 2*r1r3_2*x1_2*y3_2 - 4*r1r3_2*x1*x3_3 - 4*r1r3_2*x1*x3*y1_2 + 8*r1r3_2*x1*x3*y1*y3 - 4*r1r3_2*x1*x3*y3_2 + r1r3_2*x3_4 + 2*r1r3_2*x3_2*y1_2 - 4*r1r3_2*x3_2*y1*y3 + 2*r1r3_2*x3_2*y3_2 + r1r3_2*y1_4 - 4*r1r3_2*y1_3*y3 + 6*r1r3_2*y1_2*y3_2 - 4*r1r3_2*y1*y3_3 + r1r3_2*y3_4 - 2*r1*r2_2*r3*x1_2*x2_2 + 4*r1*r2_2*r3*x1_2*x2*x3 - 2*r1*r2_2*r3*x1_2*x3_2 - 2*r1*r2_2*r3*x1_2*y2_2 + 4*r1*r2_2*r3*x1_2*y2*y3 - 2*r1*r2_2*r3*x1_2*y3_2 + 4*r1*r2_2*r3*x1*x2_3 - 8*r1*r2_2*r3*x1*x2_2*x3 + 4*r1*r2_2*r3*x1*x2*x3_2 + 4*r1*r2_2*r3*x1*x2*y2_2 - 8*r1*r2_2*r3*x1*x2*y2*y3 + 4*r1*r2_2*r3*x1*x2*y3_2 - 2*r1*r2_2*r3*x2_4 + 4*r1*r2_2*r3*x2_3*x3 - 2*r1*r2_2*r3*x2_2*x3_2 - 2*r1*r2_2*r3*x2_2*y1_2 + 4*r1*r2_2*r3*x2_2*y1*y2 - 4*r1*r2_2*r3*x2_2*y2_2 + 4*r1*r2_2*r3*x2_2*y2*y3 - 2*r1*r2_2*r3*x2_2*y3_2 + 4*r1*r2_2*r3*x2*x3*y1_2 - 8*r1*r2_2*r3*x2*x3*y1*y2 + 4*r1*r2_2*r3*x2*x3*y2_2 - 2*r1*r2_2*r3*x3_2*y1_2 + 4*r1*r2_2*r3*x3_2*y1*y2 - 2*r1*r2_2*r3*x3_2*y2_2 - 2*r1*r2_2*r3*y1_2*y2_2 + 4*r1*r2_2*r3*y1_2*y2*y3 - 2*r1*r2_2*r3*y1_2*y3_2 + 4*r1*r2_2*r3*y1*y2_3 - 8*r1*r2_2*r3*y1*y2_2*y3 + 4*r1*r2_2*r3*y1*y2*y3_2 - 2*r1*r2_2*r3*y2_4 + 4*r1*r2_2*r3*y2_3*y3 - 2*r1*r2_2*r3*y2_2*y3_2 - 2*r1*r2*r3_2*x1_2*x2_2 + 4*r1*r2*r3_2*x1_2*x2*x3 - 2*r1*r2*r3_2*x1_2*x3_2 - 2*r1*r2*r3_2*x1_2*y2_2 + 4*r1*r2*r3_2*x1_2*y2*y3 - 2*r1*r2*r3_2*x1_2*y3_2 + 4*r1*r2*r3_2*x1*x2_2*x3 - 8*r1*r2*r3_2*x1*x2*x3_2 + 4*r1*r2*r3_2*x1*x3_3 + 4*r1*r2*r3_2*x1*x3*y2_2 - 8*r1*r2*r3_2*x1*x3*y2*y3 + 4*r1*r2*r3_2*x1*x3*y3_2 - 2*r1*r2*r3_2*x2_2*x3_2 - 2*r1*r2*r3_2*x2_2*y1_2 + 4*r1*r2*r3_2*x2_2*y1*y3 - 2*r1*r2*r3_2*x2_2*y3_2 + 4*r1*r2*r3_2*x2*x3_3 + 4*r1*r2*r3_2*x2*x3*y1_2 - 8*r1*r2*r3_2*x2*x3*y1*y3 + 4*r1*r2*r3_2*x2*x3*y3_2 - 2*r1*r2*r3_2*x3_4 - 2*r1*r2*r3_2*x3_2*y1_2 + 4*r1*r2*r3_2*x3_2*y1*y3 - 2*r1*r2*r3_2*x3_2*y2_2 + 4*r1*r2*r3_2*x3_2*y2*y3 - 4*r1*r2*r3_2*x3_2*y3_2 - 2*r1*r2*r3_2*y1_2*y2_2 + 4*r1*r2*r3_2*y1_2*y2*y3 - 2*r1*r2*r3_2*y1_2*y3_2 + 4*r1*r2*r3_2*y1*y2_2*y3 - 8*r1*r2*r3_2*y1*y2*y3_2 + 4*r1*r2*r3_2*y1*y3_3 - 2*r1*r2*r3_2*y2_2*y3_2 + 4*r1*r2*r3_2*y2*y3_3 - 2*r1*r2*r3_2*y3_4 + r2r3_2*x2_4 - 4*r2r3_2*x2_3*x3 + 6*r2r3_2*x2_2*x3_2 + 2*r2r3_2*x2_2*y2_2 - 4*r2r3_2*x2_2*y2*y3 + 2*r2r3_2*x2_2*y3_2 - 4*r2r3_2*x2*x3_3 - 4*r2r3_2*x2*x3*y2_2 + 8*r2r3_2*x2*x3*y2*y3 - 4*r2r3_2*x2*x3*y3_2 + r2r3_2*x3_4 + 2*r2r3_2*x3_2*y2_2 - 4*r2r3_2*x3_2*y2*y3 + 2*r2r3_2*x3_2*y3_2 + r2r3_2*y2_4 - 4*r2r3_2*y2_3*y3 + 6*r2r3_2*y2_2*y3_2 - 4*r2r3_2*y2*y3_3 + r2r3_2*y3_4)/(r1r2r3_2*base));
        if (radicand < 0) {
            return false;
        }
        //ROS_INFO_STREAM("TriRSD: A12 radicand" << radicand << " r1r2r3_2 " << r1r2r3_2 << " base " << base);
        double positive_root = sqrt(radicand);
        double negative_root = -positive_root;
        double r1r2r3_2_pr = r1r2r3_2 * positive_root;
        double r1r2r3_2_nr = r1r2r3_2 * negative_root;

        double part1 = - x1_2*y2_2
                       - x2_2*y1_2
                       - x1_2*y3_2
                       - x3_2*y1_2
                       - x2_2*y3_2
                       - x3_2*y2_2;
        double part2 = + x1*x2*y3_2
                       + x1*x3*y2_2
                       + x2*x3*y1_2
                       + x1_2*y2*y3
                       + x2_2*y1*y3
                       + x3_2*y1*y2;
        double part3 = + x1*x2*y1*y2
                       - x1*x2*y1*y3
                       - x1*x3*y1*y2
                       - x1*x2*y2*y3
                       + x1*x3*y1*y3
                       - x2*x3*y1*y2
                       - x1*x3*y2*y3
                       - x2*x3*y1*y3
                       + x2*x3*y2*y3;
        double sum = part1 + 2 * part2 + 2 * part3;

        double nominator_part = r1*r2r3_2*x1_2*x2_2 + r1_2*r2*r3_2*x1_2*x2_2 - 2*r1r2_2*r3*x1_2*x2_2 + r1*r2r3_2*x1_2*x3_2 - 2*r1_2*r2*r3_2*x1_2*x3_2 + r1r2_2*r3*x1_2*x3_2 - 2*r1*r2r3_2*x2_2*x3_2 + r1_2*r2*r3_2*x2_2*x3_2 + r1r2_2*r3*x2_2*x3_2 + r1*r2r3_2*x1_2*y2_2 + r1*r2r3_2*x2_2*y1_2 + r1_2*r2*r3_2*x1_2*y2_2 + r1_2*r2*r3_2*x2_2*y1_2 + r1*r2r3_2*x1_2*y3_2 + r1*r2r3_2*x3_2*y1_2 + r1r2_2*r3*x1_2*y3_2 + r1r2_2*r3*x3_2*y1_2 + r1_2*r2*r3_2*x2_2*y3_2 + r1_2*r2*r3_2*x3_2*y2_2 + r1r2_2*r3*x2_2*y3_2 + r1r2_2*r3*x3_2*y2_2 + r1*r2r3_2*y1_2*y2_2 + r1_2*r2*r3_2*y1_2*y2_2 - 2*r1r2_2*r3*y1_2*y2_2 + r1*r2r3_2*y1_2*y3_2 - 2*r1_2*r2*r3_2*y1_2*y3_2 + r1r2_2*r3*y1_2*y3_2 - 2*r1*r2r3_2*y2_2*y3_2 + r1_2*r2*r3_2*y2_2*y3_2 + r1r2_2*r3*y2_2*y3_2 - r1*r2r3_2*x1*x2_3 - r1_2*r2*r3_2*x1_3*x2 + r1r2_2*r3*x1*x2_3 + r1r2_2*r3*x1_3*x2 - r1*r2r3_2*x1*x3_3 + r1_2*r2*r3_2*x1*x3_3 + r1_2*r2*r3_2*x1_3*x3 - r1r2_2*r3*x1_3*x3 + r1*r2r3_2*x2*x3_3 + r1*r2r3_2*x2_3*x3 - r1_2*r2*r3_2*x2*x3_3 - r1r2_2*r3*x2_3*x3 - r1*r2r3_2*y1*y2_3 - r1_2*r2*r3_2*y1_3*y2 + r1r2_2*r3*y1*y2_3 + r1r2_2*r3*y1_3*y2 - r1*r2r3_2*y1*y3_3 + r1_2*r2*r3_2*y1*y3_3 + r1_2*r2*r3_2*y1_3*y3 - r1r2_2*r3*y1_3*y3 + r1*r2r3_2*y2*y3_3 + r1*r2r3_2*y2_3*y3 - r1_2*r2*r3_2*y2*y3_3 - r1r2_2*r3*y2_3*y3
                                + r1*r2r3_2*x1*x2*x3_2 + r1*r2r3_2*x1*x2_2*x3 - 2*r1*r2r3_2*x1_2*x2*x3 + r1_2*r2*r3_2*x1*x2*x3_2 - 2*r1_2*r2*r3_2*x1*x2_2*x3 + r1_2*r2*r3_2*x1_2*x2*x3 - 2*r1r2_2*r3*x1*x2*x3_2 + r1r2_2*r3*x1*x2_2*x3 + r1r2_2*r3*x1_2*x2*x3 - r1_2*r2*r3_2*x1*x2*y1_2 + r1r2_2*r3*x1*x2*y1_2 - r1*r2r3_2*x1*x2*y2_2 + r1_2*r2*r3_2*x1*x3*y1_2 + r1r2_2*r3*x1*x2*y2_2 - r1r2_2*r3*x1*x3*y1_2 - r1*r2r3_2*x1*x2*y3_2 - r1*r2r3_2*x1*x3*y2_2 - 2*r1*r2r3_2*x2*x3*y1_2 - r1_2*r2*r3_2*x1*x2*y3_2 - 2*r1_2*r2*r3_2*x1*x3*y2_2 - r1_2*r2*r3_2*x2*x3*y1_2 - 2*r1r2_2*r3*x1*x2*y3_2 - r1r2_2*r3*x1*x3*y2_2 - r1r2_2*r3*x2*x3*y1_2 - r1*r2r3_2*x1*x3*y3_2 + r1*r2r3_2*x2*x3*y2_2 + r1_2*r2*r3_2*x1*x3*y3_2 - r1r2_2*r3*x2*x3*y2_2 + r1*r2r3_2*x2*x3*y3_2 - r1_2*r2*r3_2*x2*x3*y3_2 - r1_2*r2*r3_2*x1_2*y1*y2 + r1r2_2*r3*x1_2*y1*y2 - r1*r2r3_2*x2_2*y1*y2 + r1_2*r2*r3_2*x1_2*y1*y3 - r1r2_2*r3*x1_2*y1*y3 + r1r2_2*r3*x2_2*y1*y2 - 2*r1*r2r3_2*x1_2*y2*y3 - r1*r2r3_2*x2_2*y1*y3 - r1*r2r3_2*x3_2*y1*y2 - r1_2*r2*r3_2*x1_2*y2*y3 - 2*r1_2*r2*r3_2*x2_2*y1*y3 - r1_2*r2*r3_2*x3_2*y1*y2 - r1r2_2*r3*x1_2*y2*y3 - r1r2_2*r3*x2_2*y1*y3 - 2*r1r2_2*r3*x3_2*y1*y2 + r1*r2r3_2*x2_2*y2*y3 - r1*r2r3_2*x3_2*y1*y3 + r1_2*r2*r3_2*x3_2*y1*y3 - r1r2_2*r3*x2_2*y2*y3 + r1*r2r3_2*x3_2*y2*y3 - r1_2*r2*r3_2*x3_2*y2*y3 + r1*r2r3_2*y1*y2*y3_2 + r1*r2r3_2*y1*y2_2*y3 - 2*r1*r2r3_2*y1_2*y2*y3 + r1_2*r2*r3_2*y1*y2*y3_2 - 2*r1_2*r2*r3_2*y1*y2_2*y3 + r1_2*r2*r3_2*y1_2*y2*y3 - 2*r1r2_2*r3*y1*y2*y3_2 + r1r2_2*r3*y1*y2_2*y3 + r1r2_2*r3*y1_2*y2*y3
                                - 4*r1r2_2*r3*x1*x2*y1*y2 + 2*r1_2*r2*r3_2*x1*x2*y1*y3 + 2*r1_2*r2*r3_2*x1*x3*y1*y2 + 2*r1r2_2*r3*x1*x2*y1*y3 + 2*r1r2_2*r3*x1*x3*y1*y2 + 2*r1*r2r3_2*x1*x2*y2*y3 + 2*r1*r2r3_2*x2*x3*y1*y2 - 4*r1_2*r2*r3_2*x1*x3*y1*y3 + 2*r1r2_2*r3*x1*x2*y2*y3 + 2*r1r2_2*r3*x2*x3*y1*y2 + 2*r1*r2r3_2*x1*x3*y2*y3 + 2*r1*r2r3_2*x2*x3*y1*y3 + 2*r1_2*r2*r3_2*x1*x3*y2*y3 + 2*r1_2*r2*r3_2*x2*x3*y1*y3 - 4*r1*r2r3_2*x2*x3*y2*y3;
        double denominator = r1r2_2*x1_2 - 2*r1r2_2*x1*x2 + r1r2_2*x2_2 + r1r2_2*y1_2 - 2*r1r2_2*y1*y2 + r1r2_2*y2_2 - 2*r1_2*r2*r3*x1_2 + 2*r1_2*r2*r3*x1*x2 + 2*r1_2*r2*r3*x1*x3 - 2*r1_2*r2*r3*x2*x3 - 2*r1_2*r2*r3*y1_2 + 2*r1_2*r2*r3*y1*y2 + 2*r1_2*r2*r3*y1*y3 - 2*r1_2*r2*r3*y2*y3 + r1r3_2*x1_2 - 2*r1r3_2*x1*x3 + r1r3_2*x3_2 + r1r3_2*y1_2 - 2*r1r3_2*y1*y3 + r1r3_2*y3_2 + 2*r1*r2_2*r3*x1*x2 - 2*r1*r2_2*r3*x1*x3 - 2*r1*r2_2*r3*x2_2 + 2*r1*r2_2*r3*x2*x3 + 2*r1*r2_2*r3*y1*y2 - 2*r1*r2_2*r3*y1*y3 - 2*r1*r2_2*r3*y2_2 + 2*r1*r2_2*r3*y2*y3 - 2*r1*r2*r3_2*x1*x2 + 2*r1*r2*r3_2*x1*x3 + 2*r1*r2*r3_2*x2*x3 - 2*r1*r2*r3_2*x3_2 - 2*r1*r2*r3_2*y1*y2 + 2*r1*r2*r3_2*y1*y3 + 2*r1*r2*r3_2*y2*y3 - 2*r1*r2*r3_2*y3_2 + r2r3_2*x2_2 - 2*r2r3_2*x2*x3 + r2r3_2*x3_2 + r2r3_2*y2_2 - 2*r2r3_2*y2*y3 + r2r3_2*y3_2;
        I1 = (nominator_part + r1r2r3_2_pr * sum) / denominator;
        I2 = (nominator_part + r1r2r3_2_nr * sum) / denominator;
        //ROS_INFO_STREAM("Nominator " << nominator_part << " Denominator " << denominator);
        res_x1 = -((y2-y3)*((y2_2-y1_2)+(x2_2-x1_2)+(I1/r1-I1/r2))-(y1-y2)*((y3_2-y2_2)+(x3_2-x2_2)+(I1/r2-I1/r3)))
                /(2*((x1-x2)*(y2-y3)-(x2-x3)*(y1-y2)));
        res_y1 = -((x2-x3)*((x2_2-x1_2)+(y2_2-y1_2)+(I1/r1-I1/r2))-(x1-x2)*((x3_2-x2_2)+(y3_2-y2_2)+(I1/r2-I1/r3)))
                 /(2*((y1-y2)*(x2-x3)-(y2-y3)*(x1-x2)));
        res_x2 = -((y2-y3)*((y2_2-y1_2)+(x2_2-x1_2)+(I2/r1-I2/r2))-(y1-y2)*((y3_2-y2_2)+(x3_2-x2_2)+(I2/r2-I2/r3)))
                 /(2*((x1-x2)*(y2-y3)-(x2-x3)*(y1-y2)));
        res_y2 = -((x2-x3)*((x2_2-x1_2)+(y2_2-y1_2)+(I2/r1-I2/r2))-(x1-x2)*((x3_2-x2_2)+(y3_2-y2_2)+(I2/r2-I2/r3)))
                 /(2*((y1-y2)*(x2-x3)-(y2-y3)*(x1-x2)));
        return true;
    }
};

#endif //RADIATION_MAPPING_TEST_H

