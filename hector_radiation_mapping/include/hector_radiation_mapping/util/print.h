#ifndef RADIATION_MAPPING_PRINT_H
#define RADIATION_MAPPING_PRINT_H

const Eigen::IOFormat CleanFmt = Eigen::IOFormat(4, 0, ", ", "\n", "[", "]");

class Print{
public:
    /**
     * Print a matrix to the console
     * @param mat matrix to print
     */
    template<typename Derived>
    static void print(const Eigen::MatrixBase<Derived>& mat){
        ROS_INFO_STREAM("\n"<<mat.format(CleanFmt));
    }

    /**
     * Convert a matrix to a string
     * @param mat matrix to convert
     * @return string representation of the matrix
     */
    template<typename Derived>
    std::string toStr(const Eigen::MatrixBase<Derived>& mat) {
        std::stringstream ss;
        ss << mat.transpose().format(CleanFmt);
        return ss.str();;
    }
};

#endif //RADIATION_MAPPING_PRINT_H