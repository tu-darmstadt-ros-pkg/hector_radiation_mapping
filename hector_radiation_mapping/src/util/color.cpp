#include "pch.h"
#include "util/color.h"

Eigen::MatrixX4d Color::applyColorMap(const Eigen::MatrixX3d &colorMap, const Vector &values, bool invert) {
    if (colorMap.rows() < 2) {
        STREAM("ColorMap does not have enough samples, it needs 2 or more but currently has " << colorMap.rows());
        return {};
    }

    // normalize values to be between 0 and 1
    double min = values.minCoeff();
    double max = values.maxCoeff();
    Vector values_norm;
    if (min == max) {
        values_norm.setZero(values.size());
    } else {
        values_norm = (values.array() - min) * (1 / (max - min));
    }
    if (invert) {
        values_norm = 1 - values_norm.array();
    }

    // interpolate values
    int length = colorMap.rows() - 1;
    Vector capped = values_norm.array().max(0.0).min(1.0);

    Eigen::VectorXi a = (capped * length).cast<int>();
    Eigen::VectorXi b = (a.array() + 1).min(length);
    Vector f = capped * length - a.cast<double>();

    Eigen::Vector3d tmp;
    Eigen::MatrixX4d Res(values_norm.size(), 4);
    Res.conservativeResize(values_norm.size(), 4);
    for (int i = 0; i < values_norm.size(); i++) {
        tmp = (colorMap.row(a[i]) + (colorMap.row(b[i]) - colorMap.row(a[i])) * f[i]);
        Res.row(i) << tmp.transpose(), 1.0;
    }
    return Res;
}

void Color::saveColorMap(const Eigen::MatrixX3d &colorMap, const std::string &filename) {
    double pos = 0.0;
    double step = 1.0 / (colorMap.rows() - 1);
    std::vector<std::string> data;
    for (int i = 0; i < colorMap.rows(); ++i) {
        int r = colorMap(i, 0) * 255.0;
        int g = colorMap(i, 1) * 255.0;
        int b = colorMap(i, 2) * 255.0;
        std::string color = "<step "
                            "r=\"" + std::to_string(r) +
                            "\" g=\"" + std::to_string(g) +
                            "\" b=\"" + std::to_string(b) +
                            "\" pos=\"" + std::to_string(pos) +
                            "\" />";
        data.push_back(color);
        pos += step;
    }

    std::string path = ros::package::getPath("hector_radiation_mapping");
    path += "/scripts";
    if (!std::filesystem::exists(path)) {
        std::filesystem::create_directories(path);
    }
    std::ofstream file(path + "/" + filename + ".xml", std::ios::trunc);
    STREAM_DEBUG(path);

    if (file.is_open()) {
        for (const auto& value : data) {
            file << value << '\n';
        }
        file.close();
        std::cout << "Data saved to file successfully.\n";
    } else {
        std::cerr << "Unable to open the file.\n";
    }
}
