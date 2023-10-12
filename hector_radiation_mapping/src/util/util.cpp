/*
 * Copyright (C) 2023  Martin Volz, Jonas Suess
 *
 * This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "util/util.h"
#include "util/parameters.h"

std::vector<double> Util::vectorXdToStdVector(const Vector &v_) {
    std::vector<double> vec;
    vec.resize(v_.size());
    Vector::Map(&vec[0], v_.size()) = v_;
    return vec;
}

bool Util::appendColWise(Matrix &X, const Matrix &Y, bool checkNumRows) {
    long xr = X.rows();
    long xc = X.cols();
    long yr = Y.rows();
    long yc = Y.cols();

    if (xc <= 0) {
        X.conservativeResize(yr, yc);
        X.block(0, 0, yr, yc) = Y;
        return true;
    }
    if (checkNumRows) {
        if (xr != yr) {
            return false;
        }
        X.conservativeResize(xr, yc + xc);
        X.block(0, xc, yr, yc) = Y;
        return true;
    }
    X.conservativeResize((xr > yr ? xr : yr), xc + yc);
    X.block(0, xc, yr, yc) = Y;
    return true;
}

bool Util::appendRowWise(Matrix &X, const Matrix &Y, bool checkNumCols) {
    long xr = X.rows();
    long xc = X.cols();
    long yr = Y.rows();
    long yc = Y.cols();

    if (xr <= 0) {
        X.conservativeResize(yr, yc);
        X.block(0, 0, yr, yc) = Y;
        return true;
    }

    if (checkNumCols) {
        if (xc != yc) {
            return false;
        }
        X.conservativeResize(xr + yr, yc);
        X.block(xr, 0, yr, yc) = Y;
        return true;
    }
    X.conservativeResize(xr + yr, (xc > yc ? xc : yc));
    X.block(xr, 0, yr, yc) = Y;
    return true;
}

void Util::appendToVector(Vector &v1, const Vector &v2) {
    Vector vec_joined(v1.rows() + v2.rows());
    vec_joined << v1, v2;
    v1 = vec_joined;
}

void Util::appendToVector(Vector &v1, double value) {
    v1.conservativeResize(v1.rows() + 1);
    v1[v1.rows() - 1] = value;
}

void Util::removeRow(Matrix &matrix, unsigned int rowToRemove) {
    unsigned int numRows = matrix.rows() - 1;
    unsigned int numCols = matrix.cols();

    if (rowToRemove < numRows)
        matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols)
                = matrix.block(rowToRemove + 1, 0, numRows - rowToRemove, numCols);

    matrix.conservativeResize(numRows, numCols);
}

void Util::removeColumn(Matrix &matrix, unsigned int colToRemove) {
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols() - 1;

    if (colToRemove < numCols)
        matrix.block(0, colToRemove, numRows, numCols - colToRemove)
                = matrix.block(0, colToRemove + 1, numRows, numCols - colToRemove);

    matrix.conservativeResize(numRows, numCols);
}

double Util::getRamUsageMB() {
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    return 0.001024 * usage.ru_maxrss;
}

void Util::exportVectorToTxtFile(const std::vector<double>& data, const std::string& path, const std::string& filename, TxtExportType type, bool print) {
    std::ofstream file;
    std::string filePath;
    switch (type) {
        case TxtExportType::REPLACE: {
            filePath = path + "/" + filename + ".txt";
            file = std::ofstream(filePath, std::ios::trunc);
            break;
        }
        case TxtExportType::APPEND: {
            filePath = path + "/" + filename + ".txt";
            file = std::ofstream(filePath, std::ios::app);
            break;
        }
        case TxtExportType::NEW: {
            std::string uniqueFilename = getUniqueFileName(path, filename);
            filePath = path + "/" + uniqueFilename + ".txt";
            file = std::ofstream(filePath, std::ios::trunc);
            break;
        }
        default:
            ROS_INFO_STREAM("Unknown export type.");
            break;
    }

    if (file.is_open()) {
        if (print) ROS_INFO_STREAM("Writing file at " << filePath << " with mode " << static_cast<int>(type) << ".");
        for (const auto& value : data) {
            file << value << '\n';
        }
        file.close();
        if (print) ROS_INFO_STREAM("Data saved to file successfully.");
    } else {
        if (print) ROS_INFO_STREAM("Unable to open the file.");
    }
}

std::string Util::getExportPath(const std::string &folder) {
    std::string path = ros::package::getPath("hector_radiation_mapping") + Parameters::instance().exportPath + folder;
    if (!std::filesystem::exists(path)) {
        std::filesystem::create_directories(path);
    }
    return path;
}

bool Util::fileExists(const std::string& filePath) {
    struct stat buffer;
    return (stat(filePath.c_str(), &buffer) == 0);
}

std::string Util::getUniqueFileName(const std::string& path, const std::string& filename) {
    std::string uniqueFilename = filename;
    int index = 0;

    while (fileExists(path + "/" + uniqueFilename + ".txt")) {
        index++;
        std::ostringstream oss;
        oss << filename << "_" << index;
        uniqueFilename = oss.str();
    }
    return uniqueFilename;
}