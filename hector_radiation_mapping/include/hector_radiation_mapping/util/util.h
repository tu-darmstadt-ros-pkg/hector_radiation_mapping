#ifndef RADIATION_MAPPING_UTIL_H
#define RADIATION_MAPPING_UTIL_H

class Util{
public:
    enum class TxtExportType {
        REPLACE,
        APPEND,
        NEW
    };

    /**
     * Exports a vector to a txt file. The file will be created if it does not exist.
     * If it exists, depending on the TxtExportType, the file will be replaced, appended or
     * a new file with a unique name will be created.
     * @param data the vector to be exported
     * @param path data will be exported to this path
     * @param filename the name of the file
     * @param type the export type (REPLACE, APPEND, NEW)
     * @param print if true, the function will print some information
     */
    static void exportVectorToTxtFile(const std::vector<double>& data, const std::string& path, const std::string& filename, TxtExportType type = TxtExportType::REPLACE, bool print = false);

    /**
     * Returns true if the file exists.
     * @param filePath the path to the file
     * @return true if the file exists
     */
    static bool fileExists(const std::string& filePath);

    /**
     * Returns a unique filename. If the file already exists, a number will be appended to the filename.
     * @param path the path to the file
     * @param filename the filename
     * @return a unique filename (e.g. filename_1)
     */
    static std::string getUniqueFileName(const std::string& path, const std::string& filename);

    /**
     * Returns the path to the folder where the data should be exported to.
     * If the folder does not exist, it will be created.
     * The main export folder is defined in the config file. The string "folder" is the name of the subfolder.
     */
    static std::string getExportPath(const std::string &folder);

    /**
     * Returns the current RAM usage in MB.
     * @return  RAM usage in MB as double
     */
    static double getRamUsageMB();

    /**
     * Returns a std::vector<double> from a VectorXd.
     * @param v  VectorXd
     * @return  std::vector<double>
     */
    static std::vector<double> vectorXdToStdVector(const Vector& v);

    /**
     * Appends a Matrix to another Matrix row-wise.
     * If checkNumCols is true, the function will check if the number of columns is the same and return false that is not the case.
     * If checkNumCols is false, the function will append the Matrix without checking the number of columns, leaving undefined values in the new columns.
     * @param X Matrix to append to (will be modified)
     * @param Y Matrix to append (will not be modified)
     * @param checkNumCols if true, the function will check if the number of columns is the same and return false if that is not the case
     * @return true if the Matrix was appended successfully
     */
    static bool appendRowWise(Matrix &X, const Matrix &Y, bool checkNumCols = true);

    /**
     * Appends a Matrix to another Matrix column-wise.
     * If checkNumRows is true, the function will check if the number of rows is the same and return false that is not the case.
     * If checkNumRows is false, the function will append the Matrix without checking the number of rows, leaving undefined values in the new rows.
     * @param X Matrix to append to (will be modified)
     * @param Y Matrix to append (will not be modified)
     * @param checkNumRows if true, the function will check if the number of rows is the same and return false if that is not the case
     * @return true if the Matrix was appended successfully
     */
    static bool appendColWise(Matrix &X, const Matrix &Y, bool checkNumRows = true);

    /**
     * Appends a Vector to another Vector.
     * @param v1 Vector to append to (will be modified)
     * @param v2 Vector to append (will not be modified)
     */
    static void appendToVector(Vector &v1, const Vector &v2);

    /**
     * Appends a double to a Vector.
     * @param v1 Vector to append to (will be modified)
     * @param value double to append
     */
    static void appendToVector(Vector &v1, double value);

    /**
     * Removes a row from a Matrix.
     * @param matrix Matrix to remove row from
     * @param rowToRemove row to remove
     */
    static void removeRow(Matrix& matrix, unsigned int rowToRemove);

    /**
     * Removes a column from a Matrix.
     * @param matrix Matrix to remove column from
     * @param colToRemove column to remove
     */
    static void removeColumn(Matrix& matrix, unsigned int colToRemove);
private:
    Util() = default;
};

#endif //RADIATION_MAPPING_UTIL_H