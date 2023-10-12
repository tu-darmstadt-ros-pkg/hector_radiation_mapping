#ifndef RADIATION_MAPPING_MARKER_H
#define RADIATION_MAPPING_MARKER_H

/**
 * @brief The Marker class is a wrapper for the visualization_msgs::Marker class. It provides methods to set the color,
 * orientation, position and scale of the marker. The marker is published if the publish parameter is true.
 * The marker is deleted from the marker manager if the deleteMarker method is called.
 */
class Marker {
public:
    /**
     * Set the Color of the marker. The color is published if publish is true.
     * @param r red value between 0 and 1
     * @param g green value between 0 and 1
     * @param b blue value between 0 and 1
     * @param a alpha value between 0 and 1
     * @param publish publish the marker if true
     */
    void setColor(double r, double g, double b, double a = 1.0, bool publish = true);

    /**
     * Set the Color of the marker. The color is published if publish is true.
     * @param color vector with red, green, blue and alpha value between 0 and 1
     * @param publish publish the marker if true
     */
    void setColor(const Eigen::Vector4d &color, bool publish = true);

    /**
     * Set the orientation of the marker. The orientation is published if publish is true.
     * @param x x value of quaternion, default is 0
     * @param y y value of quaternion, default is 0
     * @param z z value of quaternion, default is 0
     * @param w w value of quaternion, default is 1
     * @param publish publish the marker if true
     */
    void setOrientation(double x = 0.0, double y = 0.0, double z = 0.0, double w = 1.0, bool publish = true);

    /**
     * Set the orientation of the marker. The orientation is published if publish is true.
     * @param orientation vector with x, y, z and w value of quaternion
     * @param publish publish the marker if true
     */
    void setOrientation(Eigen::Vector4d orientation, bool publish = true);

    /**
     * Set the position of the marker. The position is published if publish is true.
     * @param pos vector with x, y and z value of position
     * @param publish publish the marker if true
     */
    virtual void setPos(const Eigen::Vector3d &pos, bool publish);

    /**
     * Set the position of the marker. The position is published if publish is true.
     * @param x x value of position
     * @param y y value of position
     * @param z z value of position
     * @param publish publish the marker if true
     */
    virtual void setPos(double x, double y, double z, bool publish);

    /**
     * Set the scale of the marker. The scale is published if publish is true.
     * @param x x value of scale, default is 0.1
     * @param y y value of scale, default is 0.1
     * @param z z value of scale, default is 0.1
     * @param publish publish the marker if true
     */
    void setScale(double x = 0.1, double y = 0.1, double z = 0.1, bool publish = true);

    /**
     * Set the scale of the marker. The scale is published if publish is true.
     * @param scale vector with x, y and z value of scale
     * @param publish publish the marker if true
     */
    void setScale(const Eigen::Vector3d &scale, bool publish = true);

    /**
     * Delete this marker from the marker manager.
     */
    virtual void deleteMarker();

protected:
    Marker();

    int id_;
    inline static int idCounter_ = 0;
    visualization_msgs::Marker marker_;
};

class SphereMarker : public Marker {
public:
    explicit SphereMarker(const Eigen::Vector3d &origin = {0, 0, 0}, const Eigen::Vector4d &color = {0.0, 1.0, 0.0, 1.0});
};

class TextMarker : public Marker {
public:
    explicit TextMarker(const Eigen::Vector3d &origin = {0.0, 0.0, 0.0},
               const std::string &text = "default",
               double zOffset = 0.2,
               const Eigen::Vector4d &color = {1.0, 1.0, 1.0, 1.0});

    /**
     * Set the position of the marker. The position is published if publish is true. Also sets the position of the
     * sphere marker.
     * @param pos vector with x, y and z value of position
     * @param publish publish the marker if true
     */
    void setPos(const Eigen::Vector3d &pos, bool publish) override;

    /**
     * Set the position of the marker. The position is published if publish is true. Also sets the position of the
     * sphere marker.
     * @param x x value of position
     * @param y y value of position
     * @param z z value of position
     * @param publish publish the marker if true
     */
    void setPos(double x, double y, double z, bool publish) override;

    /**
     * Set the z offset of the marker. Vertical distance of text to position.
     * @param zOffset The z offset of the marker.
     */
    void setzOffset(double zOffset);

    /**
     * Set the text of the marker.
     * @param text The text of the marker.
     */
    void setText(const std::string &text);

    /**
     * Get the text of the marker.
     * @return The text of the marker.
     */
    std::string getText();

    /**
     * Delete this marker from the marker manager.
     */
    void deleteMarker() override;

private:
    double zOffset_;
    SphereMarker sphere;
};

class TrajectoryMarker : public Marker {
public:
    explicit TrajectoryMarker(const Eigen::Vector3d &origin = {0.0, 0.0, 0.0},
                     const Eigen::Vector4d &color = {1.0, 1.0, 0.0, 1.0});

    /**
     * Add points to the trajectory.
     * @param newValues radiation values of new points as vector
     * @param newPositions positions of new points as matrix with columns x, y and z
     */
    void addPoints(const Vector &newValues, const Eigen::Matrix3Xd &newPositions);

    /**
     * Add a point to the trajectory.
     * @param value radiation value of new point
     * @param position position of new point as vector with x, y and z
     */
    void addPoint(const double &value, const Eigen::Vector3d &position);

private:
    Vector values_;
};

#endif //RADIATION_MAPPING_MARKER_H