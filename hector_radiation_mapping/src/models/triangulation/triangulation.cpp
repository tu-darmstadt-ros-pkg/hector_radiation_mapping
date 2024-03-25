#include "models/triangulation/triangulation.h"
#include "util/util.h"
#include "util/parameters.h"
#include "util/clock.h"
#include "util/print.h"
#include "util/dddynamic_reconfigure.h"
#include "hector_radiation_mapping/sampleManager.h"
#include "gsl/gsl_multimin.h"

Triangulation::Triangulation() : Model(ModelType::TRIANGULATION, Parameters::instance().tr_on_start_up, Parameters::instance().tr_min_update_time) {
    layerNameVal_ = "valueMap";
    layerNameErr_ = "errorMap";
    grid_map_ = std::make_shared<GridMap>(Parameters::instance().tr_grid_map_topic,
                                          Parameters::instance().tr_grid_map_resolution);
    grid_map_->addLayer(layerNameVal_);
    grid_map_->addLayer(layerNameErr_);

    useCircle_ = true;
    zLevel2D_ = 0.0;
    zLevel2Dtemp_ = 0.0;
    useExpensiveErrorMap_ = true;

    slam_map_subscriber_ = std::make_shared<ros::Subscriber>(
            Parameters::instance().node_handle_ptr_->subscribe<nav_msgs::OccupancyGrid>(
                    Parameters::instance().environment_map_topic, 1, &Triangulation::slamMapCallback, this));

    /// set up dddynamicReconfigure
    /*
    DDDynamicReconfigure::instance().registerVariable<double>(getShortModelName() + "_2d_map_z", 0.0,
                                                              boost::bind(&Triangulation::zLevelCallback, this, _1),
                                                              "RPModel_z", 0.0, 10.0, getShortModelName());*/
    DDDynamicReconfigure::instance().registerVariable<bool>(getShortModelName() + "_eval_error_map", false,
                                                            boost::bind(&Triangulation::useErrorMap, this, _1), "on/off",
                                                            false, true, getShortModelName());

    /*
    Clock clock;
    clock.tick();
    double A1;
    double A2;
    double x1;
    double x2;
    double y1;
    double y2;
    bool one = Test::calculateA12(A1, A2, x1, y1, x2, y2, 10, 12, 9, 2, 5, 4, 8, 5, 9);
    ROS_INFO_STREAM(one  << " A1: " << A1 << " A2: " << A2 << " time ms: " << clock.tock());
    bool two = Test::calculateA12(A1, A2,  x1, y1, x2, y2, 1.9, 1.9, 1.0, -1.9, 0.9, 1.0, 0.7, 1.2, 1.0);
    ROS_INFO_STREAM(two << " A1: " << A1 << " A2: " << A2 << " time ms: " << clock.tock());
    */
}

void Triangulation::reset() {

}

void Triangulation::update() {
    {
        std::unique_lock<std::mutex> lock{sample_queue_mtx_};
        update_condition_.wait(lock, [this]() { return (!samples_add_queue_.empty()) || !active_; });
        if (!active_) return;
    }
    updateSamples();

    // STUFF
    Clock clock;
    ROS_INFO_STREAM("RP updateModel start");
    if (samples_.size() > 1) {
        clock.tick();
        updateSourcePrediction();
        ROS_INFO_STREAM("RP ModelUpdate time(ms): " << clock.tock());
        clock.tick();
        update2DView();
        ROS_INFO_STREAM("RP ModelView time(ms): " << clock.tock());
    }

    grid_map_->publish();
}

void Triangulation::updateSourcePrediction() {
    Eigen::Vector3d t = samples_.back().sensor_position_;
    Eigen::Vector3d minPos = nelderMeadMinimization(t);
    ROS_INFO_STREAM("neldermead completed");
    Source source(minPos, calculateSourceValue(minPos), calculateSourceValue(minPos));
    sources_.push_back(source);
    ROS_INFO_STREAM("source strength" << calculateSourceValue(minPos));
    if (sources_.size() > 1) {
        //sources_.at(0).marker_->deleteMarker();
        sources_.erase(sources_.begin());
    }
}

void Triangulation::update2DView() {
    Eigen::MatrixX2d samplePositions;
    Eigen::Vector2d center;
    double radius = 5.0;
    bool useCircle = useCircle_;
    std::shared_ptr<nav_msgs::OccupancyGrid> slam_map = slam_map_;
    std::lock_guard<std::mutex> lock{grid_map_->getGridMapMutex()};
    grid_map_->updateGridDimensionWithSlamMap(slam_map);
    if (useCircle) {
        center = SampleManager::instance().getLastSamplePos().topRows(2);
        samplePositions = grid_map_->getCircleSamplePositions(center, radius);
    } else {
        samplePositions = grid_map_->getMapSamplePositions();

    }
    if (useCircle) {
        grid_map_->updateLayer(layerNameVal_, sample2DMapValue(samplePositions), center, radius);
        if (useExpensiveErrorMap_) {
            grid_map_->updateLayer(layerNameErr_, sample2DMapError(samplePositions), center, radius);
        }
    } else {
        grid_map_->updateLayer(layerNameVal_, sample2DMapValue(samplePositions));
        if (useExpensiveErrorMap_) {
            grid_map_->updateLayer(layerNameErr_, sample2DMapError(samplePositions));
        }
    }
}

double errorSum(const gsl_vector *v, void *params) {
    auto *samples = (std::vector<Sample> *) params;
    Eigen::Vector3d pos(gsl_vector_get(v, 0), gsl_vector_get(v, 1), gsl_vector_get(v, 2));
    double errorSum = 0;
    int n = 0;
    for (int x = 0; x < samples->size() - 1; x++) {
        for (int y = x + 1; y < samples->size(); y++) {
            errorSum += Triangulation::error(samples->at(x), samples->at(y), pos);
            n++;
        }
    }
    return errorSum / n;
}

Eigen::Vector3d Triangulation::nelderMeadMinimization(Eigen::Vector3d startPos) {
    const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex2;
    gsl_multimin_fminimizer *s = nullptr;
    gsl_vector *ss, *x;
    gsl_multimin_function minex_func;
    size_t iter = 0;
    int status;
    double size;
    /* Starting point */
    x = gsl_vector_alloc(3);
    gsl_vector_set(x, 0, startPos.x());
    gsl_vector_set(x, 1, startPos.y());
    gsl_vector_set(x, 2, startPos.z());
    /* Set initial step sizes to 1 */
    ss = gsl_vector_alloc(3);
    gsl_vector_set_all(ss, 1.0);
    gsl_vector_set(ss, 2, -1.0);
    std::vector<Sample> samples = SampleManager::instance().getSamplesWithinRadius(startPos, 5.0);
    /* Initialize method and iterate */
    minex_func.n = 3;
    minex_func.f = errorSum;
    minex_func.params = &samples;
    s = gsl_multimin_fminimizer_alloc(T, 3);
    gsl_multimin_fminimizer_set(s, &minex_func, x, ss);
    do {
        iter++;
        status = gsl_multimin_fminimizer_iterate(s);
        if (status)
            break;

        size = gsl_multimin_fminimizer_size(s);
        status = gsl_multimin_test_size(size, 1e-2);
        /*
        if(false){
        if (status == GSL_SUCCESS) {
            printf ("converged to minimum at\n");
        }
            printf ("%5d %10.3e %10.3e %10.3e f() = %7.3f size = %.3f\n",
                    iter,
                    gsl_vector_get (s->x, 0),
                    gsl_vector_get (s->x, 1),
                    gsl_vector_get (s->x, 2),
                    s->fval, size);
        }*/
    } while (status == GSL_CONTINUE && iter < 100);
    Eigen::Vector3d result;
    result.x() = gsl_vector_get(s->x, 0);
    result.y() = gsl_vector_get(s->x, 1);
    result.z() = gsl_vector_get(s->x, 2);
    gsl_vector_free(x);
    gsl_vector_free(ss);
    gsl_multimin_fminimizer_free(s);
    return result;
}

double Triangulation::error(const Sample &sample1, const Sample &sample2, const Eigen::Vector3d &posSource) {
    double val1 = Parameters::instance().use_dose_rate ? sample1.dose_rate_ : sample1.cps_;
    double val2 = Parameters::instance().use_dose_rate ? sample2.dose_rate_ : sample2.cps_;
    return pow(
            val1 * (sample1.sensor_position_ - posSource).squaredNorm() - val2 * (sample2.sensor_position_ - posSource).squaredNorm(),
            2);
}

double Triangulation::calculateWeight() {
    return 1.0;
}

Vector Triangulation::sample2DMapError(const Eigen::MatrixX2d &positions) {
    /// evaluate Error Function on every map position. Extremely costly atm
    Vector errorSum = Eigen::VectorXd::Zero(positions.rows());
    int n = 0;
    for (int x = 0; x < samples_.size() - 1; x++) {
        for (int y = x + 1; y < samples_.size(); y++) {
            Sample i1 = samples_[x];
            Sample i2 = samples_[y];
            for (int i = 0; i < positions.rows(); ++i) {
                Eigen::Vector3d pos;
                pos << positions(i, 0), positions(i, 1), zLevel2D_;
                errorSum[i] += error(i1, i2, pos);
            }
            n++;
        }
    }
    return errorSum / n;
}

Vector Triangulation::sample2DMapValue(const Eigen::MatrixX2d &positions) {
    Vector values = Eigen::VectorXd::Zero(positions.rows());
    for (int i = 0; i < positions.rows(); ++i) {
        Eigen::Vector3d pos;
        pos << positions(i, 0), positions(i, 1), zLevel2D_;
        values[i] = calculateIntensity(pos);
    }
    return values;
}

double Triangulation::calculateIntensity(const Eigen::Vector3d &position) {
    double totalIntensity = 0.0;
    for (Source &source: sources_) {

        totalIntensity += calculateIntensity(position, source);
    }
    return totalIntensity;
}

double Triangulation::calculateIntensity(const Eigen::Vector3d &position, const Source &source) {
    double ds = (source.getPos() - position).squaredNorm();
    ds = std::max(0.1, ds); /// Under 10cm the radiation strength just gets approximated constant.
    return (source.getStrength()) / ds;
}

double Triangulation::calculateSourceValue(const Eigen::Vector3d &position) {
    /// use the value of the 10 closest samples and take the average of the "would be values at source"
    std::vector<std::tuple<double, double>> listSDistVal; /// squared distance, Value

    listSDistVal.reserve(samples_.size());
    for (Sample &sample: samples_) {
        listSDistVal.emplace_back((sample.sensor_position_ - position).squaredNorm(),
                                  (Parameters::instance().use_dose_rate ? sample.dose_rate_ : sample.cps_));
    }
    auto sortByDistance = [](const std::tuple<double, double> &i1, const std::tuple<double, double> &i2) {
        return (std::get<0>(i1) < std::get<0>(i2));
    };
    std::sort(listSDistVal.begin(), listSDistVal.end(), sortByDistance);
    int n = 0;
    double value = 0.0;
    for (std::tuple<double, double> &sDistVal: listSDistVal) {
        value += std::get<1>(sDistVal) / std::get<0>(sDistVal);
        n++;
        if (n >= 10) break;
    }
    return n == 0 ? 0 : value / (double) n;
}

void Triangulation::slamMapCallback(const nav_msgs::OccupancyGrid_<std::allocator<void>>::ConstPtr &grid_msg_ptr) {
    slam_map_ = std::make_shared<nav_msgs::OccupancyGrid>(*grid_msg_ptr);
}

void Triangulation::useErrorMap(bool use) {
    useExpensiveErrorMap_ = use;
}
