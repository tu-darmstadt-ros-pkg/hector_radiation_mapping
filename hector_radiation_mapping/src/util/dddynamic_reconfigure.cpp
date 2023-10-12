#include "util/dddynamic_reconfigure.h"

DDDynamicReconfigure::DDDynamicReconfigure(){
    DDR_ = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(*Parameters::instance().nodeHandle_);
}

DDDynamicReconfigure &DDDynamicReconfigure::instance() {
    static DDDynamicReconfigure instance;
    return instance;
}

void DDDynamicReconfigure::publish() {
    auto lambda = [this]<typename T>(RecVariable<T> &t) {
        t.variable_.which() == 0 ?
        DDR_->registerVariable<T>(t.name_, boost::get<T>(t.variable_), t.callback_, t.description_, t.min_, t.max_,
                                  t.group_) :
        DDR_->registerVariable<T>(t.name_, boost::get<T *>(t.variable_), t.callback_, t.description_, t.min_, t.max_,
                                  t.group_);
    };

    // reset parameter publisher (clears all variables that where published there from this radiationMappingNode)
    DDR_.reset(new ddynamic_reconfigure::DDynamicReconfigure(*Parameters::instance().nodeHandle_));

    // iterate over all variables stored in this class and publish them.
    for (boost::variant<RecVariable<bool>, RecVariable<int>, RecVariable<double>, RecVariable<std::string>> &var: vars_) {
        switch (var.which()) {
            case 0:
                lambda(boost::get<RecVariable<bool>>(var));
                break;
            case 1:
                lambda(boost::get<RecVariable<int>>(var));
                break;
            case 2:
                lambda(boost::get<RecVariable<double>>(var));
                break;
            case 3:
                lambda(boost::get<RecVariable<std::string>>(var));
                break;
        }
    }
    // finally start the parameter sever again.
    // after starting it, there can be no changes made to it, without resetting first.
    DDR_->publishServicesTopics();
}

void DDDynamicReconfigure::deleteVariable(const std::string &name) {
    auto it = vars_.begin();
    while (it != vars_.end()) {
        boost::variant<RecVariable<bool>, RecVariable<int>, RecVariable<double>, RecVariable<std::string>> &var = *it;
        switch (var.which()) {
            case 0: {
                RecVariable<bool> cast = boost::get<RecVariable<bool>>(var);
                if (cast.name_ == name) {
                    vars_.erase(it);
                }
                break;
            }
            case 1: {
                RecVariable<int> cast = boost::get<RecVariable<int>>(var);
                if (cast.name_ == name) {
                    vars_.erase(it);
                }
                break;
            }
            case 2: {
                RecVariable<double> cast = boost::get<RecVariable<double>>(var);
                if (cast.name_ == name) {
                    vars_.erase(it);
                }
                break;
            }
            case 3: {
                RecVariable<std::string> cast = boost::get<RecVariable<std::string>>(var);
                if (cast.name_ == name) {
                    vars_.erase(it);
                }
                break;
            }
        }
        it++;
    }
}

void DDDynamicReconfigure::deleteGroup(const std::string &group) {
    auto it = vars_.begin();
    while (it != vars_.end()) {
        boost::variant<RecVariable<bool>, RecVariable<int>, RecVariable<double>, RecVariable<std::string>> &var = *it;
        switch (var.which()) {
            case 0: {
                RecVariable<bool> cast = boost::get<RecVariable<bool>>(var);
                if (cast.group_ == group) {
                    it = vars_.erase(it);
                }
                break;
            }
            case 1: {
                RecVariable<int> cast = boost::get<RecVariable<int>>(var);
                if (cast.group_ == group) {
                    it = vars_.erase(it);
                }
                break;
            }
            case 2: {
                RecVariable<double> cast = boost::get<RecVariable<double>>(var);
                if (cast.group_ == group) {
                    it = vars_.erase(it);
                }
                break;
            }
            case 3: {
                RecVariable<std::string> cast = boost::get<RecVariable<std::string>>(var);
                if (cast.group_ == group) {
                    it = vars_.erase(it);
                }
                break;
            }
        }
        it++;
    }
}

void DDDynamicReconfigure::reset() {
    DDR_.reset(new ddynamic_reconfigure::DDynamicReconfigure(*Parameters::instance().nodeHandle_));
    vars_.clear();
}