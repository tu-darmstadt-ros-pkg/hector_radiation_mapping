#ifndef RADIATION_MAPPING_DDDYNAMIC_RECONFIGURE_H
#define RADIATION_MAPPING_DDDYNAMIC_RECONFIGURE_H

#include <boost/variant.hpp>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include "parameters.h"

/**
 * @brief The DDDynamicReconfigure class. (Singleton pattern)
 * This class is a wrapper for the ddynamic_reconfigure package and used to allow for new parameters to be added at runtime.
 * This is done by saving all registered variables and groups and resetting the parameter server when a new variable is registered.
 * It allows to register variables and groups and publish them to the parameter server.
 * The variables can be registered with a callback function, which is called when the variable is changed.
 */
class DDDynamicReconfigure {
public:
    static DDDynamicReconfigure& instance();

    /**
     * registerVariable register a variable to be modified via the dynamic_reconfigure API. When a change is made, it will be reflected in the variable directly
     * Strings mustn't have spaces in them.
     * @tparam T type of the variable
     * @param name name of the variable
     * @param variable pointer to the variable
     * @param description  description of the variable
     * @param min minimum value of the variable
     * @param max maximum value of the variable
     * @param group group of the variable
     */
    template <typename T>
    void registerVariable(const std::string &name, T *variable,
                          const std::string &description = "", T min = getMin<T>(),
                          T max = getMax<T>(), const std::string &group = "Default"){
        registerVariable(name, variable, {}, description, min, max, group);
    }

    /**
     * registerVariable register a variable to be modified via the dynamic_reconfigure API. When a change is made, it will be reflected in the variable directly
     * Strings mustn't have spaces in them.
     * @tparam T type of the variable
     * @param name name of the variable
     * @param variable pointer to the variable
     * @param callback callback function that is called when the variable is changed
     * @param description description of the variable
     * @param min minimum value of the variable
     * @param max maximum value of the variable
     * @param group group of the variable
     */
    template <typename T>
    void registerVariable(const std::string &name, T *variable,
                          const boost::function<void(T value)> &callback,
                          const std::string &description = "", T min = getMin<T>(), T max = getMax<T>(),
                          const std::string &group = "Default"){
        vars_.emplace_back(RecVariable<T>(name, variable, callback, description, min, max, group));
    }

    /**
     * registerVariable register a variable to be modified via the dynamic_reconfigure API. When a change is made, it will be reflected in the variable directly
     * Strings mustn't have spaces in them.
     * @tparam T type of the variable
     * @param name name of the variable
     * @param current_value current value of the variable
     * @param callback callback function that is called when the variable is changed
     * @param description description of the variable
     * @param min minimum value of the variable
     * @param max maximum value of the variable
     * @param group group of the variable
     */
    template <typename T>
    void registerVariable(const std::string &name, T current_value,
                                 const boost::function<void(T value)> &callback,
                                 const std::string &description = "", T min = getMin<T>(), T max = getMax<T>(),
                                 const std::string &group = "Default"){
        vars_.emplace_back(RecVariable<T>(name, current_value, callback, description, min, max, group));
    }

    /**
     * Deletes a variable from the parameter server and from the list of registered variables and groups.
     * @param name name of the variable
     */
    void deleteVariable(const std::string &name);

    /**
     * Deletes a group from the parameter server and from the list of registered variables and groups.
     * @param group name of the group
     */
    void deleteGroup(const std::string &group);

    /**
     * Deletes all variables and groups from the parameter server and from the list of registered variables and groups.
     */
    void reset();

    /**
     * Publishes all registered variables and groups to the parameter server.
     */
    void publish();
private:

    DDDynamicReconfigure();
    DDDynamicReconfigure(const DDDynamicReconfigure&) = delete;
    DDDynamicReconfigure& operator=(const DDDynamicReconfigure&) = delete;

    template <typename T>
    struct RecVariable{
    public:
        RecVariable(const std::string &name, T variable,
                      const boost::function<void(T value)> &callback,
                      const std::string &description = "", T min = getMin<T>(), T max = getMax<T>(),
                      const std::string &group = "Default"){
            this->name_ = name;
            this->variable_ = variable;
            this->callback_ = callback;
            this->description_ = description;
            this->min_ = min;
            this->max_ = max;
            this->group_ = group;
        }

        RecVariable(const std::string &name, T *variable,
                    const boost::function<void(T value)> &callback,
                    const std::string &description = "", T min = getMin<T>(), T max = getMax<T>(),
                    const std::string &group = "Default"){
            this->name_ = name;
            this->variable_ = variable;
            this->callback_ = callback;
            this->description_ = description;
            this->min_ = min;
            this->max_ = max;
            this->group_ = group;
        }
        std::string name_;
        boost::variant<T, T*> variable_;
        boost::function<void(T value)> callback_;
        std::string description_;
        T min_;
        T max_;
        std::string group_;
    };

    std::vector<boost::variant<RecVariable<bool>, RecVariable<int>, RecVariable<double>, RecVariable<std::string>>> vars_;
    std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> DDR_;
};

#endif //RADIATION_MAPPING_DDDYNAMIC_RECONFIGURE_H