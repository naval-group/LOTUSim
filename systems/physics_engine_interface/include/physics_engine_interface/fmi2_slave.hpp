/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_PHYSICS_ENGINE_INTERFACE_FMI2_SLAVE_HH_
#define LOTUSIM_PHYSICS_ENGINE_INTERFACE_FMI2_SLAVE_HH_

#include <filesystem>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include "fmi2FunctionTypes.h"
#include "lotusim_common/logger.hpp"

namespace lotusim::gazebo {

/**
 * @brief Raised for anything that keeps an FMU from loading or running: a
 * missing or malformed archive, a model that is not FMI 2.0 Co-Simulation,
 * an unknown variable, an fmi2 call that does not return fmi2OK.
 */
class FmuError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/**
 * @brief One instance of an FMI 2.0 Co-Simulation FMU.
 *
 * The .fmu archive is unpacked into a directory of its own, removed with
 * the instance, so that two instances of the same FMU load two copies of
 * its library and share no global state. Variables are addressed by name.
 *
 * Sequence: construct, set parameters, initialize(), then setX / doStep /
 * getX each step. Not thread-safe: one instance is stepped by one thread at
 * a time.
 */
class Fmi2Slave {
public:
    enum class Type
    {
        Real,
        Integer,
        Boolean,
        String
    };

    struct Variable {
        fmi2ValueReference value_reference;
        Type type;
        std::string causality;    ///< "input", "output", "parameter", ...
        std::string description;  ///< empty when modelDescription.xml has none
        std::map<std::string, int> items;  ///< an Enumeration's, by name
    };

    /**
     * @brief Unpack and load an FMU, and instantiate it.
     *
     * @param fmu_path an .fmu archive
     * @param instance_name name the FMU uses in its log messages
     * @param logger where the FMU's own log messages go
     * @throws FmuError
     */
    Fmi2Slave(
        const std::string& fmu_path,
        const std::string& instance_name,
        std::shared_ptr<spdlog::logger> logger);

    ~Fmi2Slave();

    Fmi2Slave(const Fmi2Slave&) = delete;
    Fmi2Slave& operator=(const Fmi2Slave&) = delete;

    const std::string& modelName() const
    {
        return m_model_name;
    }

    bool hasVariable(const std::string& name) const;
    const Variable& variable(const std::string& name) const;
    const std::map<std::string, Variable>& variables() const
    {
        return m_variables;
    }

    /** @brief Setup, enter and exit initialization mode, at start_time. */
    void initialize(double start_time);
    bool initialized() const
    {
        return m_initialized;
    }

    void setReal(const std::string& name, double value);
    void setInteger(const std::string& name, int value);
    void setBoolean(const std::string& name, bool value);
    void setString(const std::string& name, const std::string& value);
    /**
     * @brief Set a variable of any type from text ("0.2", "3", "true"), an
     * Enumeration by item name or value ("rk4").
     */
    void setFromString(const std::string& name, const std::string& value);

    double getReal(const std::string& name);

    /** @brief Advance from t by h. */
    void doStep(double t, double h);

private:
    void check(int status, const char* call) const;
    const Variable& expect(const std::string& name, Type type) const;
    void unpack(const std::string& fmu_path);
    void readModelDescription();
    void load();

    std::shared_ptr<spdlog::logger> m_logger;
    std::string m_instance_name;
    std::filesystem::path m_dir;  ///< the unpacked FMU, removed with us
    std::string m_model_name;
    std::string m_model_identifier;
    std::string m_guid;
    std::map<std::string, Variable> m_variables;

    void* m_library = nullptr;
    struct Functions;
    std::unique_ptr<Functions> m_fn;
    fmi2CallbackFunctions m_callbacks{};
    fmi2Component m_component = nullptr;
    bool m_initialized = false;
};

}  // namespace lotusim::gazebo

#endif
