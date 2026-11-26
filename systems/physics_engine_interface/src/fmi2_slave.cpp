/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "physics_engine_interface/fmi2_slave.hpp"

#include <dlfcn.h>
#include <tinyxml2.h>
#include <zip.h>

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <vector>

namespace lotusim::gazebo {

struct Fmi2Slave::Functions {
    fmi2InstantiateTYPE* instantiate;
    fmi2FreeInstanceTYPE* freeInstance;
    fmi2SetupExperimentTYPE* setupExperiment;
    fmi2EnterInitializationModeTYPE* enterInitializationMode;
    fmi2ExitInitializationModeTYPE* exitInitializationMode;
    fmi2TerminateTYPE* terminate;
    fmi2SetRealTYPE* setReal;
    fmi2SetIntegerTYPE* setInteger;
    fmi2SetBooleanTYPE* setBoolean;
    fmi2SetStringTYPE* setString;
    fmi2GetRealTYPE* getReal;
    fmi2DoStepTYPE* doStep;
};

namespace {

const char* statusName(int status)
{
    switch (status) {
        case fmi2OK:
            return "OK";
        case fmi2Warning:
            return "Warning";
        case fmi2Discard:
            return "Discard";
        case fmi2Error:
            return "Error";
        case fmi2Fatal:
            return "Fatal";
        case fmi2Pending:
            return "Pending";
        default:
            return "unknown status";
    }
}

// The FMU's messages, to the logger of the instance that owns it.
void logMessage(
    fmi2ComponentEnvironment environment,
    fmi2String instance,
    fmi2Status status,
    fmi2String category,
    fmi2String message,
    ...)
{
    auto* logger = static_cast<spdlog::logger*>(environment);
    if (!logger || !message) {
        return;
    }
    va_list args;
    va_start(args, message);
    va_list copy;
    va_copy(copy, args);
    const int size = std::vsnprintf(nullptr, 0, message, copy);
    va_end(copy);
    std::string text(size > 0 ? size : 0, '\0');
    if (size > 0) {
        std::vsnprintf(text.data(), text.size() + 1, message, args);
    }
    va_end(args);
    const std::string line = fmt::format(
        "FMU {} [{}]: {}",
        instance ? instance : "",
        category ? category : "",
        text);
    if (status == fmi2OK) {
        logger->debug(line);
    } else if (status == fmi2Warning || status == fmi2Discard) {
        logger->warn(line);
    } else {
        logger->error(line);
    }
}

void stepFinished(fmi2ComponentEnvironment, fmi2Status) {}

}  // namespace

Fmi2Slave::Fmi2Slave(
    const std::string& fmu_path,
    const std::string& instance_name,
    std::shared_ptr<spdlog::logger> logger)
    : m_logger(std::move(logger))
    , m_instance_name(instance_name)
    , m_fn(new Functions{})
{
    try {
        unpack(fmu_path);
        readModelDescription();
        load();
    } catch (...) {
        if (m_library) {
            dlclose(m_library);
        }
        if (!m_dir.empty()) {
            std::error_code ignored;
            std::filesystem::remove_all(m_dir, ignored);
        }
        throw;
    }
}

Fmi2Slave::~Fmi2Slave()
{
    if (m_component) {
        if (m_initialized) {
            m_fn->terminate(m_component);
        }
        m_fn->freeInstance(m_component);
    }
    if (m_library) {
        dlclose(m_library);
    }
    if (!m_dir.empty()) {
        std::error_code ignored;
        std::filesystem::remove_all(m_dir, ignored);
    }
}

void Fmi2Slave::unpack(const std::string& fmu_path)
{
    int error = 0;
    zip_t* archive = zip_open(fmu_path.c_str(), ZIP_RDONLY, &error);
    if (!archive) {
        throw FmuError("cannot open FMU '" + fmu_path + "'");
    }
    std::string pattern =
        (std::filesystem::temp_directory_path() / "lotusim-fmu-XXXXXX")
            .string();
    if (!mkdtemp(pattern.data())) {
        zip_close(archive);
        throw FmuError(
            "cannot create a directory to unpack '" + fmu_path + "'");
    }
    m_dir = pattern;

    const zip_int64_t entries = zip_get_num_entries(archive, 0);
    std::vector<char> buffer(1 << 16);
    for (zip_int64_t i = 0; i < entries; ++i) {
        const std::string name = zip_get_name(archive, i, 0);
        const std::filesystem::path target = (m_dir / name).lexically_normal();
        // No entry may land outside the directory ("../", absolute paths).
        const std::string relative = target.lexically_relative(m_dir).string();
        if (relative.empty() || relative.rfind("..", 0) == 0) {
            zip_close(archive);
            throw FmuError(
                "FMU '" + fmu_path + "' has an unsafe entry '" + name + "'");
        }
        if (!name.empty() && name.back() == '/') {
            std::filesystem::create_directories(target);
            continue;
        }
        std::filesystem::create_directories(target.parent_path());
        zip_file_t* in = zip_fopen_index(archive, i, 0);
        std::ofstream out(target, std::ios::binary);
        if (!in || !out) {
            if (in) {
                zip_fclose(in);
            }
            zip_close(archive);
            throw FmuError(
                "cannot unpack '" + name + "' from '" + fmu_path + "'");
        }
        zip_int64_t n;
        while ((n = zip_fread(in, buffer.data(), buffer.size())) > 0) {
            out.write(buffer.data(), n);
        }
        zip_fclose(in);
        if (n < 0 || !out) {
            zip_close(archive);
            throw FmuError(
                "cannot unpack '" + name + "' from '" + fmu_path + "'");
        }
    }
    zip_close(archive);
}

void Fmi2Slave::readModelDescription()
{
    const std::string path = (m_dir / "modelDescription.xml").string();
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        throw FmuError("cannot read " + path + ": " + doc.ErrorStr());
    }
    const tinyxml2::XMLElement* root =
        doc.FirstChildElement("fmiModelDescription");
    if (!root) {
        throw FmuError(path + " has no fmiModelDescription");
    }
    const char* version = root->Attribute("fmiVersion");
    if (!version || std::string(version) != "2.0") {
        throw FmuError(
            "FMU '" +
            std::string(
                root->Attribute("modelName") ? root->Attribute("modelName")
                                             : "") +
            "' is FMI " + (version ? version : "?") +
            ", only 2.0 is supported");
    }
    m_model_name =
        root->Attribute("modelName") ? root->Attribute("modelName") : "";
    m_guid = root->Attribute("guid") ? root->Attribute("guid") : "";
    const tinyxml2::XMLElement* cs = root->FirstChildElement("CoSimulation");
    if (!cs || !cs->Attribute("modelIdentifier")) {
        throw FmuError(
            "FMU '" + m_model_name + "' has no Co-Simulation interface");
    }
    m_model_identifier = cs->Attribute("modelIdentifier");

    // Enumeration items by type name, so that a parameter is set by name.
    std::map<std::string, std::map<std::string, int>> enumerations;
    const tinyxml2::XMLElement* types =
        root->FirstChildElement("TypeDefinitions");
    for (const tinyxml2::XMLElement* t =
             types ? types->FirstChildElement("SimpleType") : nullptr;
         t;
         t = t->NextSiblingElement("SimpleType")) {
        const tinyxml2::XMLElement* e = t->FirstChildElement("Enumeration");
        if (!e || !t->Attribute("name")) {
            continue;
        }
        auto& items = enumerations[t->Attribute("name")];
        for (const tinyxml2::XMLElement* i = e->FirstChildElement("Item"); i;
             i = i->NextSiblingElement("Item")) {
            if (i->Attribute("name")) {
                items[i->Attribute("name")] = i->IntAttribute("value");
            }
        }
    }

    const tinyxml2::XMLElement* variables =
        root->FirstChildElement("ModelVariables");
    for (const tinyxml2::XMLElement* v =
             variables ? variables->FirstChildElement("ScalarVariable")
                       : nullptr;
         v;
         v = v->NextSiblingElement("ScalarVariable")) {
        const char* name = v->Attribute("name");
        if (!name) {
            continue;
        }
        Variable var{};
        var.value_reference = v->UnsignedAttribute("valueReference");
        var.causality =
            v->Attribute("causality") ? v->Attribute("causality") : "local";
        var.description =
            v->Attribute("description") ? v->Attribute("description") : "";
        const tinyxml2::XMLElement* enumeration =
            v->FirstChildElement("Enumeration");
        if (v->FirstChildElement("Real")) {
            var.type = Type::Real;
        } else if (enumeration) {
            var.type = Type::Integer;
            const char* type = enumeration->Attribute("declaredType");
            const auto items = enumerations.find(type ? type : "");
            if (items != enumerations.end()) {
                var.items = items->second;
            }
        } else if (v->FirstChildElement("Integer")) {
            var.type = Type::Integer;
        } else if (v->FirstChildElement("Boolean")) {
            var.type = Type::Boolean;
        } else if (v->FirstChildElement("String")) {
            var.type = Type::String;
        } else {
            continue;
        }
        m_variables[name] = var;
    }
}

void Fmi2Slave::load()
{
    const std::filesystem::path library =
        m_dir / "binaries" / "linux64" / (m_model_identifier + ".so");
    m_library = dlopen(library.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!m_library) {
        const char* why = dlerror();
        throw FmuError(
            "cannot load " + library.string() + ": " + (why ? why : "?"));
    }
    auto symbol = [&](const char* name) {
        void* f = dlsym(m_library, name);
        if (!f) {
            throw FmuError(library.string() + " does not export " + name);
        }
        return f;
    };
    m_fn->instantiate =
        reinterpret_cast<fmi2InstantiateTYPE*>(symbol("fmi2Instantiate"));
    m_fn->freeInstance =
        reinterpret_cast<fmi2FreeInstanceTYPE*>(symbol("fmi2FreeInstance"));
    m_fn->setupExperiment = reinterpret_cast<fmi2SetupExperimentTYPE*>(
        symbol("fmi2SetupExperiment"));
    m_fn->enterInitializationMode =
        reinterpret_cast<fmi2EnterInitializationModeTYPE*>(
            symbol("fmi2EnterInitializationMode"));
    m_fn->exitInitializationMode =
        reinterpret_cast<fmi2ExitInitializationModeTYPE*>(
            symbol("fmi2ExitInitializationMode"));
    m_fn->terminate =
        reinterpret_cast<fmi2TerminateTYPE*>(symbol("fmi2Terminate"));
    m_fn->setReal = reinterpret_cast<fmi2SetRealTYPE*>(symbol("fmi2SetReal"));
    m_fn->setInteger =
        reinterpret_cast<fmi2SetIntegerTYPE*>(symbol("fmi2SetInteger"));
    m_fn->setBoolean =
        reinterpret_cast<fmi2SetBooleanTYPE*>(symbol("fmi2SetBoolean"));
    m_fn->setString =
        reinterpret_cast<fmi2SetStringTYPE*>(symbol("fmi2SetString"));
    m_fn->getReal = reinterpret_cast<fmi2GetRealTYPE*>(symbol("fmi2GetReal"));
    m_fn->doStep = reinterpret_cast<fmi2DoStepTYPE*>(symbol("fmi2DoStep"));

    m_callbacks.logger = logMessage;
    m_callbacks.allocateMemory = std::calloc;
    m_callbacks.freeMemory = std::free;
    m_callbacks.stepFinished = stepFinished;
    m_callbacks.componentEnvironment = m_logger.get();

    const std::string resources = "file://" + (m_dir / "resources").string();
    m_component = m_fn->instantiate(
        m_instance_name.c_str(),
        fmi2CoSimulation,
        m_guid.c_str(),
        resources.c_str(),
        &m_callbacks,
        fmi2False,
        fmi2False);
    if (!m_component) {
        throw FmuError("fmi2Instantiate failed for FMU '" + m_model_name + "'");
    }
}

void Fmi2Slave::check(int status, const char* call) const
{
    if (status != fmi2OK && status != fmi2Warning) {
        throw FmuError(
            std::string(call) + " returned " + statusName(status) + " (FMU '" +
            m_model_name + "', instance '" + m_instance_name + "')");
    }
}

bool Fmi2Slave::hasVariable(const std::string& name) const
{
    return m_variables.count(name) > 0;
}

const Fmi2Slave::Variable& Fmi2Slave::variable(const std::string& name) const
{
    const auto it = m_variables.find(name);
    if (it == m_variables.end()) {
        throw FmuError(
            "FMU '" + m_model_name + "' has no variable '" + name + "'");
    }
    return it->second;
}

const Fmi2Slave::Variable& Fmi2Slave::expect(const std::string& name, Type type)
    const
{
    const Variable& v = variable(name);
    if (v.type != type) {
        throw FmuError(
            "FMU '" + m_model_name + "': variable '" + name +
            "' has another type");
    }
    return v;
}

void Fmi2Slave::initialize(double start_time)
{
    check(
        m_fn->setupExperiment(
            m_component,
            fmi2False,
            0.0,
            start_time,
            fmi2False,
            0.0),
        "fmi2SetupExperiment");
    check(
        m_fn->enterInitializationMode(m_component),
        "fmi2EnterInitializationMode");
    check(
        m_fn->exitInitializationMode(m_component),
        "fmi2ExitInitializationMode");
    m_initialized = true;
}

void Fmi2Slave::setReal(const std::string& name, double value)
{
    const fmi2ValueReference vr = expect(name, Type::Real).value_reference;
    check(m_fn->setReal(m_component, &vr, 1, &value), "fmi2SetReal");
}

void Fmi2Slave::setInteger(const std::string& name, int value)
{
    const fmi2ValueReference vr = expect(name, Type::Integer).value_reference;
    const fmi2Integer v = value;
    check(m_fn->setInteger(m_component, &vr, 1, &v), "fmi2SetInteger");
}

void Fmi2Slave::setBoolean(const std::string& name, bool value)
{
    const fmi2ValueReference vr = expect(name, Type::Boolean).value_reference;
    const fmi2Boolean v = value ? fmi2True : fmi2False;
    check(m_fn->setBoolean(m_component, &vr, 1, &v), "fmi2SetBoolean");
}

void Fmi2Slave::setString(const std::string& name, const std::string& value)
{
    const fmi2ValueReference vr = expect(name, Type::String).value_reference;
    const fmi2String v = value.c_str();
    check(m_fn->setString(m_component, &vr, 1, &v), "fmi2SetString");
}

void Fmi2Slave::setFromString(const std::string& name, const std::string& value)
{
    try {
        switch (variable(name).type) {
            case Type::Real:
                return setReal(name, std::stod(value));
            case Type::Integer: {
                const auto& items = variable(name).items;
                const auto item = items.find(value);
                return setInteger(
                    name,
                    item != items.end() ? item->second : std::stoi(value));
            }
            case Type::Boolean:
                return setBoolean(name, value == "true" || value == "1");
            case Type::String:
                return setString(name, value);
        }
    } catch (const std::invalid_argument&) {
        throw FmuError(
            "FMU '" + m_model_name + "': '" + value + "' is no value for '" +
            name + "'");
    } catch (const std::out_of_range&) {
        throw FmuError(
            "FMU '" + m_model_name + "': '" + value +
            "' is out of range for '" + name + "'");
    }
}

double Fmi2Slave::getReal(const std::string& name)
{
    const fmi2ValueReference vr = expect(name, Type::Real).value_reference;
    fmi2Real value = 0;
    check(m_fn->getReal(m_component, &vr, 1, &value), "fmi2GetReal");
    return value;
}

void Fmi2Slave::doStep(double t, double h)
{
    check(m_fn->doStep(m_component, t, h, fmi2True), "fmi2DoStep");
}

}  // namespace lotusim::gazebo
