#ifndef __PARAM_NODE_H__
#define __PARAM_NODE_H__

#include "DataLoader.h"

namespace liquidai {
namespace gazebo {

/**
 * @brief Base class for all node handling params.
 *
 * @tparam DataType
 */
template <typename DataType>
class ParamNode {
public:
    ParamNode()
        : m_current_index(0)
        , m_using_file(false)
    {
    }

    DataType &getParam()
    {
        std::lock_guard<std::mutex> lock(m_param_mutex);
        auto it = m_param_map.begin();
        std::advance(it, m_current_index);
        return it->second;
    }

    bool setParam(const DataType &param)
    {
        if (m_using_file) {
            // If params are retreived from file, no param changes is allowed
            return false;
        }
        std::lock_guard<std::mutex> lock(m_param_mutex);
        m_param_map.begin()->second = param;
        return true;
    }

    bool usingFile()
    {
        std::lock_guard<std::mutex> lock(m_param_mutex);
        return m_using_file;
    }

    /**
     * @brief The next timestep. Return true if there is a change in param.
     *
     */
    bool step(const std::chrono::steady_clock::duration &current_time)
    {
        auto it = m_param_map.begin();
        std::advance(it, m_current_index);

        if (m_using_file && m_current_index < m_param_map.size() - 1 &&
            it->first >= current_time) {
            m_current_index += 1;
            return true;
        }
        return false;
    }

protected:
    /**
     * @brief Logger to log all errors in the system used
     *
     * @param err
     */
    virtual void logError(std::string err_statement) = 0;

    /**
     * @brief Params is not provided through file. Derived function is to
     * specify where to get the params
     *
     * @param file_path
     * @param format
     * @param default_param
     * @return true
     * @return false
     */
    bool loadParamFromFile(
        const std::string &file_path,
        liquidai::FileFormat format,
        DataType &default_param)
    {
        bool result(false);
        std::lock_guard<std::mutex> lock(m_param_mutex);
        if (file_path.empty() || !loadFile(file_path, format, m_param_map)) {
            logError({"ParamNode::loadParam: Param loaded is empty. "
                      "Loading default param."});
        }
        else {
            m_using_file = true;
            result = true;
        }

        if (m_param_map.empty() ||
            m_param_map.begin()->first != std::chrono::seconds(0)) {
            m_param_map.insert({std::chrono::seconds(0), default_param});
        }
        return result;
    }

protected:
    /**
     * @brief mapping of the time of simulation to the param
     *
     */
    std::map<std::chrono::steady_clock::duration, DataType> m_param_map;

    /**
     * @brief Mutex for param map
     *
     */
    std::mutex m_param_mutex;

    bool m_using_file;
    size_t m_current_index;
};

} // namespace gazebo
} // namespace liquidai

#endif