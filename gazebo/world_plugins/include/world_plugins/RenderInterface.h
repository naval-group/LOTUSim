#ifndef __RENDER_INTERFACE_HH__
#define __RENDER_INTERFACE_HH__

#include <chrono>
#include <gz/sim/components/Pose.hh>
#include <sdf/sdf.hh>
#include <vector>

namespace liquidai {
namespace gazebo {

class RenderInterfaceBase {
public:
    virtual bool
    ConfigureInterface(const std::shared_ptr<const sdf::Element> &_sdf) = 0;

    virtual bool SendPosition(
        const std::chrono::steady_clock::duration &runTime,
        const std::vector<std::pair<std::string, gz::math::Pose3d>> &poses) = 0;
};

} // namespace gazebo
} // namespace liquidai

#endif