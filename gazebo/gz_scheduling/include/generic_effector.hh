#pragma once
#include <gz/msgs.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>

#include <string>

/// @brief A generic effector from which every effector derives. An effector
/// should have its data set up in its constructor and have its logic by
/// implementing the abstract apply_effector function.
class GenericEffector {
public:
    GenericEffector(){};
    // This function is supposed to be called on PreUpdate and setup the
    // effectors to be applied on Update, especially from Xdyn.
    virtual void apply_effector() = 0;
};