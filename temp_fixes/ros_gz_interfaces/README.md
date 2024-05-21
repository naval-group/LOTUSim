# Message and service data structures for interacting with Gazebo from ROS2

This package currently contains some Gazebo-specific ROS message and service data structures (.msg and .srv)

## Messages (.msg)

* [Contact](msg/Contact.msg): related to [gz::msgs::Contact](https://github.com/gazebosim/gz-msgs/blob/ign-msgs7/proto/gz/msgs/contact.proto). Contant info bewteen collisions in Gazebo Sim.
* [Contacts](msg/Contacts.msg): related to [gz::msgs::Contacts](https://github.com/gazebosim/gz-msgs/blob/ign-msgs7/proto/gz/msgs/contacts.proto). A list of contacts.
* [Entity](msg/Entity.msg): related to [gz::msgs::Entity](https://github.com/gazebosim/gz-msgs/blob/ign-msgs7/proto/gz/msgs/entity.proto). Entity of Gazebo Sim.
* [EntityFactory](msg/EntityFactory.msg): related to [gz::msgs::EntityFactory](https://github.com/gazebosim/gz-msgs/blob/ign-msgs7/proto/gz/msgs/entity_factory.proto). Message to create a new entity.
* [Light](msg/Light.msg): related to [gz::msgs::Light](https://github.com/gazebosim/gz-msgs/blob/ign-msgs7/proto/gz/msgs/light.proto). Light info in Gazebo Sim.
* [WorldControl](msg/WorldControl.msg): related to [gz::msgs::WorldControl](https://github.com/gazebosim/gz-msgs/blob/ign-msgs7/proto/gz/msgs/world_control.proto). Message to control world of Gazebo Sim.
* [WorldReset](msg/WorldReset.msg): related to [gz::msgs::WorldReset](https://github.com/gazebosim/gz-msgs/blob/ign-msgs7/proto/gz/msgs/world_reset.proto). Reset time and model of simulation.

## Services (.srv)

* [ControlWorld](srv/ControlWorld.srv): Control world of Gazebo Sim,for example,pasue,pasue with multiple steps,resume,etc.
* [DeleteEntity](srv/DeleteEntity.srv): Delete Entity in Gazebo Sim
* [SetEntityPose](srv/SetEntityPose.srv): Set pose of Entity in Gazebo Sim
* [SpawnEntity](srv/SpawnEntity.srv): Spawn a Entity in Gazebo Sim
