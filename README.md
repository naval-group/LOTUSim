# <div align="center">  LOTUSim </div>


This is an opensource simulator for EDB's project and is created based on opensource [Plankton](https://github.com/Liquid-ai/Plankton), [uuv](https://github.com/uuvsimulator/uuv_simulator) , [xdyn](https://github.com/sirehna/xdyn).

This simulation is built on Gazebo Harmonic and ROS Humble.


[[_TOC_]]

## Get Started

### Installing

1. Creating workspace

```bash
cd;
mkdir -p lotusim_ws/src;
cd lotusim_ws/src;
git clone -b develop https://developers.naval-group.com/gitlab/naval-group/naval-group-pacific/lotus/lotusim.git 
```

2. Adding environment variable
```
bash lotusim_ws/src/lotusim/launch/lotusim_first_run.sh
```

3. Read help for Lotusim and xdyn

```bash
lotusim --help
xdyn --help
xdyn-for-cs --help
```

## Tutorial

### Running example without xdyn

```bash
lotusim --gui run drl/poc_drl.world
```

### Running example with xdyn

1. Run surface xdyn

```bash
xdyn-for-cs $HOME/lotusim_ws/src/lotusim/assets/models/dtmb_hull/dtmb-xdyn.yml --verbose --address 127.0.0.1 --dt 0.2 --port 12345
```

2. Run xdyn underwater

```bash
xdyn-for-cs $HOME/lotusim_ws/src/lotusim/assets/models/lrauv/lrauv.yml --verbose --address 127.0.0.1 --dt 0.2 --port 12346
```

3. Run lotusim

To run with xdyn, your world should contain:

```xml
<plugin filename="physics_interface_plugin" name="lotusim::gazebo::PhysicsInterfacePlugin"></plugin>
```

Then:

```bash
lotusim --gui run xdyn_test.world
```

4. Closing

Close terminals using ctrl+c 

### Building lotusim in debug mode for debug logs

```
lotusim --debug clean_build
```

## Unity interface 

### By TCP/UDP 

1. assets/models/your_model_folder/model.sdf should contain: 

```xml
<render_interface>
    <publish_render>true</publish_render>
    <renderer_type_name>lrauv</renderer_type_name>
</render_interface>
```
The name of your model need to be exactly the same inside Unity.

2. assets/worlds/your_world.sdf should contain: 


```xml
<plugin filename="render_plugin" name="lotusim::gazebo::RenderPlugin">
    <connection_protocol>TCPUDP</connection_protocol>
    <ip>127.0.0.1</ip>
    <udp_port>23456</udp_port>
    <tcp_port>23457</tcp_port>
</plugin>
```

3. Open scene in [Unity](https://developers.naval-group.com/gitlab/naval-group/naval-group-pacific/lotus/unity/unity-modules/-/tree/features/ros-node?ref_type=heads)

Make sure world script "Gazebo model interface" is activated.

## Contributing

### Workflow

We use the Gitflow collaborating workflow. You can find the explanation of this workflow [here](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow).

We are using the default Gitflow branch naming like [here](https://www.gitkraken.com/blog/gitflow).

### Issues

When you open an issue, you need to put the correct label corresponding to the category of the issue e.g. ~bug or ~suggestion. It needs to be written in english.

 **IMPORTANT** You have to put ~"project::development" or ~"project::management" label on each issue as they are used for filtering in the different issue boards

You can find the description of the labels [here](https://developers.naval-group.com/gitlab/naval-group/naval-group-pacific/lotusim/-/labels).
