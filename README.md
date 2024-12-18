# LOTUSim

This is an opensource simulator for EDB's project and is created based on opensource [Plankton](https://github.com/Liquid-ai/Plankton), [uuv](https://github.com/uuvsimulator/uuv_simulator) , [xdyn](https://github.com/sirehna/xdyn).

This simulation is built on Gazebo Harmonic and ROS Humble.

</div>

[[_TOC_]]

## Get Started

### Installing

1. Creating workspace

```bash
cd;
git clone https://developers.naval-group.com/gitlab/naval-group/naval-group-pacific/lotus/lotusim.git lotusim_ws/src/lotusim;
```

2. Adding environment variable

```bash
cat <<EOF >> ~/.bashrc
export PATH=\$HOME/lotusim_ws/src/lotusim/physics/:\$HOME/lotusim_ws/src/lotusim/launch:\$PATH
export LD_LIBRARY_PATH=\$HOME/lotusim_ws/src/lotusim/physics:\$LD_LIBRARY_PATH
export LOTUSIM_WS=\$HOME/lotusim_ws/
export LOTUSIM_PATH=\$LOTUSIM_WS/src/lotusim
export XDYN_PATH=\$LOTUSIM_PATH/assets/models/
source \$LOTUSIM_PATH/launch/bash_completion.sh
EOF
chmod -R +x $HOME/lotusim_ws/src/lotusim/launch/*
source ~/.bashrc
```

3. Install [gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu), [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html), and libraries needed

```bash
lotusim install
```

4. Read help for Lotusim and xdyn

```bash
lotusim --help
xdyn --help
xdyn-for-cs --help
```

## Tutorial

### Running example without xdyn

```bash
lotusim --gui run no_xdyn_test.world
```

### Running example with xdyn

1. Run surface and underwater xdyn

```bash
xdyn-for-cs $HOME/lotusim_ws/src/lotusim/assets/models/dtmb_hull/dtmb-xdyn.yml --verbose --address 127.0.0.1 --dt 0.2 --port 12345
xdyn-for-cs $HOME/lotusim_ws/src/lotusim/assets/models/lrauv_xdyn/lrauv.yml --verbose --address 127.0.0.1 --dt 0.2 --port 12346
```

2. Run lotusim

```bash
lotusim run <world>
```

### Building lotusim in debug mode for debug logs

```
lotusim --debug clean_build
```

## Unity interface 

### By TCP/UDP 

Inside asset/models/model.sdf

```xml
<render_interface>
    <publish_render>true</publish_render>
    <renderer_type_name>lrauv</renderer_type_name>
</render_interface>
```

Inside world.sdf

```xml
<plugin filename="render_plugin" name="lotusim::gazebo::RenderPlugin">
    <connection_protocol>TCPUDP</connection_protocol>
    <ip>127.0.0.1</ip>
    <udp_port>23456</udp_port>
    <tcp_port>23457</tcp_port>
</plugin>
```

## Contributing

### Workflow

We use the Gitflow collaborating workflow. You can find the explanation of this workflow [here](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow).

We are using the default Gitflow branch naming like [here](https://www.gitkraken.com/blog/gitflow).

### Issues

When you open an issue, you need to put the correct label corresponding to the category of the issue e.g. ~bug or ~suggestion. It needs to be written in english.

 **IMPORTANT** You have to put ~"project::development" or ~"project::management" label on each issue as they are used for filtering in the different issue boards

You can find the description of the labels [here](https://developers.naval-group.com/gitlab/naval-group/naval-group-pacific/lotusim/-/labels).
