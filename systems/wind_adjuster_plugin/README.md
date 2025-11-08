# README: Wind Adjuster Plugin

This README provides an overview of the `WindAdjusterPlugin`, a Gazebo GUI plugin that allows users to adjust wind velocity in a simulated environment. The plugin is designed to interact with Gazebo's simulation world and provides a user-friendly interface for wind manipulation.

## Prerequisites

To use this plugin, ensure that you have the following installed:
- Gazebo Simulator
- Qt libraries (specifically `QQmlContext`, `QQmlEngine`, and `QtQml`)

## Plugin Overview

### Key Features

- **Dynamic Wind Adjustment**: Adjust wind velocities along the X, Y, and Z axes in real-time.
- **QML Integration**: Seamlessly integrates with QML for a smooth user interface experience.
- **Gazebo Messaging**: Publishes wind velocity updates to the Gazebo simulation environment.

### Key Components

#### Constructor and Destructor

- `WindAdjusterPlugin()`: Initializes the wind publisher and registers the plugin with QML.
- `~WindAdjusterPlugin()`: Explicit destructor definition.

#### `LoadConfig`

- Configures the plugin and sets the QML context property to the plugin instance.

#### `Update`

- A stub function for potential future updates.

### Wind Velocity Accessors and Mutators

- `XVelocity()`, `YVelocity()`, `ZVelocity()`: Get the current wind velocities along the respective axes.
- `SetXVelocity(double x)`, `SetYVelocity(double y)`, `SetZVelocity(double z)`: Set the wind velocities and publish updates.

### `setWindVelocity`

- Publishes the wind velocity to the Gazebo simulation using the `gz::msgs::Wind` message.

## Usage

### Installation

1. Place the `WindAdjusterPlugin` source files in your Gazebo plugin directory.
2. Build the plugin using the appropriate CMake configuration.

### Running the Plugin

1. Launch Gazebo with a world that includes wind simulation.
2. Load the `WindAdjusterPlugin` via the Gazebo GUI or specify it in the world file.
3. Use the plugin interface to adjust wind velocities in real-time.


