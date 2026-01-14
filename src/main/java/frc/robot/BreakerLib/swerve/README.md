# Swerve Drive System

Provides a complete system for controlling swerve drive robots - the type of drive system where each wheel can rotate independently to allow the robot to move in any direction while rotating. When you build a swerve drive robot, this system handles all the complex math and motor control needed to make the robot move smoothly, controlling how fast each wheel spins, which direction each wheel points, and coordinating all four wheels to work together. It also handles autonomous driving, simulation testing, and driver control. Swerve drive is one of the most capable drive systems in FRC because it allows robots to move in any direction without turning, making robots much more maneuverable and efficient during games. However, swerve drive is complex to program - this system handles all that complexity so you can focus on your robot's game strategy instead of struggling with drive code.

## Components

### BreakerSwerveDrivetrain.java

The core system that makes your swerve drive robot work. This handles all the complex calculations needed to control four wheels that can each rotate independently. It manages your motors (using Phoenix 6 motor controllers), tracks where your robot is on the field, follows autonomous paths, and handles driver control. Think of it as the "brain" that coordinates all the moving parts of your swerve drive system.

### BreakerSwerveTeleopControl.java

Makes it easier for drivers to control swerve drive robots smoothly and precisely. This system processes driver inputs to prevent jerky movements, smooths out controller inputs, and lets you customize how the robot responds to driver commands. It handles things like preventing accidental movement when controllers are near center position, smoothing out rough joystick movements, and allowing you to set different control modes for different situations.

### BreakerSimSwerveDrivetrain.java

Lets you test your swerve drive code without needing the actual robot hardware. This creates a virtual version of your robot that behaves realistically, so you can test your autonomous programs, driver controls, and other features on your computer before trying them on the real robot. This saves time and prevents damage to your robot while you're developing and testing code.

### BreakerSwerveChoreoController.java

Integrates with Choreo (a powerful trajectory planning tool) to create smooth, efficient autonomous paths. Choreo is a software tool that helps you design complex robot movements and paths, then this component helps your robot follow those paths accurately. It handles the complex math needed to make your robot follow the planned path smoothly, adjusts the path in real-time if needed, and optimizes the robot's movement for efficiency and accuracy.

## Key Capabilities

### Motor Controller Integration

- **Full Phoenix 6 support**: Complete integration with the latest CTRE motor controllers with advanced configuration options
- **Automatic motor configuration**: Automatically sets up motors with optimal settings for swerve drive
- **Comprehensive error handling**: Detects and recovers from motor faults and communication issues

### Autonomous Navigation

- **PathPlanner integration**: Seamlessly works with PathPlanner for generating and following autonomous paths
- **Choreo support**: Advanced trajectory optimization for smooth and efficient autonomous movement
- **Multiple autonomous modes**: Supports different autonomous strategies and behaviors

### Simulation Support

- **Complete simulation environment**: Test your swerve drive code without physical hardware
- **Realistic physics modeling**: Accurate simulation of motor behavior and robot dynamics
- **NetworkTables integration**: Share simulation data with other systems for comprehensive testing

### Teleop Control

- **Advanced input processing**: Smooth and responsive driver control with filtering and deadband handling
- **Configurable control schemes**: Customize how driver inputs translate to robot movement
- **Rate limiting and smoothing**: Prevent jerky movement and provide precise control

## Dependencies

- **Phoenix 6**: CTRE motor controller library
- **PathPlanner**: Autonomous path planning library
- **Choreo**: Advanced trajectory following library
- **WPILib**: Core FRC library for swerve drive base

## Integration

The swerve system integrates with:

- **Odometry systems** for accurate localization
- **Path planning tools** for autonomous navigation
- **Driver station controls** for teleop operation
- **Simulation environments** for testing and development
