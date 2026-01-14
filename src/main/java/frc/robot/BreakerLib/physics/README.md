# Physics

Provides mathematical tools for working with robot movement, positioning, and physical calculations that robots need to understand where they are and how to move accurately. When robots need to calculate distances, angles, speeds, or positions, these tools handle the complex math involved, helping robots understand their own movement, calculate trajectories, and work with coordinate systems. For example, when a robot needs to turn to face a specific direction or move to a precise location, these physics tools do the calculations. Robots need to understand their physical environment and movement to operate effectively - whether it's driving to a specific position, calculating how fast to move, or understanding acceleration and deceleration, these physics tools provide the mathematical foundation that makes precise robot control possible. Without them, robots would have trouble moving accurately or understanding their position on the field.

## Components

### BreakerVector2.java

2D vector mathematics for planar robot operations (movement on a flat surface like the FRC field). Provides vector creation, arithmetic operations, magnitude and angle calculations, normalization, and rotation utilities. This handles the basic math needed for robots moving around on the field - calculating distances, directions, and positions in 2D space.

### BreakerVector3.java

3D vector mathematics for spatial robot operations (movement in three-dimensional space, including height). Provides vector creation, arithmetic operations, magnitude and direction calculations, cross product and dot product operations, and rotation utilities. This is useful for more complex robot movements, mechanisms that move up and down, or when working with 3D positioning systems.

### ChassisAccels.java

Chassis acceleration calculations for robot dynamics. The chassis is the main body of the robot, and this component tracks how quickly the robot is speeding up or slowing down (linear acceleration) and how quickly it's changing direction (angular acceleration). It integrates with swerve drive systems and provides real-time acceleration monitoring to help understand robot performance and movement characteristics.

## Key Capabilities

### Vector Mathematics

- **Comprehensive 2D and 3D vector operations**: Complete set of mathematical operations for working with vectors in both 2D and 3D space
- **Efficient mathematical computations**: Optimized calculations for real-time robot control
- **Type-safe vector manipulations**: Prevents errors when working with vector data
- **Integration with WPILib geometry classes**: Works seamlessly with WPILib's built-in geometry tools

### Robot Dynamics

- **Chassis acceleration tracking**: Monitor how quickly the robot is speeding up or slowing down
- **Real-time dynamics calculations**: Calculate robot movement characteristics as they happen
- **Integration with drive systems**: Work directly with swerve drive and other drive systems
- **Performance monitoring capabilities**: Track robot performance and identify issues

### Spatial Calculations

- **Coordinate system transformations**: Convert between different coordinate systems (field coordinates, robot coordinates, etc.)
- **Rotation and translation utilities**: Handle robot rotation and movement calculations
- **Geometric calculations for robot positioning**: Calculate distances, angles, and positions accurately
- **Integration with odometry systems**: Work with position tracking systems for precise navigation

## Dependencies

- **WPILib**: Core FRC library for geometry and math utilities
- **Java Math**: Standard Java mathematical operations

## Integration

The physics utilities integrate with:

- **Swerve drive systems** for kinematics calculations
- **Odometry systems** for position tracking
- **Path planning** for trajectory calculations
- **Simulation systems** for physics modeling
