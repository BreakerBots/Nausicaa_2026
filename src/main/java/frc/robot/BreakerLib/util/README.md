# Utilities

Provides a collection of helpful tools and functions that make it easier to build and program FRC robots, including math utilities, logging systems, command helpers, and data management tools. These utilities handle common tasks that come up when building robots - things like advanced math calculations, keeping track of what the robot is doing (logging), creating custom commands, and managing data. They provide ready-made solutions for problems that every robot builder faces, so you don't have to reinvent the wheel. Building a competitive FRC robot involves solving many complex problems, and these utilities provide proven solutions for common challenges, saving time and reducing errors. Good logging helps you debug problems quickly, advanced math tools help with precise control, and command utilities make autonomous programming easier. Together, they make robot development more efficient and reliable.

## Core Utilities

### BreakerLibVersion

Tracks the version of the BreakerLib library and provides version comparison utilities. Helps you know which version of BreakerLib you're using and check for compatibility with other components. Useful for debugging and ensuring all parts of your robot code are using the same library version.

**Key features**:
- **Current version tracking**: Always know which version of BreakerLib you're using
- **Version comparison utilities**: Check if your version is compatible with other components
- **Library identification**: Easily identify which library version is being used

### Localizer

Provides utilities for tracking and managing robot position and orientation on the field. When your robot needs to know where it is on the field or track its movement, the Localizer handles the coordinate system transformations and position calculations. Essential for autonomous navigation and precise positioning.

**Key features**:
- **Position and orientation tracking**: Keep track of where the robot is and which way it's facing
- **Coordinate system transformations**: Convert between different coordinate systems (field coordinates, robot coordinates, etc.)
- **Integration with odometry systems**: Work with position tracking systems for accurate navigation

### MechanismRatio

Calculates gear ratios and mechanical relationships for robot mechanisms like gearboxes and transmissions. When you design mechanisms with gears, pulleys, or other mechanical components, this utility helps you calculate the relationship between input and output speeds, torques, and positions. Essential for designing efficient mechanisms.

**Key features**:
- **Gear ratio calculations**: Determine the relationship between input and output speeds
- **Speed and torque conversions**: Convert between different units and understand power transmission
- **Mechanism efficiency tracking**: Monitor how well your mechanisms are performing

### TimestampedValue

Stores values along with timestamps for data logging and time-based analysis. When you need to track how values change over time (like motor speeds, sensor readings, or robot positions), this utility stores both the value and when it was recorded. Useful for debugging, performance analysis, and data logging.

**Key features**:
- **Value storage with timestamps**: Store data with precise timing information
- **Time-based data analysis**: Analyze how values change over time
- **Integration with logging systems**: Work with logging systems for comprehensive data tracking

## Command Utilities

### commands/

Provides custom command implementations and utilities that extend WPILib's command system. When you need specialized commands for your robot that aren't available in standard WPILib, these utilities provide ready-made solutions. Makes autonomous programming and complex robot behaviors easier to implement.

**Files**:
- **TimedWaitUntilCommand.java**: Commands that wait for a specific amount of time or until a condition is met

**Key features**:
- **Custom command implementations**: Create specialized commands for your robot's unique needs
- **Advanced command patterns**: Use proven patterns for complex robot behaviors
- **Integration with WPILib command system**: Work seamlessly with WPILib's command framework

## Factory Patterns

### factory/

Provides factory pattern implementations for creating and configuring hardware objects consistently. When you need to create multiple instances of hardware (like motor controllers or sensors) with similar configurations, factory patterns ensure consistency and reduce code duplication. Makes hardware setup more reliable and maintainable.

**Files**:
- **BreakerCANCoderFactory.java**: Factory for creating and configuring CANCoder sensors

**Key features**:
- **Factory patterns for hardware creation**: Create hardware objects with consistent configurations
- **Configuration management**: Manage hardware settings in a centralized way
- **Error handling in object creation**: Detect and handle problems during hardware setup

## Logging System

### logging/

Provides a comprehensive logging system for tracking what your robot is doing and debugging problems. When your robot is running, the logging system records important events, errors, and data. This helps you understand what happened when things go wrong and track robot performance over time. Essential for debugging and optimization.

**Files**:
- **BreakerLog.java**: Main logging interface for recording messages and events
- **Elastic.java**: Integration with Elasticsearch for storing logs remotely
- **LoggedAlert.java**: System for creating and managing alerts for critical events

**Key features**:
- **Multi-level logging**: Different levels of detail (DEBUG, INFO, WARNING, ERROR) for different types of information
- **NetworkTables integration**: Share log data in real-time with other systems for monitoring
- **File-based logging**: Store logs locally for offline analysis
- **Elasticsearch integration**: Store logs remotely for team-wide access and analysis
- **Alert system**: Create notifications for critical events that need immediate attention

## Mathematical Utilities

### math/

Provides advanced mathematical operations and utilities for robot calculations and data processing. When you need to perform complex math operations, interpolate between values, or fuse data from multiple sources, these utilities provide the mathematical foundation. Essential for precise robot control and data analysis.

**Files**:
- **BreakerMath.java**: Extended mathematical operations and functions
- **OdometryFusion.java**: Combines data from multiple odometry sources for improved accuracy
- **OdometryFusion3d.java**: 3D version of odometry fusion for spatial positioning
- **functions/**: Mathematical function implementations and utilities
- **interpolation/**: Algorithms for interpolating between data points

**Key features**:
- **Extended mathematical functions**: Advanced math operations beyond basic arithmetic
- **Interpolation algorithms**: Smooth transitions between data points using various mathematical methods
- **Odometry fusion**: Combine multiple position sensors for more accurate robot location
- **Mathematical function implementations**: Ready-made solutions for common mathematical problems
- **Advanced statistical operations**: Analyze data and calculate statistics for robot performance

## Dependencies

- **WPILib**: Core FRC library for basic utilities
- **NetworkTables**: For logging and data sharing
- **Elasticsearch**: For remote log storage (optional)
- **Java Math**: Standard Java mathematical operations

## Integration

The utility classes integrate with:
- **All BreakerLib components** for enhanced functionality
- **Robot subsystems** for logging and diagnostics
- **Command systems** for advanced command patterns
- **Hardware systems** for factory-based creation
- **External systems** for data logging and analysis
