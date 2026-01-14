# BreakerLib

**Version:** 2025.1

BreakerLib is a comprehensive utility library designed to streamline FRC robot development by providing advanced implementations and utilities that go beyond WPILib's basic functionality. 

## What's Included

BreakerLib is organized into six main components:

### [Drivers](./drivers/README.md)
Hardware driver implementations for various sensors and devices including ZED cameras, RFID readers, REV Blinkin LED controllers, and GTSAM integration for advanced SLAM.

### [Swerve Drive System](./swerve/README.md)
Advanced swerve drive implementations with Phoenix 6 motor controller support, PathPlanner integration, Choreo trajectory following, simulation support, and enhanced teleop control.

### [Driver Station](./driverstation/README.md)
Enhanced driver station and input handling capabilities including improved gamepad support, advanced input stream processing, and haptic feedback control.

### [Physics](./physics/README.md)
Physics and mathematical modeling utilities including 2D and 3D vector operations, chassis acceleration calculations, and spatial mathematics.

### [Sensors](./sensors/README.md)
Sensor utilities and abstractions providing unified interfaces for digital sensors and other sensor types.

### [Utilities](./util/README.md)
General utilities and helper functions including advanced mathematical operations, comprehensive logging systems, command utilities, factory patterns, and data management tools.

## Dependencies

BreakerLib integrates with several key FRC libraries:

- **WPILib**: Core FRC library
- **Phoenix 6**: CTRE motor controller library
- **PathPlanner**: Autonomous path planning
- **Choreo**: Advanced trajectory following
- **GTSAM**: SLAM and mapping (optional)

## Contributing

This library is maintained for FRC team use. For questions or contributions, please contact the development team.

## License

This library follows the WPILib BSD license. See the main project license file for details.

