# Sensors

Provides a standardized way to work with various sensors that robots use to detect things in their environment, like limit switches, proximity sensors, and other digital input devices. When you add sensors to your robot (like switches that detect when mechanisms reach certain positions, or sensors that detect nearby objects), this system provides a consistent way to read and work with all these different types of sensors, instead of having to write different code for each type of sensor. Sensors are crucial for robots to understand what's happening around them and to operate safely and effectively - they help robots know when mechanisms are in the right position, detect obstacles, and understand their environment. This system makes it much easier to add and work with sensors, reducing the time and effort needed to integrate them into your robot.

## Components

### BreakerDigitalSensor.java

A simple, consistent way to work with any type of digital sensor (sensors that give you a simple on/off signal). Instead of having to learn different programming methods for each type of sensor, this gives you one standard way to read all digital sensors. Whether you're using a limit switch, proximity sensor, or any other digital sensor, you can use the same code to check if the sensor is triggered or not.

## Key Capabilities

### Sensor Abstraction

- **Unified interface for different sensor types**: Use the same code to work with any type of digital sensor
- **Hardware-agnostic operations**: Works regardless of the specific sensor hardware being used
- **Consistent error handling**: Standardized way to detect and handle sensor problems
- **Status monitoring and diagnostics**: Easy way to check if sensors are working properly

### Digital Sensor Support

- **Limit switches**: Detect when mechanisms reach end positions or specific locations
- **Proximity sensors**: Detect when objects are nearby without physical contact
- **Optical sensors**: Use light to detect objects, colors, or distances
- **Hall effect sensors**: Detect magnetic fields for position or speed sensing
- **Any digital input device**: Works with any sensor that provides on/off signals

### Integration Features

- **Easy integration with robot subsystems**: Simple to add sensors to existing robot systems
- **Consistent API across sensor types**: Same programming interface for all sensors
- **Built-in error detection and handling**: Automatically detects when sensors fail or malfunction
- **Status reporting capabilities**: Get information about sensor health and performance

## Dependencies

- **WPILib**: Core FRC library for digital input handling
- **Hardware-specific drivers**: For various sensor types

## Integration

The sensor utilities integrate with:

- **Robot subsystems** for sensor-based operations
- **Command systems** for sensor-triggered commands
- **Safety systems** for sensor-based safety features
- **Diagnostic systems** for sensor monitoring and troubleshooting

## Extending the System

To add support for new sensor types:

1. Implement the `BreakerDigitalSensor` interface
2. Provide hardware-specific configuration
3. Implement required sensor operations
4. Add error handling and status monitoring
