# Driver Station

Provides enhanced tools for controlling robots through gamepads and other input devices, making it easier for drivers to control robots precisely and comfortably. When drivers use gamepads to control robots, this system processes their inputs to make control smoother and more responsive, handling things like smoothing out jerky joystick movements, adding deadzones to prevent accidental movements, and providing haptic feedback (rumble) to give drivers information about what the robot is doing. Good driver control is crucial in FRC - the difference between winning and losing often comes down to how well drivers can control their robots. This system makes robots easier and more comfortable to drive by smoothing out inputs, providing better feedback, and supporting different types of controllers, helping reduce driver fatigue during long competition days.

## Components

### BreakerInputStream.java

Enhanced input stream handling with advanced processing capabilities. Provides input filtering, deadband handling, rate limiting, and customizable input transformations.

### BreakerInputStream2d.java

2D input stream utilities for joystick and thumbstick processing. Handles magnitude and angle calculations, deadzone handling, and coordinate system transformations.

### gamepad/

Gamepad controller implementations that extend WPILib's basic gamepad support with additional features and easier-to-use interfaces.

**Files**:

- **BreakerGenericGamepad.java**: Base gamepad interface that provides consistent access to all controller components
- **BreakerXboxController.java**: Xbox controller implementation with simplified button access
- **BreakerPlaystationController.java**: PlayStation controller implementation
- **BreakerControllerRumbleType.java**: Rumble control types for haptic feedback
- **components/**: Individual gamepad component implementations

Provides gamepad handling that goes beyond WPILib's basic support by offering component-level access to buttons, triggers, and thumbsticks, advanced rumble control with different patterns, and simplified programming interfaces that make it easier to work with controller inputs.

## Key Capabilities

### Input Processing

- **Advanced filtering and smoothing**: Removes noise and jerky movements from joystick inputs for smoother control
- **Deadband handling**: Prevents accidental robot movement when joysticks are near center position
- **Rate limiting and acceleration curves**: Controls how quickly inputs change to prevent sudden movements
- **Customizable input transformations**: Modify how driver inputs translate to robot actions

### Gamepad Support

- **Component-level access**: Direct access to individual buttons, triggers, and thumbsticks through dedicated objects
- **Simplified programming**: Easier-to-use methods for accessing controller inputs compared to raw WPILib
- **Multiple controller types**: Support for Xbox, PlayStation, and generic controllers
- **Consistent interface**: Same programming approach works across different controller types

### Haptic Feedback

- **Advanced rumble control**: Precise control over controller vibration patterns and intensity
- **Multiple rumble types and patterns**: Different vibration patterns for different types of feedback
- **Timed rumble commands**: Automatically stop rumble after a specified time
- **Customizable haptic feedback**: Create custom vibration patterns for specific robot events

### 2D Input Handling

- **Joystick and thumbstick processing**: Specialized handling for 2D input devices
- **Magnitude and angle calculations**: Convert X/Y inputs into magnitude and direction
- **Coordinate system transformations**: Handle different coordinate systems and orientations
- **Deadzone handling for precise control**: Prevent accidental movement near the center of joysticks

## Dependencies

- **WPILib**: Core FRC library for basic input handling
- **XInput**: For Xbox controller support
- **DirectInput**: For generic gamepad support

## Integration

The driver station utilities integrate with:

- **Swerve drive systems** for teleop control
- **Robot subsystems** for input processing
- **Command systems** for input-based commands
- **Haptic feedback systems** for user interaction
