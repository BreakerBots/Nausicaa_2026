# Drivers

Provides software to talk to and control various sensors and devices that robots use to understand their environment and operate effectively. When you connect sensors like cameras, RFID readers, or LED controllers to your robot, these drivers handle the communication between your robot's code and the actual hardware, translating raw data into useful information your robot can use. FRC robots need to sense their environment to make good decisions - cameras help robots see game pieces and their position on the field, RFID readers can identify batteries or game elements, and LED controllers provide visual feedback to drivers and spectators. Without these drivers, you'd have to write complex code from scratch to communicate with each device.

## Components

### ZED.java

ZED camera integration for computer vision and pose estimation. The ZED is a stereo camera (a camera with two lenses that work like human eyes) that can see in 3D and track objects in real-time. This driver provides real-time object detection, pose estimation (figuring out where objects are located and oriented), and NetworkTables integration for sharing camera data across the robot.

### BatteryRFID.java

RFID (Radio Frequency Identification) reader for battery identification and management. RFID uses small electronic tags that can be read wirelessly - think of it like a barcode that doesn't need to be visible. This driver tracks battery usage and provides battery monitoring capabilities by reading RFID tags attached to batteries.

### BreakerRevBlinkin.java

REV Blinkin LED controller driver. REV Robotics makes a device called the Blinkin that can control multiple LED lights and create different lighting patterns. This driver controls LED patterns for robot status indication and visual feedback - for example, showing different colors to indicate if the robot is ready, has an error, or is in autonomous mode.

### gtsam/

GTSAM (Georgia Tech Smoothing and Mapping) integration for advanced SLAM capabilities. SLAM stands for "Simultaneous Localization and Mapping" - it's a technique that helps robots build a map of their environment while figuring out where they are in that map at the same time. GTSAM is a powerful library that makes SLAM more accurate and efficient.

**Files**:

- **GTSAM.java**: Main GTSAM integration interface
- **TagDetection.java**: AprilTag detection utilities (AprilTags are special black and white patterns that robots can easily recognize and use for navigation)
- **TagDetectionStruct.java**: Data structures for tag detection

## Dependencies

- **NetworkTables**: For ZED camera data communication
- **GTSAM Library**: For advanced SLAM functionality
- **REV Hardware**: For Blinkin LED controller support

## Integration

These drivers integrate seamlessly with the rest of BreakerLib, providing enhanced sensor capabilities for:

- Autonomous navigation and localization
- Robot status indication and debugging
- Battery management and monitoring
- Computer vision applications
