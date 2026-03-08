// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.BackLeft;
import static frc.robot.Constants.DriveConstants.BackRight;
import static frc.robot.Constants.DriveConstants.DRIVETRAIN_CONSTANTS;
import static frc.robot.Constants.DriveConstants.FrontLeft;
import static frc.robot.Constants.DriveConstants.FrontRight;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain;

public class Drivetrain extends BreakerSwerveDrivetrain {

    public Drivetrain() {
        super(DRIVETRAIN_CONSTANTS, FrontLeft, FrontRight, BackLeft, BackRight);
    }

    /** Returns the vector from the robot's position to the given field point. */
    public Translation2d getRobotToPointTranslation(Translation2d targetPoint) {
        return targetPoint.minus(getLocalizer().getPose().getTranslation());
    }
}
