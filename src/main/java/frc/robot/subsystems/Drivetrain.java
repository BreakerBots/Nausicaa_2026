// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.BackLeft;
import static frc.robot.Constants.DriveConstants.BackRight;
import static frc.robot.Constants.DriveConstants.DRIVETRAIN_CONSTANTS;
import static frc.robot.Constants.DriveConstants.FrontLeft;
import static frc.robot.Constants.DriveConstants.FrontRight;

import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain;

public class Drivetrain extends BreakerSwerveDrivetrain {
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    super(DRIVETRAIN_CONSTANTS, FrontLeft, FrontRight, BackLeft, BackRight);
  }
}
