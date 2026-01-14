// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Roller.State;


public final class Autos {
  /** Example static factory for an autonomous command. */

  public static Command moveForward(Drivetrain drivetrain, Roller roller, Arm arm) {
    final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

    // Makes the auto run positive if red, negative if blue   (flip automation)
    var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;
    var directionVelocity =  alliance == Alliance.Red ? 0.5 : -0.5;   // Temporary slower to avoid skid

    return Commands.sequence(
      Commands.runOnce(() -> drivetrain.setControl(request.withVelocityX(directionVelocity)), drivetrain),  // make this positive if red, negative if blue
      Commands.waitSeconds(6.5), // 2.2 for CENTER  |  3.05 for 10 ft(angle)  |  6.7 for 22 ft(taxi)   ALL TIMES FOR 1 METER PER SEC
      Commands.runOnce(() -> drivetrain.setControl(request.withVelocityX(0.0)), drivetrain),
      Commands.runOnce(() -> arm.setStateCommand(Arm.State.EXTAKE)),
      Commands.runOnce(() -> roller.setState(Roller.State.CORAL_EXTAKE)),
      Commands.waitSeconds(1.0),
      Commands.runOnce(() -> roller.setState(Roller.State.IDLE))

    );
  }

  // public static Command startSide(Drivetrain drivetrain, Roller roller, Arm arm) {
  //   final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

  //   // Makes the auto run positive if red, negative if blue   (flip automation)
  //   var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;
  //   var directionVelocity =  alliance == Alliance.Red ? 0.5 : -0.5;

  //   return Commands.sequence(
  //     Commands.runOnce(() -> drivetrain.setControl(request.withVelocityX(directionVelocity)), drivetrain),  // make this positive if red, negative if blue
  //     Commands.waitSeconds(6.5), // 2.2 for CENTER  |  3.05 for 10 ft(angle)  |  6.7 for 22 ft(taxi)   ALL TIMES FOR 1 METER PER SEC
  //     Commands.runOnce(() -> drivetrain.setControl(request.withVelocityX(0.0)), drivetrain),
  //     Commands.runOnce(() -> roller.setState(Roller.State.CORAL_EXTAKE)),
  //     Commands.runOnce(() -> arm.setStateCommand(Arm.State.EXTAKE)),
  //     Commands.waitSeconds(1.0),
  //     Commands.runOnce(() -> roller.setState(Roller.State.IDLE)),

      
  //   );
  // }
}
