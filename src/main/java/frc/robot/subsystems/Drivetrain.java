// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.BackLeft;
import static frc.robot.Constants.DriveConstants.BackRight;
import static frc.robot.Constants.DriveConstants.DRIVETRAIN_CONSTANTS;
import static frc.robot.Constants.DriveConstants.FrontLeft;
import static frc.robot.Constants.DriveConstants.FrontRight;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
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

    /**
     * Returns the vector from the shooter center to the given field point.
     * Accounts for shooter offset (3" right of robot center) so rotation aligns the shooter, not the robot center.
     */
    public Translation2d getShooterCenterToPointTranslation(Translation2d targetPoint) {
        Pose2d pose = getLocalizer().getPose();
        Translation2d robotCenter = pose.getTranslation();
        double theta = pose.getRotation().getRadians();
        // Shooter is offset to the right: in robot frame +Y is left, so right = -Y. Rotate (0, -offset) to field.
        double dx = Constants.ShooterConstants.SHOOTER_OFFSET_RIGHT_METERS * Math.sin(theta);
        double dy = -Constants.ShooterConstants.SHOOTER_OFFSET_RIGHT_METERS * Math.cos(theta);
        Translation2d shooterCenter = robotCenter.plus(new Translation2d(dx, dy));
        return targetPoint.minus(shooterCenter);
    }

    /** Locks wheels in X pattern (brake) to resist motion during shooting. 
     * Unlock by running any drivetrain command.
     * Phase 1: Position wheels in X using Velocity.
     * Phase 2: DutyCycleOut(0) on drive (direct brake), PositionVoltage on steer (hold angle). */
    private static final double LOCK_POSITION_SECONDS = 0.3;

    public Command lockWheelsCommand() {
        return Commands.run(() -> setControl(new SwerveRequest.SwerveDriveBrake()
            .withDriveRequestType(DriveRequestType.Velocity)), this)
            .withTimeout(LOCK_POSITION_SECONDS)
            .andThen(Commands.run(() -> setControl(new HoldWithDriveBrakeRequest()), this))
            .finallyDo(interrupted -> setControl(new SwerveRequest.Idle()));
    }

    /** Holds steer at current angle, applies DutyCycleOut(0) to drive (direct brake). Used after wheels are in X. */
    private static class HoldWithDriveBrakeRequest implements SwerveRequest {
        @Override
        public StatusCode apply(com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
            for (var module : modulesToApply) {
                double steerPos = module.getSteerMotor().getPosition().getValueAsDouble();
                module.apply(new DutyCycleOut(0), new PositionVoltage(steerPos));
            }
            return StatusCode.OK;
        }
    }
}
