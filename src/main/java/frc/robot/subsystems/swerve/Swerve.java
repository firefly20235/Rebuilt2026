// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.revrobotics.spark.config.ClosedLoopConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.subsystems.swerve.SwerveConstants.MAX_VELOCITY;

/**
 * Represents a swerve drive style drivetrain.
 */
public class Swerve extends SubsystemBase {
    public Swerve() {
        SwerveConstants.m_gyro.reset();
    }

    public static FunctionalCommand getDriveCommand(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, boolean isFieldRelative) {
        return new FunctionalCommand(
                () -> {},
                () -> drive(x.getAsDouble(), y.getAsDouble(), theta.getAsDouble(), isFieldRelative),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
//    public static void drive(
//            double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
//        var swerveModuleStates =
//                SwerveConstants.m_kinematics.toSwerveModuleStates(
//                        ChassisSpeeds.discretize(
//                                fieldRelative
//                                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
//                                        xSpeed, ySpeed, rot, SwerveConstants.m_gyro.getRotation2d())
//                                        : new ChassisSpeeds(xSpeed, ySpeed, rot),
//                                Constants.LOOP_PERIOD_SECONDS));
//        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_VELOCITY.in(Units.MetersPerSecond));
//        SwerveConstants.m_frontLeft.setState(swerveModuleStates[0], false);
//        SwerveConstants.m_frontRight.setState(swerveModuleStates[1], false);
//        SwerveConstants.m_backLeft.setState(swerveModuleStates[2], false);
//        SwerveConstants.m_backRight.setState(swerveModuleStates[3], false);
//
//
//    }
    public static void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        ChassisSpeeds rawSpeeds =
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot,
                        SwerveConstants.m_gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot);

        ChassisSpeeds discretizedSpeeds =
                ChassisSpeeds.discretize(
                        rawSpeeds,
                        Constants.LOOP_PERIOD_SECONDS
                );


        SwerveModuleState[] moduleStates =
                SwerveConstants.m_kinematics.toSwerveModuleStates(discretizedSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
                moduleStates,
                SwerveConstants.MAX_VELOCITY.in(Units.MetersPerSecond)
        );

        System.out.println("x=" + xSpeed + " y=" + ySpeed + " rot=" + rot);




        SwerveConstants.m_frontLeft.setState(moduleStates[0], true);
        SwerveConstants.m_frontRight.setState(moduleStates[1], true);
        SwerveConstants.m_backLeft.setState(moduleStates[2], true);
        SwerveConstants.m_backRight.setState(moduleStates[3], true);
    }


    /**
     * Updates the field relative position of the robot.
     */
    public void updateOdometry() {
        SwerveConstants.m_odometry.update(
                SwerveConstants.m_gyro.getRotation2d(),
                new SwerveModulePosition[]{
                        SwerveConstants.m_frontLeft.getPosition(),
                        SwerveConstants.m_frontRight.getPosition(),
                        SwerveConstants.m_backLeft.getPosition(),
                        SwerveConstants.m_backRight.getPosition()
                });
    }

    public static void stopDrive() {

    }

    @Override
    public void periodic() {
        super.periodic();
        System.out.println("Back Left Angle " + SwerveConstants.m_backLeft.getAngle().getDegrees());
    }
}