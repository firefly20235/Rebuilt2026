// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.revrobotics.spark.config.ClosedLoopConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import frc.robot.Constants;

/** Represents a swerve drive style drivetrain. */
public class Swerve {
    public Swerve() {
        SwerveConstants.m_gyro.reset();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(
            double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates =
                SwerveConstants.m_kinematics.toSwerveModuleStates(
                        ChassisSpeeds.discretize(
                                fieldRelative
                                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        xSpeed, ySpeed, rot, SwerveConstants.m_gyro.getRotation2d())
                                        : new ChassisSpeeds(xSpeed, ySpeed, rot),
                                Constants.LOOP_PERIOD_SECONDS));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_VELOCITY.in(Units.MetersPerSecond));
        SwerveConstants.m_frontLeft.setState(swerveModuleStates[0],false);
        SwerveConstants.m_frontRight.setState(swerveModuleStates[1],false);
        SwerveConstants.m_backLeft.setState(swerveModuleStates[2],false);
        SwerveConstants.m_backRight.setState(swerveModuleStates[3],false);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        SwerveConstants.m_odometry.update(
                SwerveConstants.m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        SwerveConstants.m_frontLeft.getPosition(),
                        SwerveConstants.m_frontRight.getPosition(),
                        SwerveConstants. m_backLeft.getPosition(),
                        SwerveConstants.m_backRight.getPosition()
                });
    }
}