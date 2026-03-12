// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.apriltagcamera.AprilTagCamera;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public static final Climb CLIMB = new Climb();

    public static final Intake INTAKE = new Intake();

    public static final Conveyor CONVEYOR = new Conveyor();

    public static final Shooter SHOOTER = new Shooter();

//    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(
//            new AprilTagCamera(
//                    AprilTagCameraConstants.AprilTagCameraType.LIMELIGHT,//Type of camera
//                    "TestLimelight",//Name of camera. Just how it logs it/refers to it in code. Doesn't need to match anything
//                    new Transform3d()//The offset of the camera from the center of the robot. For testing, it should be fine, but for a real robot it needs to know what to offset the position by to find the final robot's position.
//            )
//    );


    public static final Swerve SWERVE = new Swerve();


    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController =
            new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);

    private final CommandXboxController operatorController =
            new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);

    public RobotContainer() {
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {


        operatorController.rightBumper().whileTrue(CONVEYOR.runConveyorCommand());
//        operatorController.y().onTrue(SHOOTER.ShootCommand());


        SWERVE.setDefaultCommand(Swerve.getDriveCommand(
                        () -> -driverController.getLeftY() * SwerveConstants.MAX_VELOCITY.in(Units.MetersPerSecond) / 2.5,
                        () -> -driverController.getLeftX() * SwerveConstants.MAX_VELOCITY.in(Units.MetersPerSecond) / 2.5,
                        () -> -driverController.getRightX() * SwerveConstants.MAX_ANGULAR_VELOCITY.in(Units.RadiansPerSecond) / 2,
                        true
                )
        );

    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
//    public Command getAutonomousCommand() {
//        // An example command will be run in autonomous
//        return Autos.exampleAuto(exampleSubsystem);
//    }


}
