//package frc.robot.subsystems.intake;
//
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.FunctionalCommand;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import frc.robot.RobotContainer;
//
//public class IntakeCommands {
//
//    public static Command extend() {
//        return new FunctionalCommand(
//                () -> RobotContainer.INTAKE.initController(IntakeConstants.IntakeState.EXTENDED),
//                () -> {
//                    RobotContainer.INTAKE.runArm();
//                    RobotContainer.INTAKE.runRoller(6.0); // TODO: tune voltage
//                },
//                (interrupted) -> {
//                    RobotContainer.INTAKE.stopArm();
//                    RobotContainer.INTAKE.stopRoller();
//                },
//                () -> false, // driver controls when to retract
//                RobotContainer.INTAKE
//        );
//    }
//
//    public static Command retract() {
//        return new FunctionalCommand(
//                () -> RobotContainer.INTAKE.initController(IntakeConstants.IntakeState.RETRACTED),
//                RobotContainer.INTAKE::runArm,
//                (interrupted) -> RobotContainer.INTAKE.stopArm(),
//                RobotContainer.INTAKE::isAtGoal,
//                RobotContainer.INTAKE
//        );
//    }
//
//    public static Command resetEncoder(Intake intake) {
//        return new InstantCommand(intake::resetEncoder, intake).ignoringDisable(true);
//    }
//}