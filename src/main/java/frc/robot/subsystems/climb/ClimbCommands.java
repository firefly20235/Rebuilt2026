package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;

public class ClimbCommands {

    public static Command raiseArms() {
        return new FunctionalCommand(
                () -> RobotContainer.CLIMB.initControllers(ClimbConstants.ClimbState.RAISED),
                RobotContainer.CLIMB::runMotors,
                (interrupted) -> RobotContainer.CLIMB.stop(),
                RobotContainer.CLIMB::isAtGoal,
                RobotContainer.CLIMB
        );
    }

    public static Command climb() {
        return new FunctionalCommand(
                () -> {},
                RobotContainer.CLIMB::runOpenLoop,
                (interrupted) -> RobotContainer.CLIMB.stop(),
                () -> false,
                RobotContainer.CLIMB
        );
    }

    public static Command resetEncoders(Climb climb) {
        return new InstantCommand(climb::resetEncoders, climb).ignoringDisable(true);
    }
}