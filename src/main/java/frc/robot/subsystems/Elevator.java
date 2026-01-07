package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class Elevator extends SubsystemBase {

    private final TalonFX motor;
    private final CANcoder encoder;
    private final PIDController pidController;

    private final DigitalInput limitSwitch;

    public Elevator(TalonFX motor, CANcoder encoder, DigitalInput limitSwitch) {
        this.encoder = encoder;
        this.motor = motor;
        this.limitSwitch = limitSwitch;
        pidController = new PIDController(0, 0, 0);

        new Trigger(limitSwitch::get).onTrue(getResetEncoderCommand());


        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        setDefaultCommand(getBottomCommand());
    }

    public Command getTopCommand() {
        return getToHeight(ElevatorConstants.Heights.TOP);
    }

    public Command getMiddleCommand() {
        return getToHeight(ElevatorConstants.Heights.MIDDLE);
    }

    public Command getBottomCommand() {
        return getToHeight(ElevatorConstants.Heights.BOTTOM);
    }

    public Command getResetEncoderCommand() {
        return Commands.runOnce(() -> {
            encoder.setPosition(0);
        });
    }

    public Command getToHeight(ElevatorConstants.Heights height) {
        return new FunctionalCommand(() -> {
            pidController.reset();

            pidController.setSetpoint(height.height);
        }, () -> {
            motor.set(pidController.calculate(getHeight()));
        }, (Boolean interrupted) -> {
            motor.stopMotor();
        }, pidController::atSetpoint);
    }

    public double getHeight() {
        double rotationsPerMeter = 2;

        double rotations = encoder.getPosition().getValue().in(Units.Rotation);

        return rotations / rotationsPerMeter;
    }

}