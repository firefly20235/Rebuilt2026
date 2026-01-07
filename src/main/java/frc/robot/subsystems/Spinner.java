package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spinner extends SubsystemBase {
    final TalonFX motor = new TalonFX(3);
    public Spinner() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        motor.setNeutralMode(NeutralModeValue.Brake);

    }

    public void spinForward(){
        motor.set(0.5);
    }


    public void spinBackward(){
        motor.set(-0.5);
    }


    public void stop(){
        motor.stopMotor();
    }

    public Command getSpinForwardCommand(){
        return this.startEnd(this::spinForward, this::stop);
    }

    public Command getSpinBackwardsCommand(){
        return this.startEnd(this::spinBackward,this::stop);
    }
}

