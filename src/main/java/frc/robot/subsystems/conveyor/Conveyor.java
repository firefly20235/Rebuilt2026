package frc.robot.subsystems.conveyor;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Main;


public class Conveyor extends SubsystemBase {

    private final SparkMax MainConveyorMotor = new SparkMax(40, SparkLowLevel.MotorType.kBrushless);

    public Conveyor() {
    }

    private void setSpeeds() {
        MainConveyorMotor.set(ConveyorConstants.mainSpeed);
    }

    private void setOppositeSpeeds() {
        MainConveyorMotor.set(-ConveyorConstants.mainSpeed);
    }

    private void stop() {
        MainConveyorMotor.stopMotor();
    }

    public StartEndCommand runConveyorCommand() {
        return new StartEndCommand(
                () -> setSpeeds(),
                () -> {
                    stop();
                },
                this
        );
    }

    public StartEndCommand oppositeConveyorCommand() {
        return new StartEndCommand(
                () -> setOppositeSpeeds(),
                () -> stop(),
                this
        );
    }

}