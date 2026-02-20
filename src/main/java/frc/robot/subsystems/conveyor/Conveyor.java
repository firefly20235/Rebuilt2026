package frc.robot.subsystems.conveyor;


import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Conveyor extends SubsystemBase {

    private final SparkMax conveyorMotor = new SparkMax(40, SparkLowLevel.MotorType.kBrushless);

    public Conveyor() {
    }

    private void setSpeed(LinearVelocity speed) {
        conveyorMotor.set(speed.in(Units.MetersPerSecond));
    }

    private void stop() {
        conveyorMotor.stopMotor();
    }

    public StartEndCommand runConveyorCommand(LinearVelocity speed) {
        return new StartEndCommand(
                () -> setSpeed(speed),
                () -> {
                    stop();
                },
                this
        );
    }

}

