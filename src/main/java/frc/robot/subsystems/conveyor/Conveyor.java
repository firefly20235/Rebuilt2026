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
package frc.robot.subsystems.conveyor;// package frc.robot.subsystems.conveyor;


// import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.spark.SparkLowLevel;
// import com.revrobotics.spark.SparkMax;
// import edu.wpi.first.units.measure.Units;
// import edu.wpi.first.units.measure.LinearVelocity;
// import edu.wpi.first.wpilibj2.command.StartEndCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;


// public class Conveyor extends SubsystemBase {

//     private final SparkMax MainConveyorMotor = new SparkMax(40, SparkLowLevel.MotorType.kBrushless);
//     private final TalonFX SideConveyorMotor = new TalonFX(41);

//     public Conveyor() {
//     }

//     private void setSpeeds(LinearVelocity mainSpeed, LinearVelocity sideSpeed) {
//         MainConveyorMotor.set(mainSpeed.in(Units.MetersPerSecond));
//         SideConveyorMotor.set(sideSpeed.in(Units.MetersPerSecond));
//     }

//     private void stop() {
//         MainConveyorMotor.stopMotor();
//         SideConveyorMotor.stopMotor();
//     }

//     public StartEndCommand runConveyorCommand(LinearVelocity mainSpeed, LinearVelocity sideSpeed) {
//         return new StartEndCommand(
//                 () -> setSpeeds(mainSpeed,sideSpeed),
//                 () -> {
//                     stop();
//                 },
//                 this
//         );
//     }

// }

