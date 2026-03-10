package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;

@Logged
public class Intake extends SubsystemBase {

    public Intake() {
    }

    public void initController(IntakeState state) {
        IntakeConstants.controller.setGoal(state.position);
        IntakeConstants.controller.reset(getCurrentPosition(), getCurrentVelocity());
    }

    public void runArm() {
        double output = IntakeConstants.controller.calculate(getCurrentPosition());
        IntakeConstants.armMotor.setVoltage(output);
    }

    public void runRoller(double voltage) {
        IntakeConstants.rollerMotor.setVoltage(voltage);
    }

    public void stopRoller() {
        IntakeConstants.rollerMotor.set(0);
    }

    public void stopArm() {
        IntakeConstants.armMotor.set(0);
    }

    public double getCurrentPosition() {
        return IntakeConstants.armMotor.getEncoder().getPosition();
    }

    public double getCurrentVelocity() {
        return IntakeConstants.armMotor.getEncoder().getVelocity();
    }

    public boolean isAtGoal() {
        return IntakeConstants.controller.atGoal();
    }

    public void resetEncoder() {
        IntakeConstants.armMotor.getEncoder().setPosition(0);
    }
}