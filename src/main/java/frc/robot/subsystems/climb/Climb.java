package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbConstants.ClimbState;

@Logged
public class Climb extends SubsystemBase {

    public Climb() {
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.encoder.positionConversionFactor(ClimbConstants.ENCODER_TO_RADIANS);
        leftConfig.encoder.velocityConversionFactor(ClimbConstants.ENCODER_TO_RADIANS / 60);
        ClimbConstants.leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.encoder.positionConversionFactor(ClimbConstants.ENCODER_TO_RADIANS);
        rightConfig.encoder.velocityConversionFactor(ClimbConstants.ENCODER_TO_RADIANS / 60);
        rightConfig.inverted(ClimbConstants.RIGHT_MOTOR_INVERTED);
        ClimbConstants.rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        resetEncoders();
    }

    public void initControllers(ClimbState state) {
        ClimbConstants.leftController.setGoal(state.position);
        ClimbConstants.leftController.reset(getLeftPosition(), getLeftVelocity());

        ClimbConstants.rightController.setGoal(state.position);
        ClimbConstants.rightController.reset(getRightPosition(), getRightVelocity());
    }

    public void runMotors() {
        double leftOutput = ClimbConstants.leftController.calculate(getLeftPosition());
        double rightOutput = ClimbConstants.rightController.calculate(getRightPosition());

        ClimbConstants.leftMotor.setVoltage(leftOutput);
        ClimbConstants.rightMotor.setVoltage(rightOutput);
    }

    public void runOpenLoop() {
        ClimbConstants.leftMotor.setVoltage(ClimbConstants.CLIMB_VOLTAGE);
        ClimbConstants.rightMotor.setVoltage(ClimbConstants.CLIMB_VOLTAGE);
    }

    public void stop() {
        ClimbConstants.leftMotor.set(0);
        ClimbConstants.rightMotor.set(0);
    }

    public double getLeftPosition() {
        return ClimbConstants.leftMotor.getEncoder().getPosition();
    }

    public double getRightPosition() {
        return ClimbConstants.rightMotor.getEncoder().getPosition();
    }

    public double getLeftVelocity() {
        return ClimbConstants.leftMotor.getEncoder().getVelocity();
    }

    public double getRightVelocity() {
        return ClimbConstants.rightMotor.getEncoder().getVelocity();
    }

    public void resetEncoders() {
        ClimbConstants.leftMotor.getEncoder().setPosition(0);
        ClimbConstants.rightMotor.getEncoder().setPosition(0);
    }

    public boolean isAtGoal() {
        return ClimbConstants.leftController.atGoal() && ClimbConstants.rightController.atGoal();
    }
}