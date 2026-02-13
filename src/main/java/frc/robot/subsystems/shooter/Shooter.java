package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import static frc.robot.RobotContainer.SWERVE;

public class Shooter extends SubsystemBase {

    static final TalonFX rightShooterMotor = new TalonFX(20);
    static final TalonFX leftShooterMotor = new TalonFX(21);
    static final TalonFX feederMotor = new TalonFX(22);
    final Servo angleServo = new Servo(0);
    static final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);


    public Shooter() {
        leftShooterMotor.setControl(
                new Follower(rightShooterMotor.getDeviceID(), false)
        );
    }

    public static FunctionalCommand ShootCommand(AngularVelocity targetSpeed) {
        return new FunctionalCommand(

                () -> {
                    setShooterSpeed(
                            targetSpeed.in(Units.Rotations.per(Units.Second)));
                },

                () -> {
                    if (isAtSpeed(targetSpeed)) {
                        feedFuel();
                    } else {
                        holdFuel();
                    }
                },

                interrupted -> {
                    stopShooter();
                },

                () -> false,

                this
        );
    }


    public void setAngle(Angle angle) {
        angleServo.setAngle(angle.in(Units.Degrees));
    }

    public static void holdFuel() {
        feederMotor.set(ShooterConstants.MOTOR_HOLD_VALUE);
    }


    public static void feedFuel() {
        feederMotor.set(ShooterConstants.MOTOR_FEED_VALUE);
    }

    public static AngularVelocity getShooterSpeed() {
        return rightShooterMotor.getVelocity().getValue();
    }

    public static void setShooterSpeed(double targetRPS) {
        rightShooterMotor.setControl(
                velocityRequest.withVelocity(targetRPS)
        );
    }

    public static void stopShooter() {
        rightShooterMotor.stopMotor();
        leftShooterMotor.stopMotor();
        feederMotor.stopMotor();
    }

    public static boolean isAtSpeed(AngularVelocity targetShooterRPS) {

        AngularVelocity currentSpeed = getShooterSpeed();

        AngularVelocity error = currentSpeed.minus(targetShooterRPS);

        return error < ShooterConstants.SHOOTER_TOLERANCE_RPS;
    }
}

