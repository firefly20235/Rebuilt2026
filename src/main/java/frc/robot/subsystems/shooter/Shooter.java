package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    final TalonFX rightShooterMotor = new TalonFX(20);
    final TalonFX leftShooterMotor = new TalonFX(21);

    final TalonFX feederMotor = new TalonFX(22);

    final Servo angleServo = new Servo(0);

    final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);


    public Shooter() {
        leftShooterMotor.setControl(
                new Follower(rightShooterMotor.getDeviceID(), false)
        );
    }

    public void setAngle(Angle angle) {
        angleServo.setAngle(angle.in(Units.Degrees));
    }

    public void holdFuel() {
        feederMotor.set(ShooterConstants.MOTOR_HOLD_VALUE);
    }


    public void feedFuel() {
        feederMotor.set(ShooterConstants.MOTOR_FEED_VALUE);
    }

    public Angle getShooterSpeed() {
        return rightShooterMotor.getVelocity().getValue().times(Units.Second.one());
    }

    public void setShooterSpeed(double targetRPS) {
        rightShooterMotor.setControl(
                velocityRequest.withVelocity(targetRPS)
        );
    }

    public void stopShooter() {
        rightShooterMotor.stopMotor();
        leftShooterMotor.stopMotor();
        feederMotor.stopMotor();
    }

//    public boolean isAtSpeed() {
//        double error = Math.abs(getShooterSpeed());
//
//        return error < ShooterConstants.SHOOTER_TOLERANCE_RPS
//                && getShooterSpeedRPS() > targetShooterRPS * 0.9;
//    }



    /*TODO:
    setShooterSpeed
    isAtSpeed
     */


}

