package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    final TalonFX rightShooterMotor = new TalonFX(20);
    final TalonFX leftShooterMotor = new TalonFX(21);
    final TalonFX feederMotor = new TalonFX(22);

    private final Servo angleServo = new Servo(0);

    public Shooter() {

    }

    public void setAngle(double degrees) {
        angleServo.setAngle(degrees);
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

    public void stopShooter() {
        rightShooterMotor.stopMotor();
        leftShooterMotor.stopMotor();
        feederMotor.stopMotor();
    }


    /*TODO:
    setShooterSpeed
    atSpeed
     */




}

