package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class IntakeConstants {

    public enum IntakeState {
        RETRACTED(0.0),
        EXTENDED(0.0); // TODO: tune this

        double position;
        IntakeState(double position) {
            this.position = position;
        }
    }

    public static final int ARM_MOTOR_ID = 50;
    public static final int ROLLER_MOTOR_ID = 51;

    static final SparkMax armMotor = new SparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
    static final TalonFX rollerMotor = new TalonFX(ROLLER_MOTOR_ID);

    static final ProfiledPIDController controller = new ProfiledPIDController(
            1.0, 0.0, 0.0, // TODO: tune PID
            new TrapezoidProfile.Constraints(1, 1) // TODO: tune constraints
    );

    public static final double GEAR_RATIO = 5.0;
    public static final double ENCODER_TO_METERS = 1.0 / GEAR_RATIO; // TODO: adjust conversion

}