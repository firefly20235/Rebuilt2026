package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ClimbConstants {

    public enum ClimbState {
        STOWED(0.0),
        RAISED(0.0); // TODO: tune this

        double position;
        ClimbState(double position) {
            this.position = position;
        }
    }

    public static final int LEFT_MOTOR_ID = 20;
    public static final int RIGHT_MOTOR_ID = 21;

    static final SparkMax leftMotor = new SparkMax(LEFT_MOTOR_ID, MotorType.kBrushless);
    static final SparkMax rightMotor = new SparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);

    // TODO: check if right motor needs to be inverted
    public static final boolean RIGHT_MOTOR_INVERTED = false;

    static final ProfiledPIDController leftController = new ProfiledPIDController(
            1.0, 0.0, 0.0, // TODO: tune PID
            new TrapezoidProfile.Constraints(1, 1) // TODO: tune constraints
    );

    static final ProfiledPIDController rightController = new ProfiledPIDController(
            1.0, 0.0, 0.0, // TODO: tune PID
            new TrapezoidProfile.Constraints(1, 1) // TODO: tune constraints
    );

    public static final double GEAR_RATIO = 45.0;
    public static final double ENCODER_TO_RADIANS = (2 * Math.PI) / GEAR_RATIO;

    public static final double CLIMB_VOLTAGE = 6.0; // TODO: tune open loop voltage for climbing
}