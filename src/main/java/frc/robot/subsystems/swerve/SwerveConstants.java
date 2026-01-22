package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.ClosedLoopConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.AnalogGyro;

import static com.ctre.phoenix6.signals.SensorDirectionValue.CounterClockwise_Positive;

class SwerveConstants {
     public static final double ANGLE_KP = 0;
     public static final double ANGLE_KI = 0;
     public static final double ANGLE_KD = 0;
     public static final SensorDirectionValue CANCODER_INVERSION = CounterClockwise_Positive;
    public static final LinearVelocity MAX_VELOCITY = Units.Meters.of(4).per(Units.Seconds);
    public static final double ANGLE_ROTATIONS_TO_RADIANS = 0;//TODO
    public static final double ANGLE_RPM_TO_RADIANS_PER_SECOND = 0;//TODO
    public static final double DRIVE_GEAR_RATIO = 0;
    public static final double WHEEL_CIRCUMFERENCE_METERS = 0;
    public static final double DRIVE_KS = 0;
    public static final double DRIVE_KV = 0;
    public static final double DRIVE_KA = 0;
    static final double kMaxSpeed = 3.0; // 3 meters per second
     static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
     static final Distance kWheelDiameter = Units.Meter.of(5.08/100);


     static final boolean isFieldRelative = true;
     static final Per<DistanceUnit, AngleUnit> kRotationToDistance = kWheelDiameter.times(Math.PI).per(Units.Rotation);

    private static final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    private static final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    private static final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    private static final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);




     static final SwerveModule m_frontRight = new SwerveModule(1, 2, 1, 10, 6);
     static final SwerveModule m_frontLeft = new SwerveModule(2, 4, 3, 11, 2);
     static final SwerveModule m_backLeft = new SwerveModule(3, 6, 5, 12, 10);
     static final SwerveModule m_backRight = new SwerveModule(4, 8, 7, 13, 14);

     static final AnalogGyro m_gyro = new AnalogGyro(0);

     static final SwerveDriveKinematics m_kinematics =
            new SwerveDriveKinematics(
                    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

     static final SwerveDriveOdometry m_odometry =
            new SwerveDriveOdometry(
                    m_kinematics,
                    m_gyro.getRotation2d(),
                    new SwerveModulePosition[] {
                            m_frontLeft.getPosition(),
                            m_frontRight.getPosition(),
                            m_backLeft.getPosition(),
                            m_backRight.getPosition()
                    });

}
