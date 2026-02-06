package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.AnalogGyro;

import static com.ctre.phoenix6.signals.SensorDirectionValue.CounterClockwise_Positive;

 public class SwerveConstants {
    //Steer motor PID constants
     public static final double ANGLE_KP = 0;
     public static final double ANGLE_KI = 0;
     public static final double ANGLE_KD = 0;

     //sensor config
     public static final SensorDirectionValue CANCODER_INVERSION = CounterClockwise_Positive;

     /*
     TODO:
     1) convert constants to use WPIlib units system
     2) tune steer motor pid
     3) calibrate offsets
      */

    //conversion factors
    public static final LinearVelocity MAX_VELOCITY = Units.Meters.of(4).per(Units.Seconds);
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = Units.RotationsPerSecond.of(3);
    public static final double STEER_GEAR_RATIO = 7/150.0;
    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double WHEEL_CIRCUMFERENCE_METERS = 0.1016;
    public static final double DRIVE_KS = 0.2;
    public static final double DRIVE_KV = 1.8;
    public static final double DRIVE_KA = 0.2;
     static final Distance kWheelDiameter = Units.Meter.of(5.08/100);


     static final boolean isFieldRelative = true;
     static final Per<DistanceUnit, AngleUnit> kRotationToDistance = kWheelDiameter.times(Math.PI).per(Units.Rotation);

    private static final Translation2d m_frontLeftLocation = new Translation2d(0.29, 0.29);
    private static final Translation2d m_frontRightLocation = new Translation2d(0.29, -0.29);
    private static final Translation2d m_backLeftLocation = new Translation2d(-0.29, 0.29);
    private static final Translation2d m_backRightLocation = new Translation2d(-0.29, -0.29);




     static final SwerveModule m_frontRight = new SwerveModule(1, 2, 1, 10, -0.282227*360);
     static final SwerveModule m_frontLeft = new SwerveModule(2, 4, 3, 11, 0.553955*360);
     static final SwerveModule m_backLeft = new SwerveModule(3, 6, 5, 12, -0.330322*360);
     static final SwerveModule m_backRight = new SwerveModule(4, 8, 7, 13, 0.257324*360);

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
