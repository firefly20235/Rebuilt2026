package frc.robot.subsystems.swerve;

import com.revrobotics.spark.config.ClosedLoopConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.AnalogGyro;

 class SwerveConstants {
     static final double kMaxSpeed = 3.0; // 3 meters per second
     static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
     static final Distance kWheelDiameter = Units.Meter.of(5.08/100);
     static final Per<DistanceUnit, AngleUnit> kRotationToDistance = kWheelDiameter.times(Math.PI).per(Units.Rotation);

    private static final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    private static final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    private static final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    private static final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

     static final SwerveModule m_frontRight = new SwerveModule(3, 4, 4, 5, 6, 7);
     static final SwerveModule m_frontLeft = new SwerveModule(1, 2, 0, 1, 2, 3);
     static final SwerveModule m_backLeft = new SwerveModule(5, 6, 8, 9, 10, 11);
     static final SwerveModule m_backRight = new SwerveModule(7, 8, 12, 13, 14, 15);

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
