package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public class SwerveModule {
    public final int moduleNumber;

    private final TalonFX driveMotor;
//    private final SimpleMotorFeedforward driveFeedforward;

    private final SparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController anglePID;

    private final CANcoder canCoder;
    private final double canCoderOffsetDegrees;

    private double lastAngle;

    public SwerveModule(int moduleNumber,int driveMotorID, int angleMotorID, int canCoderID, double canCoderOffsetDegrees ) {
        this.moduleNumber = moduleNumber;

        driveMotor = new TalonFX(driveMotorID);
//        driveFeedforward = new SimpleMotorFeedforward(kSwerve.DRIVE_KS, kSwerve.DRIVE_KV, kSwerve.DRIVE_KA);

        angleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getClosedLoopController();

        canCoder = new CANcoder(canCoderID);
        this.canCoderOffsetDegrees = canCoderOffsetDegrees;

        configureDevices();
        lastAngle = getState().angle.getRadians();
    }

    public void setState(SwerveModuleState state, boolean isOpenLoop) {
        // Prevents angle motor from turning further than it needs to.
        // E.G. rotating from 10 to 270 degrees CW vs CCW.
        state = SwerveModuleState.optimize(state, getState().angle);

        if (isOpenLoop) {
            double speed = state.speedMetersPerSecond / kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
            drivePID.setReference(speed, CANSparkMax.ControlType.kDutyCycle);
        } else {
            drivePID.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, 0, driveFeedforward.calculate(state.speedMetersPerSecond));
        }

        double angle = Math.abs(state.speedMetersPerSecond) <= kSwerve.MAX_VELOCITY_METERS_PER_SECOND * 0.01
                ? lastAngle
                : state.angle.getRadians();

        anglePID.setReference(angle, CANSparkMax.ControlType.kPosition);
        lastAngle = angle;
    }

    public SwerveModuleState getState() {
        Angle anglePerSecond = driveMotor.getVelocity().getValue().times(Units.Second.one());
        var distancePerSecond = SwerveConstants.kRotationToDistance.timesDivisor(anglePerSecond);

        Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
        return new SwerveModuleState(distancePerSecond.in(Units.Meter), rot);
    }

    public Angle getCanCoder() {
        return canCoder.getAbsolutePosition().getValue();
    }

    public Rotation2d getAngle() {
        return new Rotation2d(angleEncoder.getPosition());
    }

    public SwerveModulePosition getPosition() {
        double distance = driveEncoder.getPosition();
        Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
        return new SwerveModulePosition(distance, rot);
    }

    private void configureDevices() {
        // Drive motor configuration.
        driveMotor.restoreFactoryDefaults();
        driveMotor.setInverted(kSwerve.DRIVE_MOTOR_INVERSION);
        driveMotor.setIdleMode(kSwerve.DRIVE_IDLE_MODE);
        driveMotor.setOpenLoopRampRate(kSwerve.OPEN_LOOP_RAMP);
        driveMotor.setClosedLoopRampRate(kSwerve.CLOSED_LOOP_RAMP);
        driveMotor.setSmartCurrentLimit(kSwerve.DRIVE_CURRENT_LIMIT);

        drivePID.setP(kSwerve.DRIVE_KP);
        drivePID.setI(kSwerve.DRIVE_KI);
        drivePID.setD(kSwerve.DRIVE_KD);
        drivePID.setFF(kSwerve.DRIVE_KF);

        driveEncoder.setPositionConversionFactor(kSwerve.DRIVE_ROTATIONS_TO_METERS);
        driveEncoder.setVelocityConversionFactor(kSwerve.DRIVE_RPM_TO_METERS_PER_SECOND);
        driveEncoder.setPosition(0);

        // Angle motor configuration.
        angleMotor.restoreFactoryDefaults();
        angleMotor.setInverted(kSwerve.ANGLE_MOTOR_INVERSION);
        angleMotor.setIdleMode(kSwerve.ANGLE_IDLE_MODE);
        angleMotor.setSmartCurrentLimit(kSwerve.ANGLE_CURRENT_LIMIT);

        anglePID.setP(kSwerve.ANGLE_KP);
        anglePID.setI(kSwerve.ANGLE_KI);
        anglePID.setD(kSwerve.ANGLE_KD);
        anglePID.setFF(kSwerve.ANGLE_KF);

        anglePID.setPositionPIDWrappingEnabled(true);
        anglePID.setPositionPIDWrappingMaxInput(2 * Math.PI);
        anglePID.setPositionPIDWrappingMinInput(0);

        angleEncoder.setPositionConversionFactor(kSwerve.ANGLE_ROTATIONS_TO_RADIANS);
        angleEncoder.setVelocityConversionFactor(kSwerve.ANGLE_RPM_TO_RADIANS_PER_SECOND);
        angleEncoder.setPosition(Units.degreesToRadians(getCanCoder() - canCoderOffsetDegrees));

        // CanCoder configuration.
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfiguration.sensorDirection = kSwerve.CANCODER_INVERSION;
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;

        canCoder.configFactoryDefault();
        canCoder.configAllSettings(canCoderConfiguration);
    }
}
