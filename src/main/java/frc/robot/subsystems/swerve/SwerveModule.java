package frc.robot.subsystems.swerve;


import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
import static com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

public class SwerveModule {
    public final int moduleNumber;

    private final TalonFX driveMotor;
//    private final SimpleMotorFeedforward driveFeedforward;

    private final SparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController anglePID;

    private final CANcoder canCoder;
    private final double canCoderOffsetDegrees;

    private Angle lastAngle;

    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int canCoderID, double canCoderOffsetDegrees) {
        this.moduleNumber = moduleNumber;

        driveMotor = new TalonFX(driveMotorID);
        SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.DRIVE_KS, SwerveConstants.DRIVE_KV, SwerveConstants.DRIVE_KA);

        angleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getClosedLoopController();

        canCoder = new CANcoder(canCoderID);
        this.canCoderOffsetDegrees = canCoderOffsetDegrees;

        configureDevices();
        lastAngle = getState().angle.getMeasure();
    }

    public void setState(SwerveModuleState state, boolean isOpenLoop) {
        System.out.println(
                "setState angle=" +
                        state.angle.getDegrees() +
                        "speed=" +
                        state.speedMetersPerSecond
        );


        state.optimize(getState().angle);

        if (isOpenLoop) {
            double percentOutput =
                    state.speedMetersPerSecond
                            / SwerveConstants.MAX_VELOCITY.in(Units.MetersPerSecond);

            System.out.println("percentOutput=" + percentOutput);

            driveMotor.set(percentOutput);
        }



//        } else {
//
//            double wheelRPS =
//                    state.speedMetersPerSecond
//                            / SwerveConstants.WHEEL_CIRCUMFERENCE_METERS;
//
//            double motorRPS =
//                    wheelRPS * SwerveConstants.DRIVE_GEAR_RATIO;
//
//            driveMotor.setControl(
//                    new VelocityVoltage(motorRPS)
//                            .withFeedForward(
//                                    driveFeedforward.calculate(state.speedMetersPerSecond)
//                            )
//            );
//        }


        Angle angle = Math.abs(state.speedMetersPerSecond) <= SwerveConstants.MAX_VELOCITY.in(Units.MetersPerSecond) * 0.01
                ? lastAngle
                : state.angle.getMeasure();

        anglePID.setReference(angle.in(Units.Rotations), SparkMax.ControlType.kPosition);
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

        Angle angle = driveMotor.getPosition().getValue();
        var distance = SwerveConstants.kRotationToDistance.timesDivisor(angle);
        Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
        return new SwerveModulePosition((Distance) distance, rot);
    }

    private void configureDevices() {
        // Drive motor configuration.

        TalonFXConfiguration driveMotorConfigs = new TalonFXConfiguration().withMotorOutput(
                        new MotorOutputConfigs()
                                .withNeutralMode(NeutralModeValue.Brake))
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(Clockwise_Positive)
                );

        driveMotor.setPosition(0);


        // Angle motor configuration.
        SparkMaxConfig angleConfig = new SparkMaxConfig();

        angleConfig
                .inverted(true)
                .idleMode(SparkBaseConfig.IdleMode.kBrake);

        angleConfig.encoder
                .positionConversionFactor(SwerveConstants.STEER_GEAR_RATIO);
//                .velocityConversionFactor(SwerveConstants.ANGLE_RPM_TO_RADIANS_PER_SECOND);

        angleConfig.closedLoop
                .p(SwerveConstants.ANGLE_KP)
                .i(SwerveConstants.ANGLE_KI)
                .d(SwerveConstants.ANGLE_KD)
                .positionWrappingEnabled(true)
                .positionWrappingMinInput(0)
                .positionWrappingMaxInput(1);

        angleMotor.configure(
                angleConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        );



        // CanCoder configuration.
        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.SensorDirection = SwerveConstants.CANCODER_INVERSION;

        canCoder.getConfigurator().apply(canCoderConfiguration);
    }
}
