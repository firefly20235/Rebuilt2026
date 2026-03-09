package frc.robot.poseestimator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.apriltagcamera.AprilTagCamera;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.swerve.SwerveConstants;

public class PoseEstimator {
    private final Field2d field = new Field2d();
    private final AprilTagCamera[] aprilTagCameras;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator = createSwerveDrivePoseEstimator();
    private final SwerveDriveOdometry swerveDriveOdometry = createSwerveDriveOdometry();


    public PoseEstimator(AprilTagCamera... aprilTagCameras) {
        this.aprilTagCameras = aprilTagCameras;

        putAprilTagsOnFieldWidget();
        SmartDashboard.putData("Field",field);
    }

    public void updatePeriodically() {
        for (AprilTagCamera aprilTagCamera : aprilTagCameras) {
            aprilTagCamera.updatePeriodically();

            if (aprilTagCamera.hasValidTarget())
                swerveDrivePoseEstimator.addVisionMeasurement(
                        aprilTagCamera.getEstimatedRobotPose(),
                        aprilTagCamera.getLatestResultTimestamp(),
                        aprilTagCamera.calculateStandardDeviations()
                );
        }
        swerveDrivePoseEstimator.update(RobotContainer.SWERVE.getHeading(), RobotContainer.SWERVE.getSwerveModulePositions());
//TODO: Implement odometry update
        field.setRobotPose(getEstimatedRobotPosition());
    }

    public void resetHeading(Rotation2d newHeading) {
        swerveDrivePoseEstimator.resetRotation(newHeading);
        swerveDriveOdometry.resetRotation(newHeading);
//        RobotContainer.SWERVE.setHeading(newHeading);
        //TODO: Update gyro
    }

    public Pose2d getEstimatedRobotPosition() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }
    public Pose2d getRobotOdometryPosition(){
        return swerveDriveOdometry.getPoseMeters();
    }

    private void putAprilTagsOnFieldWidget(){
        for (int i = 1; i< FieldConstants.TAG_ID_TO_POSE.size() + 1; i++){
            final Pose2d tagPose = FieldConstants.TAG_ID_TO_POSE.get(i).toPose2d();
            field.getObject("Tag " + i).setPose(tagPose);
        }
    }


    private SwerveDrivePoseEstimator createSwerveDrivePoseEstimator() {
        final SwerveModulePosition[] swerveModulePositions = {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };
        return new SwerveDrivePoseEstimator(
                SwerveConstants.m_kinematics,
                new Rotation2d(),
                swerveModulePositions,
                new Pose2d(),
                PoseEstimatorConstants.ODOMETRY_AMBIGUITY,
                VecBuilder.fill(0, 0, 0)
        );
    }

    private SwerveDriveOdometry createSwerveDriveOdometry() {
        final SwerveModulePosition[] swerveModulePositions = {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };
        return new SwerveDriveOdometry(
                SwerveConstants.m_kinematics,
                new Rotation2d(),
                swerveModulePositions
        );

    }

}
