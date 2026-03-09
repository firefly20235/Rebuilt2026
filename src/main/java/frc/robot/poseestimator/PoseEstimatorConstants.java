package frc.robot.poseestimator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class PoseEstimatorConstants {
    static final double GYRO_UPDATE_TIME_SECONDS = 0.020;
    static final Vector<N3> ODOMETRY_AMBIGUITY = VecBuilder.fill(0.005, 0.005, 0.00005);
}

