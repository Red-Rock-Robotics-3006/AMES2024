package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Utils3006.SmartDashboardNumber;

public class Localization {
    private static int[] validIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
    private static String[] limeLightNames = {"limelight-front", "limelight-back"};
    private static double[][] limeLightStdvs = {
        {0.8, 0.8, 9999},
        {0.8, 0.8, 9999}
    };

    private static LimeLightPoseEstimateWrapper[] wrappers;

    public static void initialize() {
        wrappers = new LimeLightPoseEstimateWrapper[limeLightNames.length];
        for (int i = 0; i < limeLightNames.length; i++) {
            wrappers[i] = new LimeLightPoseEstimateWrapper().withName(limeLightNames[i]);
        }
    }

    public static LimeLightPoseEstimateWrapper[] getPoseEstimates(double headingDegrees) {
        LimeLightPoseEstimateWrapper[] estimates = new LimeLightPoseEstimateWrapper[limeLightNames.length];
        int i = 0;
        for (String s : limeLightNames) {
            LimelightHelpers.SetRobotOrientation(s, headingDegrees, 0, 0, 0, 0, 0);
            estimates[i] = new LimeLightPoseEstimateWrapper();

        }
        return estimates;
    }
    //TODO
    public static double getDistanceToTargetRed() {
        return 0;
    }

    //TODO
    public static double getDistanceToTargetBlue() {
        return 0;
    }

    //TODO
    public static Pose2d getPose2d() {
        return new Pose2d();
    }

    //TODO
    public static Rotation2d getAngleToRed() {
        return new Rotation2d();
    }

    //TODO
    public static Rotation2d getAngleToBlue() {
        return new Rotation2d();
    }

    public static class LimeLightPoseEstimateWrapper {
        public LimelightHelpers.PoseEstimate poseEstimate;
        public String name;
        public double[] stdvs;
        private SmartDashboardNumber[] kStdvs;

        public Matrix<N3, N1> getStdvs() {
            return VecBuilder.fill(
                stdvs[0],
                stdvs[1],
                stdvs[2]
            );
        }

        public LimeLightPoseEstimateWrapper withName(String name) {
            this.name = name;
            double[] stdvDefVals = new double[] {0.8, 0.8, 9999};
            for (int i = 0; i < Localization.limeLightNames.length; i++) {
                if (Localization.limeLightNames[i].equals(name)) {
                    stdvDefVals = limeLightStdvs[i];
                    break;
                }
            }

            kStdvs[0] = new SmartDashboardNumber(this.name + "/stdvX", stdvDefVals[0]);
            kStdvs[1] = new SmartDashboardNumber(this.name + "/stdvY", stdvDefVals[1]);
            kStdvs[2] = new SmartDashboardNumber(this.name + "/stdvTheta", stdvDefVals[2]);
            return this;
        }

        public LimeLightPoseEstimateWrapper withPoseEstimate(LimelightHelpers.PoseEstimate estimate) {
            this.poseEstimate = estimate;
            return this;
        }
    }
}
