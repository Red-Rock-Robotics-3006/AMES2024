package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils3006.SmartDashboardNumber;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class Localization {
    public static final Pose2d redCliffPose = new Pose2d();
    public static final Pose2d blueCliffPose = new Pose2d();

    private static int[] validIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
    private static String[] limeLightNames = {"limelight-left", "limelight-right"};
    private static double[][] limeLightStdvs = {
        {0.8, 0.8, 9999},
        {0.8, 0.8, 9999}
    };

    private static LimeLightPoseEstimateWrapper[] wrappers;

    private static SmartDashboardNumber kStdvDemoninator = new SmartDashboardNumber("localization/stdv-denom-scale", 30);

    public static void initialize() {
        wrappers = new LimeLightPoseEstimateWrapper[limeLightNames.length];
        for (int i = 0; i < limeLightNames.length; i++) {
            wrappers[i] = new LimeLightPoseEstimateWrapper().withName(limeLightNames[i]);
            LimelightHelpers.SetFiducialIDFiltersOverride(limeLightNames[i], validIDs);
        }
    }

    public static LimeLightPoseEstimateWrapper[] getPoseEstimates(double headingDegrees) {
        int i = 0;
        for (String s : limeLightNames) {
            LimelightHelpers.SetRobotOrientation(s, headingDegrees, CommandSwerveDrivetrain.getInstance().getRotationRateDegrees(), 0, 0, 0, 0);
            wrappers[i].withPoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(s))
                        .withTagInVision(LimelightHelpers.getTV(s));

        }
        return wrappers;
    }

    public static double getDistanceToTargetRed() {
        return Math.hypot(getPose2d().getX() - redCliffPose.getX(), getPose2d().getY() - redCliffPose.getY());
    }

    public static double getDistanceToTargetBlue() {
        return Math.hypot(getPose2d().getX() - blueCliffPose.getX(), getPose2d().getY() - blueCliffPose.getY());
    }

    public static Pose2d getPose2d() {
        return CommandSwerveDrivetrain.getInstance().getPose();
    }

    //TODO
    public static Rotation2d getAngleToRed() {
        return Rotation2d.fromRadians(
            Math.atan2(redCliffPose.getY() - getPose2d().getY(), redCliffPose.getX() - getPose2d().getX())
        );
    }

    //TODO
    public static Rotation2d getAngleToBlue() {
        return Rotation2d.fromRadians(
            Math.atan2(blueCliffPose.getY() - getPose2d().getY(), blueCliffPose.getX() - getPose2d().getX())
        );
    }

    public static class LimeLightPoseEstimateWrapper {
        public LimelightHelpers.PoseEstimate poseEstimate;
        public String name;
        public boolean tiv;
        private SmartDashboardNumber[] kStdvs;
        public Field2d field = new Field2d();

        public Matrix<N3, N1> getStdvs(double distanceToTarget) {
            return VecBuilder.fill(
                adjustStdv(kStdvs[0].getNumber(), distanceToTarget),
                adjustStdv(kStdvs[1].getNumber(), distanceToTarget),
                adjustStdv(kStdvs[2].getNumber(), distanceToTarget)
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

            kStdvs[0] = new SmartDashboardNumber(this.name + "/" + this.name + "-stdvX", stdvDefVals[0]);
            kStdvs[1] = new SmartDashboardNumber(this.name + "/" + this.name + "-stdvY", stdvDefVals[1]);
            kStdvs[2] = new SmartDashboardNumber(this.name + "/" + this.name + "-stdvTheta", stdvDefVals[2]);

            SmartDashboard.putData(this.name + "/" + this.name + "field", this.field);

            return this;
        }

        public LimeLightPoseEstimateWrapper withPoseEstimate(LimelightHelpers.PoseEstimate estimate) {
            this.poseEstimate = estimate;
            return this;
        }

        public LimeLightPoseEstimateWrapper withTagInVision(boolean b) {
            this.tiv = b;
            SmartDashboard.putBoolean(this.name + "/" + this.name + "-tag-in-vision", b);
            return this;
        }

        private double adjustStdv(double stdv, double distanceToTarget) {
            return stdv + stdv * (distanceToTarget * distanceToTarget) / Localization.kStdvDemoninator.getNumber();
        }
    }
}
