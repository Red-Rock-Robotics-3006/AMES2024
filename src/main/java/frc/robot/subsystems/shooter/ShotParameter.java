package frc.robot.subsystems.shooter;

public class ShotParameter {
    public final double pivotAngleDeg;
    public final double rpm;

    public ShotParameter(
        double pivotAngleDeg, double rpm) {
        this.pivotAngleDeg = pivotAngleDeg;
        this.rpm = rpm;
    }


    public boolean equals(ShotParameter other) {
        return Math.abs(other.pivotAngleDeg - pivotAngleDeg) < 0.1
            && Math.abs(other.rpm - rpm) < 0.1;
    }

    public ShotParameter interpolate(ShotParameter end, double t) {
        return new ShotParameter(
            lerp(pivotAngleDeg, end.pivotAngleDeg, t),
            lerp(rpm, end.rpm, t));
    }

    private double lerp(double y1, double y2, double t) {
        return y1 + (t * (y2 - y1));
    }
}
