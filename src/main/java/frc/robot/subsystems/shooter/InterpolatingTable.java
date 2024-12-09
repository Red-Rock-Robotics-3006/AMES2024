package frc.robot.subsystems.shooter;

import static java.util.Map.entry;

import edu.wpi.first.math.util.Units;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

public class InterpolatingTable {

    private InterpolatingTable() {}

    //TODO: these parameters are garbage
    public static TreeMap<Double, ShotParameter> redTable =
        new TreeMap<>(
            Map.ofEntries(
                entry(
                    Units.inchesToMeters(55.52922002054039),
                    new ShotParameter(45, 2800))
                ));
                
    //TODO: these parameters are garbage
    public static TreeMap<Double, ShotParameter> blueTable =
        new TreeMap<>(
            Map.ofEntries(
                entry(
                    Units.inchesToMeters(55.52922002054039),
                    new ShotParameter(45, 2800))
                ));

    // public static TreeMap<Double, ShotParameter> blueTable = redTable;

    public static ShotParameter getRed(double distanceToTarget) {
        Entry<Double, ShotParameter> ceil = redTable.ceilingEntry(distanceToTarget);
        Entry<Double, ShotParameter> floor = redTable.floorEntry(distanceToTarget);
        if (ceil == null) return floor.getValue();
        if (floor == null) return ceil.getValue();
        if (ceil.getValue().equals(floor.getValue())) return ceil.getValue();
        return floor
            .getValue()
            .interpolate(
                ceil.getValue(),
                (distanceToTarget - floor.getKey()) / (ceil.getKey() - floor.getKey()));
    }

    public static ShotParameter getBlue(double distanceToTarget) {
        Entry<Double, ShotParameter> ceil = blueTable.ceilingEntry(distanceToTarget);
        Entry<Double, ShotParameter> floor = blueTable.floorEntry(distanceToTarget);
        if (ceil == null) return floor.getValue();
        if (floor == null) return ceil.getValue();
        if (ceil.getValue().equals(floor.getValue())) return ceil.getValue();
        return floor
            .getValue()
            .interpolate(
                ceil.getValue(),
                (distanceToTarget - floor.getKey()) / (ceil.getKey() - floor.getKey()));
    }
}