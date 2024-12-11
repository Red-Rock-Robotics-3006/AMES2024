package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class Autos {
    private CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

    public Command testAuto1() {
        return swerve.getAuto("TestAuto1");
    }
}