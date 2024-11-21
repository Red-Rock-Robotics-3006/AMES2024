package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private static Shooter instance = null;

    private TalonFX m_shooterMotor = new TalonFX(39);
    private TalonFX m_hoodMotor = new TalonFX(38);

    private double targetHoodAngle;
    private double targetVelocityRPM;

    private Shooter() {
        super("Shooter");
    }

    public void setHoodAngle(double angle) {

    }

    public void setVelocity(double velocity) {
        
    }

    public static Shooter getInstance() {
        if (instance == null) instance = new Shooter();
        return instance;
    }
}
