package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils3006.SmartDashboardNumber;

public class Shooter extends SubsystemBase{
    private static Shooter instance = null;

    private TalonFX m_shooterMotor = new TalonFX(39);
    private TalonFX m_hoodMotor = new TalonFX(38);

    private Slot0Configs slot0Configs = new Slot0Configs();

    private SmartDashboardNumber shooterKs = new SmartDashboardNumber("shooter/ks", 0);
    private SmartDashboardNumber shooterKa = new SmartDashboardNumber("shooter/ka", 0);
    private SmartDashboardNumber shooterKv = new SmartDashboardNumber("shooter/kv", 0); //to be tuned;
    private SmartDashboardNumber shooterKp = new SmartDashboardNumber("shooter/kp", 0); //to be tuned;
    private SmartDashboardNumber shooterKi = new SmartDashboardNumber("shooter/ki", 0);
    private SmartDashboardNumber shooterKd = new SmartDashboardNumber("shooter/kd", 0);

    private double targetHoodAngle;
    private double targetVelocityRPM;

    private Shooter() {
        super("Shooter");

        this.m_shooterMotor.getConfigurator().apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withPeakForwardDutyCycle(1d)
                .withPeakReverseDutyCycle(-1d)
                .withNeutralMode(NeutralModeValue.Brake)
        );

        this.m_shooterMotor.getConfigurator().apply(slot0Configs);
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
