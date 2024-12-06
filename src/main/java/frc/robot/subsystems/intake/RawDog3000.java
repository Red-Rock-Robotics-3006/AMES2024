package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils3006.SmartDashboardNumber;

public class RawDog3000 extends SubsystemBase {
    private static RawDog3000 instance = null;

    private final TalonFX m_slapLeft = new TalonFX(0); // change motor ID
    private final TalonFX m_slapRight = new TalonFX(0); // change motor ID
    private final TalonFX m_intake = new TalonFX(0); // change motor ID

    // not sure if what the difference is between slot 1, slot 2, slot 0?
    // might be like diff threads or sum
    private Slot1Configs slot1Configs = new Slot1Configs();
    private Slot2Configs slot2Configs = new Slot2Configs();

    private SmartDashboardNumber intakeKs = new SmartDashboardNumber("intake/ks", 0);
    private SmartDashboardNumber intakeKa = new SmartDashboardNumber("intake/ka", 0);
    private SmartDashboardNumber intakeKv = new SmartDashboardNumber("intake/kv", 0); // to be tuned;
    private SmartDashboardNumber intakeKp = new SmartDashboardNumber("intake/kp", 0);
    private SmartDashboardNumber intakeKi = new SmartDashboardNumber("intake/ki", 0);
    private SmartDashboardNumber intakeKd = new SmartDashboardNumber("intake/kd", 0);

    private SmartDashboardNumber pivotKs = new SmartDashboardNumber("pivot/ks", 0);
    private SmartDashboardNumber pivotKa = new SmartDashboardNumber("pivot/ka", 0);
    private SmartDashboardNumber pivotKv = new SmartDashboardNumber("pivot/kv", 0); // to be tuned;
    private SmartDashboardNumber pivotKp = new SmartDashboardNumber("pivot/kp", 0); // to be tuned;
    private SmartDashboardNumber pivotKi = new SmartDashboardNumber("pivot/ki", 0);
    private SmartDashboardNumber pivotKd = new SmartDashboardNumber("pivot/kd", 0);

    private RawDog3000() {
        super("RawDog3000");
    }

    public void setInitialStowSpeed() {
        setPivotMotorSpeeds(1000000); // change this hawk tuah
    }

    public void setInitialDeploySpeed() {
        setPivotMotorSpeeds(-1000000); // change this hawk tuah
    }

    public void setIntakeSpeed(double speed) {
    }

    public void enableIntake() {
    }

    public void disableIntake() {
    }

    public void setTarget(double target) {
    }

    public void setPivotMotorSpeeds(double speed) {
    }

    public boolean atCurrentSpike() {
        return false;
    }

    public static RawDog3000 getInstance() {
        if (RawDog3000.instance == null)
            RawDog3000.instance = new RawDog3000();
        return RawDog3000.instance;
    }
}