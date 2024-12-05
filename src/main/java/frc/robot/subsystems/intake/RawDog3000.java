package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class RawDog3000 extends SubsystemBase {
    private static RawDog3000 instance = null;

    private final TalonFX m_slapLeft = new TalonFX(0); // change motor ID
    private final TalonFX m_slapRight = new TalonFX(0); // change motor ID
    private final TalonFX m_intake = new TalonFX(0); // change motor ID
    private double target;

    private RawDog3000() {
        super("RawDog3000");

        
    }

    public void setInitialStowSpeed() {
        setPivotMotorSpeeds(1000000); // change this hawk tuah
    }

    public void setInitialDeploySpeed() {
        setPivotMotorSpeeds(-1000000); // change this hawk tuah
    }

    public void setIntakeSpeed(double speed) {}

    public void enableIntake() {}

    public void disableIntake() {}

    public void setTarget(double target) {}

    public void setPivotMotorSpeeds(double speed) {}

    public boolean atCurrentSpike() {
        return false;
    }

    public static RawDog3000 getInstance() {
        if (RawDog3000.instance == null) RawDog3000.instance = new RawDog3000();
        return RawDog3000.instance;
    } 
}