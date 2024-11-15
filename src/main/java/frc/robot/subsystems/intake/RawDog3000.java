package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class RawDog3000 extends SubsystemBase {
    private static RawDog3000 instance = null;

    private final TalonFX m_slapLeft = new TalonFX(0); // change motor ID
    private final TalonFX m_slapRight = new TalonFX(0); // change motor ID

    private RawDog3000() {
        super("RawDog3000");

        
    }

    private void initSlapUp() {
        double spikeThreshold = 320.0;

        while (this.m_slapLeft.getSupplyCurrent().getValue() < spikeThreshold && this.m_slapRight.getSupplyCurrent().getValue() < spikeThreshold) {
            // change based on testing
            this.m_slapLeft.set(-100000);
            this.m_slapRight.set(-100000);
        }
    }

    public void slapUp() {
        // m_slapLeft.
    }

    public static RawDog3000 getInstance() {
        if (RawDog3000.instance == null) RawDog3000.instance = new RawDog3000();
        return RawDog3000.instance;
    } 
}