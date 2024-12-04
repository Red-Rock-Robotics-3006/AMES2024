package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils3006.SmartDashboardNumber;

public class Turret extends SubsystemBase{
    private static Turret instance = null;

    public static final double kMaxTurretRotation = 0,
                               kMaxTurretAngle = 0;
    public static final double kMinTurretRotation = 0,
                               kMinTurretAngle = 0;

    private TalonFX m_turretMotor = new TalonFX(35);

    private Slot0Configs slot0Configs = new Slot0Configs();
    private MotionMagicConfigs motionConfigs = new MotionMagicConfigs();

    private SmartDashboardNumber kTurretAccel = new SmartDashboardNumber("turret/motion-accel", 0);
    private SmartDashboardNumber kTurretVel = new SmartDashboardNumber("turret/motion-velocity", 0);

    private SmartDashboardNumber turretKs = new SmartDashboardNumber("turret/ks", 0);
    private SmartDashboardNumber turretKa = new SmartDashboardNumber("turret/ka", 0);
    private SmartDashboardNumber turretKv = new SmartDashboardNumber("turret/kv", 0); //to be tuned;
    private SmartDashboardNumber turretKp = new SmartDashboardNumber("turret/kp", 0); //to be tuned;
    private SmartDashboardNumber turretKi = new SmartDashboardNumber("turret/ki", 0);
    private SmartDashboardNumber turretKd = new SmartDashboardNumber("turret/kd", 0);

    private Turret() {
        super("turret");

        this.m_turretMotor.getConfigurator().apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withPeakForwardDutyCycle(1d)
                .withPeakReverseDutyCycle(1d)
                .withNeutralMode(NeutralModeValue.Brake)
        );

        this.slot0Configs = new Slot0Configs()
            .withKS(turretKs.getNumber())
            .withKA(turretKa.getNumber())
            .withKV(turretKv.getNumber())
            .withKP(turretKp.getNumber())
            .withKI(turretKi.getNumber())
            .withKD(turretKd.getNumber());

        this.motionConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(kTurretAccel.getNumber())
            .withMotionMagicCruiseVelocity(kTurretVel.getNumber());

        this.m_turretMotor.getConfigurator().apply(slot0Configs);
        this.m_turretMotor.getConfigurator().apply(motionConfigs);
    }

    public void setTurretPosition(double angle) {
        this.m_turretMotor.setControl(
            new MotionMagicVoltage(MathUtil.clamp(this.angleToRotation(angle), kMinTurretRotation, kMaxTurretRotation))
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(true)
        );
    }

    public void reset() {
        this.m_turretMotor.setControl(new CoastOut());
        this.m_turretMotor.setPosition(0d);
    }

    private double angleToRotation(double angle) {
        return ((kMaxTurretRotation - kMinTurretRotation) / (kMaxTurretAngle - kMinTurretAngle)) * (angle - kMinTurretAngle) + kMinTurretRotation;
    }

    @Override
    public void periodic() {
        if (turretKs.hasChanged()
        || turretKv.hasChanged()
        || turretKp.hasChanged()
        || turretKi.hasChanged()
        || turretKd.hasChanged()
        || turretKa.hasChanged()) {
            slot0Configs.kS = turretKs.getNumber();
            slot0Configs.kV = turretKv.getNumber();
            slot0Configs.kP = turretKp.getNumber();
            slot0Configs.kI = turretKi.getNumber();
            slot0Configs.kD = turretKd.getNumber();
            slot0Configs.kA = turretKa.getNumber();

            if (!Utils.isSimulation()) this.m_turretMotor.getConfigurator().apply(slot0Configs);
            System.out.println("applyied");
        }

        if (kTurretAccel.hasChanged() || kTurretVel.hasChanged()) {
            this.motionConfigs.MotionMagicAcceleration = kTurretAccel.getNumber();
            this.motionConfigs.MotionMagicCruiseVelocity = kTurretVel.getNumber();
            this.m_turretMotor.getConfigurator().apply(this.motionConfigs);
        }

        SmartDashboard.putNumber("turret/acceleration", this.m_turretMotor.getAcceleration().getValueAsDouble());
        SmartDashboard.putNumber("turret/velocity", this.m_turretMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("turret/position", this.m_turretMotor.getPosition().getValueAsDouble());
    }



    public static Turret getInstance() {
        if (instance == null) instance = new Turret();
        return instance;
    }
}
