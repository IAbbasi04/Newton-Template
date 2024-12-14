package lib.team8592.hardware.motor;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import lib.team8592.PIDGainsProfile;
import lib.team8592.Utils;

public class TalonFXMotorController extends MotorController {
    public TalonFX motor; // Made public so it can be accessed as a follower

    private TalonFXConfiguration configuration;

    private PositionVoltage positionOutput;
    private VelocityVoltage velocityOutput;

    public TalonFXMotorController(int motorID) {
        this(motorID, false);
    }

    public TalonFXMotorController(int motorID, boolean reversed) {
        this.motor = new TalonFX(motorID);
        this.motor.setInverted(reversed);

        this.configuration = new TalonFXConfiguration();
        this.configuration.MotorOutput.Inverted = reversed ? 
            InvertedValue.Clockwise_Positive :
            InvertedValue.CounterClockwise_Positive;

        this.motor.getConfigurator().apply(configuration);

        this.positionOutput = new PositionVoltage(0.0);
        this.velocityOutput = new VelocityVoltage(0.0);
    }

    @Override
    public void withGains(PIDGainsProfile gains) {
        super.motorPIDGains.add(gains.getSlot(), gains);
        
        switch (gains.pidSlot) {
            case 0:
                Slot0Configs slot0Config = new Slot0Configs()
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKG(gains.kG)
                    .withKS(gains.kS);

                this.motor.getConfigurator().apply(slot0Config);
            case 1:
                Slot1Configs slot1Config = new Slot1Configs()
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKG(gains.kG)
                    .withKS(gains.kS);

                this.motor.getConfigurator().apply(slot1Config);
                break;
            case 2:
                Slot2Configs slot2Config = new Slot2Configs()
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKG(gains.kG)
                    .withKS(gains.kS);

                this.motor.getConfigurator().apply(slot2Config);
                break;
            default:
                SlotConfigs slotConfig = new SlotConfigs()
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKG(gains.kG)
                    .withKS(gains.kS);

                this.motor.getConfigurator().apply(slotConfig);
                break;
        }

        
    }

    @Override
    public void setPercentOutput(double percent) {
        this.motor.set(percent);
    }

    @Override
    public void setVelocity(double desiredRPM, int pidSlot) {
        double desiredRPS = desiredRPM / 60.0;
        if (motorPIDGains.get(pidSlot) != null) {
            Utils.clamp(
                desiredRPS, 
                -motorPIDGains.get(pidSlot).maxVelocity,
                motorPIDGains.get(pidSlot).maxVelocity
            );
        }
        this.motor.setControl(velocityOutput.withSlot(pidSlot).withVelocity(desiredRPS));
    }

    @Override
    public void setPositionSmartMotion(double desiredRotations, int pidSlot) {
        if (motorPIDGains.get(pidSlot) != null) {
            Utils.clamp(
                desiredRotations,
                motorPIDGains.get(pidSlot).softLimitMin,
                motorPIDGains.get(pidSlot).softLimitMax
            );
        }
        this.motor.setControl(positionOutput.withSlot(pidSlot).withPosition(desiredRotations));
    }

    @Override
    public void setFollowerTo(MotorController master, boolean reversed) {
        this.motor.setControl(new Follower(master.getAsTalonFX().motor.getDeviceID(), reversed));
    }

    @Override
    public void setCurrentLimit(int currentAmps) {
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.StatorCurrentLimit = currentAmps;
        currentConfigs.StatorCurrentLimitEnable = true;

        this.configuration.CurrentLimits = currentConfigs;
        this.motor.getConfigurator().apply(configuration);
    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
        NeutralModeValue neutralMode = NeutralModeValue.Brake;
        switch(idleMode) {
            case kCoast:
                neutralMode = NeutralModeValue.Coast;
                break;
            case kBrake: default: // Should default to brake mode
                break;
        }
        this.motor.setNeutralMode(neutralMode);
    }

    @Override
    public double getVelocityRPM() {
        return this.motor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getRotations() {
        return this.motor.getPosition().getValueAsDouble();
    }

    @Override
    public void resetEncoderPosition(double rotations) {
        this.motor.setPosition(rotations);
    }
}