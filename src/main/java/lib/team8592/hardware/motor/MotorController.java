package lib.team8592.hardware.motor;

import java.util.ArrayList;
import java.util.List;

import lib.team8592.PIDGainsProfile;

public abstract class MotorController {
    protected List<PIDGainsProfile> motorPIDGains = new ArrayList<>();

    public enum IdleMode {
        kBrake,
        kCoast
    }

    public abstract void withGains(PIDGainsProfile gains);
    
    public abstract void setPercentOutput(double percent);

    public abstract void setVelocity(double desiredRPM, int pidSlot);

    public void setVelocity(double desiredRPM) {
        setVelocity(desiredRPM, 0);
    }

    public abstract void setPositionSmartMotion(double desiredRotations, int pidSlot);

    public void setPositionSmartMotion(double desiredRotations) {
        setPositionSmartMotion(desiredRotations, 0);
    }
    
    public abstract void setFollowerTo(MotorController master, boolean reversed);
    
    public void setFollowerTo(MotorController master) {
        setFollowerTo(master, false);
    }

    public abstract void setCurrentLimit(int currentAmps);

    public abstract void setIdleMode(IdleMode idleMode);

    public abstract double getVelocityRPM();

    public abstract double getRotations();

    public abstract void resetEncoderPosition(double rotations);

    public SparkFlexMotorController getAsSparkFlex() {
        return (SparkFlexMotorController)this;
    }

    public SparkMaxMotorController getAsSparkMax() {
        return (SparkMaxMotorController)this;
    }

    public TalonFXMotorController getAsTalonFX() {
        return (TalonFXMotorController)this;
    }
}