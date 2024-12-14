package lib.team8592.hardware.motor;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.AccelStrategy;

import lib.team8592.PIDGainsProfile;
import lib.team8592.Utils;

public class SparkMaxMotorController extends MotorController {
    public CANSparkMax motor; // Made public so it can be accessed as a follower
    private SparkPIDController motorCtrl;
    private RelativeEncoder encoder;

    public SparkMaxMotorController(int motorID) {
        this(motorID, false);
    }

    public SparkMaxMotorController(int motorID, boolean reversed) {
        this.motor = new CANSparkMax(motorID, MotorType.kBrushless);
        this.motorCtrl = motor.getPIDController();
        this.encoder = motor.getEncoder();

        this.motor.setInverted(reversed);
    }

    @Override
    public void withGains(PIDGainsProfile gains) {
        super.motorPIDGains.add(gains.getSlot(), gains);
        
        this.motorCtrl.setP(gains.kP);
        this.motorCtrl.setI(gains.kI);
        this.motorCtrl.setD(gains.kD);
        this.motorCtrl.setFF(gains.kFF);

        motorCtrl.setSmartMotionAllowedClosedLoopError(gains.getTolerance(), gains.getSlot());
        this.motorCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, gains.getSlot());
        this.motorCtrl.setSmartMotionMaxVelocity(gains.getMaxVelocity(), gains.getSlot());
        this.motorCtrl.setSmartMotionMaxAccel(gains.getMaxAcceleration(), gains.getSlot());
    }

    @Override
    public void setPercentOutput(double percent) {
        this.motor.set(percent);
    }

    @Override
    public void setVelocity(double desiredVelocityRPM, int pidSlot) {
        if (motorPIDGains != null) {
            Utils.clamp(
                desiredVelocityRPM, 
                -motorPIDGains.get(pidSlot).maxVelocity,
                motorPIDGains.get(pidSlot).maxVelocity
            );
        }
        this.motorCtrl.setReference(desiredVelocityRPM, ControlType.kSmartVelocity);
    }

    @Override
    public void setPositionSmartMotion(double desiredRotations, int pidSlot) {
        if (motorPIDGains != null) {
            Utils.clamp(
                desiredRotations, 
                motorPIDGains.get(pidSlot).softLimitMin,
                motorPIDGains.get(pidSlot).softLimitMax
            );
        }
        this.motorCtrl.setReference(desiredRotations, ControlType.kSmartMotion);
    }

    @Override
    public void setFollowerTo(MotorController master, boolean reversed) {
        motor.follow(master.getAsSparkFlex().motor, reversed);
    }

    @Override
    public void setCurrentLimit(int currentAmps) {
        this.motor.setSmartCurrentLimit(currentAmps);
        this.motor.setSecondaryCurrentLimit(currentAmps);
    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
        switch (idleMode) {
            case kCoast:
                motor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
                break;
            case kBrake:
            // On default set to brake mode
            default:
                motor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
                break;
        }
    }

    @Override
    public double getVelocityRPM() {
        return encoder.getVelocity();
    }

    @Override
    public double getRotations() {
        return encoder.getPosition();
    }

    @Override
    public void resetEncoderPosition(double rotations) {
        this.encoder.setPosition(rotations);
    }
}