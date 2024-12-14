package lib.team8592.hardware.motor;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import lib.team8592.PIDGainsProfile;
import lib.team8592.Utils;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

public class SparkFlexMotorController extends MotorController {
    public CANSparkFlex motor; // Made public so it can be accessed as a follower
    private SparkPIDController motorCtrl;
    private RelativeEncoder encoder;

    public SparkFlexMotorController(int motorID) {
        this(motorID, false);
    }

    public SparkFlexMotorController(int motorID, boolean reversed) {
        this.motor = new CANSparkFlex(motorID, MotorType.kBrushless);
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

        if (gains.softLimit) { // If soft limit values applied in the gains profile
            this.motor.enableSoftLimit(SoftLimitDirection.kForward, true);
            this.motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

            this.motor.setSoftLimit(SoftLimitDirection.kForward, (float)gains.softLimitMax);
            this.motor.setSoftLimit(SoftLimitDirection.kReverse, (float)gains.softLimitMin);
        }

        this.motorCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, gains.pidSlot);
        this.motorCtrl.setSmartMotionAllowedClosedLoopError(gains.tolerance, gains.pidSlot);
        this.motorCtrl.setSmartMotionMaxVelocity(gains.maxVelocity, gains.pidSlot);
        this.motorCtrl.setSmartMotionMaxAccel(gains.maxAcceleration, gains.pidSlot);
    }

    @Override
    public void setPercentOutput(double percent) {
        this.motor.set(percent);
    }

    @Override
    public void setVelocity(double desiredVelocityRPM, int pidSlot) {
        if (motorPIDGains != null && motorPIDGains.size() > 0) {
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