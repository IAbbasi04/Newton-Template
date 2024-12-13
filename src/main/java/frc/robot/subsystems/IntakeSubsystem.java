package frc.robot.subsystems;

import frc.robot.Ports;
import lib.team8592.MatchMode;
import lib.team8592.hardware.motor.*;
import lib.team8592.hardware.motor.MotorController.IdleMode;
import frc.robot.Constants.*;

public class IntakeSubsystem extends NewtonSubsystem {
    private SparkFlexMotorController topRollerMotor, bottomRollerMotor;
    private double desiredTopVelocityRPM, desiredBottomVelocityRPM;

    public IntakeSubsystem(boolean logToShuffleboard) {
        super(logToShuffleboard);

        this.topRollerMotor = new SparkFlexMotorController(Ports.INTAKE_TOP_ROLLER_CAN_ID, true);
        this.bottomRollerMotor = new SparkFlexMotorController(Ports.INTAKE_BOTTOM_ROLLER_CAN_ID);

        this.topRollerMotor.withGains(INTAKE.TOP_ROLLER_GAINS);
        this.bottomRollerMotor.withGains(INTAKE.BOTTOM_ROLLER_GAINS);

        this.topRollerMotor.setIdleMode(IdleMode.kBrake);
        this.bottomRollerMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Sets the desired velocity of the roller motors in RPM
     * @param desiredTopRPM
     * @param desiredBottomRPM
     */
    public void setVelocity(double desiredTopRPM, double desiredBottomRPM) {
        this.desiredTopVelocityRPM = desiredTopRPM;
        this.desiredBottomVelocityRPM = desiredBottomRPM;

        this.topRollerMotor.setVelocity(desiredTopRPM);
        this.bottomRollerMotor.setVelocity(desiredBottomRPM);
    }

    @Override
    public void onInit(MatchMode mode) {
        this.stop();
    }

    @Override
    public void periodicLogs() {
        this.logger.logDouble("Desired Top Roller Velocity", this.desiredTopVelocityRPM);
        this.logger.logDouble("Current Top Roller Velocity", this.topRollerMotor.getVelocityRPM());
        this.logger.logDouble("Desired Bottom Roller Velocity", this.desiredBottomVelocityRPM);
        this.logger.logDouble("Current Bottom Roller Velocity", this.bottomRollerMotor.getVelocityRPM());
    }

    @Override
    public void stop() {
        this.setVelocity(0d, 0d);
    }
}