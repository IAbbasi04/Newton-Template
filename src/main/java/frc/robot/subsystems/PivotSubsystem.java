package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.PIVOT;
import frc.robot.Constants.SIMULATION;
import frc.robot.Ports;
import frc.robot.Robot;
import lib.team8592.MatchMode;
import lib.team8592.hardware.motor.SparkFlexMotorController;

public class PivotSubsystem extends NewtonSubsystem {
    private Positions desiredPosition = Positions.REST;
    private Positions lastPosition = desiredPosition;
    private SparkFlexMotorController pivotMotor;

    private Timer simulatedMoveTimer = new Timer();

    private AbsoluteEncoder absoluteEncoder;

    private double desiredVelocityRPM = 0.0;

    public enum Positions {
        GROUND,
        REST,
        HP_LOAD,
        SCORE_HIGH,
        SCORE_LOW,
        OVERRIDE
        ;
        public double degrees = 0.0;
        Positions() {
            try { // Use the constant as a reference
                this.degrees = PIVOT.class.getDeclaredField(name() + "_DEGREES").getDouble(PIVOT.class);
            } catch (Exception e) { // If position doesn't exist set it to -1
                this.degrees = -1d;
            }
        }
    }

    public PivotSubsystem(boolean logToShuffleboard) {
        super(logToShuffleboard);
        this.pivotMotor = new SparkFlexMotorController(Ports.PIVOT_CAN_ID);

        this.pivotMotor.withGains(PIVOT.RAISE_GAINS);
        this.pivotMotor.withGains(PIVOT.LOWER_GAINS);

        this.absoluteEncoder = this.pivotMotor.motor.getAbsoluteEncoder();

        this.pivotMotor.resetEncoderPosition(this.absoluteEncoder.getPosition() - PIVOT.ABSOLUTE_ENCODER_OFFSET);
    }

    /**
     * Sets the desired pivot position state
     * @param position the desired {@code Positions} state
     */
    public void setPosition(Positions position) {
        if (this.desiredPosition == position) return;

        if (this.lastPosition != this.desiredPosition) {
            this.lastPosition = this.desiredPosition;
        }

        this.desiredPosition = position;
        this.simulatedMoveTimer.reset();
    }

    /**
     * Sets the desired velocity of the motor in RPM
     */
    public void setVelocity(double desiredRPM) {
        this.desiredVelocityRPM = desiredRPM;
        this.desiredPosition = Positions.OVERRIDE;
        this.simulatedMoveTimer.stop();
        this.simulatedMoveTimer.reset();
    }

    /**
     * Whether the pivot is at the desired target pivot angle
     * @return true if within set tolerance, false otherwise
     */
    public boolean atTargetPosition() {
        if (this.desiredPosition == Positions.OVERRIDE) return true;

        if (Robot.isSimulation()) {
            this.simulatedMoveTimer.start();
            return this.simulatedMoveTimer.get() >= this.getSimulatedMoveTime(this.lastPosition, desiredPosition);
        }

        double errorDegrees = getCurrentDegrees() - this.desiredPosition.degrees;
        return Math.abs(errorDegrees) <= PIVOT.AT_TARGET_TOLERANCE;
    }

    /**
     * Whether the pivot is at a particular pivot position
     * @param position the desired {Positions} pivot angle to compare with
     * @return true if within tolerance of such position, false otherwise
     */
    public boolean atPosition(Positions position) {
        if (Robot.isSimulation()) {
            this.simulatedMoveTimer.start();
            return this.desiredPosition.equals(position) && 
                this.simulatedMoveTimer.get() >= getSimulatedMoveTime(this.lastPosition, position);
        }

        return Math.abs(this.getCurrentDegrees() - position.degrees) 
                <= PIVOT.AT_TARGET_TOLERANCE;
    }

    /**
     * The current position of the pivot motor in rotations
     */
    private double getCurrentRotations() {
        return this.pivotMotor.getRotations();
    }

    /**
     * The current position of the pivot in degrees
     */
    private double getCurrentDegrees() {
        return PIVOT.motorRotationsToDegrees(getCurrentRotations());
    }

    /**
     * Sets the position of the motor depending on desired degrees of the pivot
     */
    private void setMotorFromDegrees(double desiredDegrees, int slot) {
        this.pivotMotor.setPositionSmartMotion(
            PIVOT.degreesToMotorRotations(desiredDegrees), 
            slot
        );
    }

    /**
     * Gets the simulated estimate of time it would take to move to the next desired position
     * @return the time it would take to move from the starting position to the ending position
     * @param start the starting position
     * @param end the ending position
     */
    private double getSimulatedMoveTime(Positions start, Positions end) {
        double percentOfDistance = (end.degrees - start.degrees) / (Positions.REST.degrees + Positions.GROUND.degrees);
        return Math.abs(SIMULATION.PIVOT_GROUND_TO_REST_MOVE_TIME * percentOfDistance);
    }

    /**
     * Sets the motor velocity in RPM
     */
    private void setMotorVelocity(double velocityRPM) {
        this.pivotMotor.setVelocity(velocityRPM);
    }

    @Override
    public void onInit(MatchMode mode) {
        this.stop();
    }

    @Override
    public void periodicLogs() {
        this.logger.logEnum("Desired Position State", this.desiredPosition);
        this.logger.logEnum("Last Position State", this.lastPosition);
        this.logger.logDouble("Desired Position Degrees", this.desiredPosition.degrees);
        this.logger.logDouble("Current Position Rotations", this.getCurrentRotations());
        this.logger.logDouble("Current Position Degrees", this.getCurrentDegrees());
        this.logger.logBoolean("At Target Position", this.atTargetPosition());
        this.logger.logDouble("Absolute Encoder Rotations", this.absoluteEncoder.getPosition());

        if (Robot.isSimulation()) { // Only log values in simulation
            this.logger.logDouble("Simulated Time to Move", this.getSimulatedMoveTime(lastPosition, desiredPosition));
        }
    }

    @Override
    public void periodicOutputs() {
        if (this.desiredPosition == null) return; // Do nothing if desired position is null

        if (this.desiredPosition == Positions.OVERRIDE) { // If set to the override state use desired velocity
            this.setMotorVelocity(desiredVelocityRPM);
            return;
        }

        int appliedPIDSlot;
        if (getCurrentRotations() <= this.desiredPosition.degrees) { // Raising pivot
            appliedPIDSlot = PIVOT.RAISE_GAINS.pidSlot;
        } else { // Lowering pivot
            appliedPIDSlot = PIVOT.LOWER_GAINS.pidSlot;
        }

        this.setMotorFromDegrees(this.desiredPosition.degrees, appliedPIDSlot);
    }

    @Override
    public void stop() {
        this.setMotorVelocity(0.0);
    }
}