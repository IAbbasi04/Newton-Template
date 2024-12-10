package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.team8592.logging.SmartLogger;

public abstract class NewtonSubsystem extends SubsystemBase {
    protected SmartLogger logger;
    protected boolean logToShuffleboard = false;

    protected NewtonSubsystem(boolean logToShuffleboard) {
        this.logToShuffleboard = logToShuffleboard;
        this.logger = new SmartLogger(getName(), logToShuffleboard);
    }

    public boolean currentlyCommanded(){
        return getCurrentCommand() != null && getCurrentCommand() != getDefaultCommand();
    }

    @Override
    public void periodic() {
        periodicOutputs();
        periodicLogs();
    }
    
    public void periodicOutputs() {}

    public abstract void periodicLogs();

    public abstract void stop();
}