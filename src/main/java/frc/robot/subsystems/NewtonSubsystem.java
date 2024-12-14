package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.team8592.MatchMode;
import lib.team8592.logging.SmartLogger;

public abstract class NewtonSubsystem extends SubsystemBase {
    protected SmartLogger logger;
    protected boolean logToShuffleboard = false;
    private boolean enabled = false;

    protected NewtonSubsystem(boolean logToShuffleboard) {
        this.logToShuffleboard = logToShuffleboard;
        this.logger = new SmartLogger(getName(), logToShuffleboard);
    }

    public void initializeLogger() {
        this.logger.initialize();
    }

    public boolean currentlyCommanded(){
        return getCurrentCommand() != null && getCurrentCommand() != getDefaultCommand();
    }

    public void enableSubsystem(boolean enable) {
        this.enabled = enable;
    }

    public boolean isEnabled() {
        return enabled;
    }
    
    public void periodicOutputs() {}

    public abstract void onInit(MatchMode mode);

    public abstract void periodicLogs();

    public abstract void stop();
}