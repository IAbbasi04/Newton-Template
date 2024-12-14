package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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

    /**
     * Creates a stop command for this subsystem
     */
    public Command getStopCommand() {
        return this.runOnce(() -> {stop();});
    }

    /**
     * Sets the subsystem's stop method as the default command
     */
    public void setStopAsDefaultCommand() {
        this.setDefaultCommand(getStopCommand());
    }
    
    /**
     * Set the default command of a subsystem (what to run if no other command requiring it is running).
     * <p> NOTE: all subsystems also have a setDefaultCommand method; this version includes a check for
     * default commands that cancel incoming commands that require the subsystem. Unless you're sure
     * of what you're doing, you should use this one.
     *
     * @param command to command to set as default
     */
    @Override
    public void setDefaultCommand(Command command) {
        super.setDefaultCommand(command.withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    }

    
    public void periodicOutputs() {}

    public abstract void onInit(MatchMode mode);

    public abstract void periodicLogs();

    public abstract void stop();
}