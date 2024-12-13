package lib.team8592.logging;

import java.util.Dictionary;
import java.util.Hashtable;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;

public class SmartLogger {
    private String name = "";
    private ShuffleboardTab shuffleboardTab;
    private Dictionary<String, GenericEntry> cards;

    private boolean logToShuffleboard = true;
    private boolean initialized = false;

    public SmartLogger(String name, boolean logToShuffleboard) {
        this.name = name;
        this.logToShuffleboard = logToShuffleboard;
        this.cards = new Hashtable<>();
    }

    public SmartLogger(String name) {
        this(name, true);
    }

    public void initialize() {
        if (logToShuffleboard && !initialized()) {
            this.shuffleboardTab = Shuffleboard.getTab(name);
            this.initialized = true;
        }
    }

    public boolean initialized() {
        return initialized;
    }

    public void enable(boolean enable) {
        this.logToShuffleboard = enable;
    }

    public void enable() {
        this.logToShuffleboard = true;
    }

    public void disable() {
        this.logToShuffleboard = false;
    }

    public void logDouble(String key, double value) {
        Logger.recordOutput(name + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (cards.get(key) == null) { // Card doesn't exist yet on shuffleboard
            cards.put(key, shuffleboardTab.add(key, value).getEntry());
        } else { // Card already exists on shuffleboard
            cards.get(key).setDouble(value);
        }
    }

    public void logString(String key, String value) {
        Logger.recordOutput(name + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (cards.get(key) == null) { // Card doesn't exist yet on shuffleboard
            cards.put(key, shuffleboardTab.add(key, value).getEntry());
        } else { // Card already exists on shuffleboard
            cards.get(key).setString(value);
        }
    }

    public void logBoolean(String key, boolean value) {
        Logger.recordOutput(name + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (cards.get(key) == null) { // Card doesn't exist yet on shuffleboard
            cards.put(key, shuffleboardTab.add(key, value).getEntry());
        } else { // Card already exists on shuffleboard
            cards.get(key).setBoolean(value);
        }
    }

    public <E extends Enum<E>> void logEnum(String key, E value) {
        Logger.recordOutput(name + "/" + key, value.name()); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (cards.get(key) == null) { // Card doesn't exist yet on shuffleboard
            cards.put(key, shuffleboardTab.add(key, value.name()).getEntry());
        } else { // Card already exists on shuffleboard
            cards.get(key).setString(value.name());
        }
    }

    public void logChassisSpeeds(String key, ChassisSpeeds value) {
        Logger.recordOutput(name + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (cards.get(key) == null) { // Card doesn't exist yet on shuffleboard
            cards.put(key, shuffleboardTab.add(key, value.toString()).getEntry());
        } else { // Card already exists on shuffleboard
            cards.get(key).setString(value.toString());
        }
    }

    public void logPose2d(String key, Pose2d value) {
        Logger.recordOutput(name + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (cards.get(key) == null) { // Card doesn't exist yet on shuffleboard
            cards.put(key, shuffleboardTab.add(key, value.toString()).getEntry());
        } else { // Card already exists on shuffleboard
            cards.get(key).setString(value.toString());
        }
    }

    public <T> void log(String key, T value) {
        if (value.getClass() == Pose2d.class) { // Pose 2d
            Logger.recordOutput(name + "/" + key, (Pose2d)value);
        }
    }
}