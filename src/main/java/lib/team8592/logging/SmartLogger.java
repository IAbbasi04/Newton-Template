package lib.team8592.logging;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
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

    public ShuffleboardTab getShuffleboardTab() {
        return this.shuffleboardTab;
    }

    public List<GenericEntry> getAllLoggedEntries() {
        List<GenericEntry> entries = new ArrayList<>();
        for (String key : Collections.list(cards.keys())) {
            entries.add(cards.get(key));
        }
        return entries;
    }

    public GenericEntry getEntry(String name) {
        for (String key : Collections.list(cards.keys())) {
            if (key.equals(name)) {
                return cards.get(key);
            }
        }
        return null;
    }

    public SmartLogger initialize() {
        if (logToShuffleboard && !initialized()) {
            this.shuffleboardTab = Shuffleboard.getTab(name);
            this.initialized = true;
        }

        return this;
    }

    public boolean initialized() {
        return initialized;
    }

    public SmartLogger enable(boolean enable) {
        this.logToShuffleboard = enable;
        return this;
    }

    public SmartLogger enable() {
        return this.enable(true);
    }

    public SmartLogger disable() {
        return this.enable(false);
    }

    public SmartLogger addSendable(Sendable sendable) {
        this.shuffleboardTab.add(sendable);
        return this;
    }

    public void log(String key, double value) {
        Logger.recordOutput(name + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (cards.get(key) == null) { // Card doesn't exist yet on shuffleboard
            cards.put(key, shuffleboardTab.add(key, value).getEntry());
        } else { // Card already exists on shuffleboard
            cards.get(key).setDouble(value);
        }
    }

    public void log(String key, String value) {
        Logger.recordOutput(name + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (cards.get(key) == null) { // Card doesn't exist yet on shuffleboard
            cards.put(key, shuffleboardTab.add(key, value).getEntry());
        } else { // Card already exists on shuffleboard
            cards.get(key).setString(value);
        }
    }

    public void log(String key, boolean value) {
        Logger.recordOutput(name + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (cards.get(key) == null) { // Card doesn't exist yet on shuffleboard
            cards.put(key, shuffleboardTab.add(key, value).getEntry());
        } else { // Card already exists on shuffleboard
            cards.get(key).setBoolean(value);
        }
    }

    public <E extends Enum<E>> void log(String key, E value) {
        Logger.recordOutput(name + "/" + key, value.name()); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (cards.get(key) == null) { // Card doesn't exist yet on shuffleboard
            cards.put(key, shuffleboardTab.add(key, value.name()).getEntry());
        } else { // Card already exists on shuffleboard
            cards.get(key).setString(value.name());
        }
    }

    public void log(String key, ChassisSpeeds value) {
        Logger.recordOutput(name + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (cards.get(key) == null) { // Card doesn't exist yet on shuffleboard
            cards.put(key, shuffleboardTab.add(key, value.toString()).getEntry());
        } else { // Card already exists on shuffleboard
            cards.get(key).setString(value.toString());
        }
    }

    public void log(String key, Pose2d value) {
        Logger.recordOutput(name + "/" + key, value); // Record to AdvantageKit logs
        if (!this.logToShuffleboard) return; // Do not proceed if we do not want to log to shuffleboard
        if (!initialized()) initialize(); // Initialize the shuffleboard tab if not already initialized
        if (cards.get(key) == null) { // Card doesn't exist yet on shuffleboard
            cards.put(key, shuffleboardTab.add(key, value.toString()).getEntry());
        } else { // Card already exists on shuffleboard
            cards.get(key).setString(value.toString());
        }
    }

    public <T> void logValue(String key, T value) {
        if (value.getClass().equals(Pose2d.class)) { // Pose 2d
            Logger.recordOutput(name + "/" + key, (Pose2d)value);
        } else if (value.getClass().equals(ChassisSpeeds.class)) { // Chassis Speeds
            Logger.recordOutput(name + "/" + key, (ChassisSpeeds)value);
        } else if (value.getClass().equals(Double.class)) { // double
            Logger.recordOutput(name + "/" + key, (Double)value);
        } else if (value.getClass().equals(Boolean.class)) { // boolean
            Logger.recordOutput(name + "/" + key, (Boolean)value);
        } else if (value.getClass().equals(String.class)) { // String
            Logger.recordOutput(name + "/" + key, (String)value);
        } else { // Enum or anything not mentioned
            Logger.recordOutput(name + "/" + key, value.toString());
        }
    }
}