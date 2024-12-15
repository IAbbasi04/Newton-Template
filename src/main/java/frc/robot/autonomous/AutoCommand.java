package frc.robot.autonomous;

import java.util.*;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.proxies.NewtonWrapperCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;

/**
 * Class to provide subsystems, convenient methods, and a constructor to autonomous commands
 */
public abstract class AutoCommand extends NewtonWrapperCommand {
    /**
     * If this is set, the odometry's known position will be set to this at the start of auto
     */
    protected Pose2d startPose = null;

    /**
     * The entire auto condensed into a single command
     */
    protected Command autoCommand = null;

    /**
     * The name of the auto to be displayed on the selector
     */
    private String name = null;

    /**
     * Startup time saving if/when multiple copies of the same path are requested
     */
    private static HashMap<String, Trajectory> cachedChoreoTrajectories = new HashMap<String,Trajectory>();
    private static HashMap<String, Trajectory> cachedPathPlannerTrajectories = new HashMap<String,Trajectory>();

    /**
     * Create an auto routine from the passed-in commands.
     *
     * @param commands as many commands as you want. Will
     * be run in sequence (one after the other).
     */
    protected AutoCommand(Command... commands) {
        super(Commands.sequence(commands));
        this.autoCommand = m_command;
        this.name = getName();
    }

    protected AutoCommand(String filePath) {
        super(new PathPlannerAuto(filePath));
        this.autoCommand = new PathPlannerAuto(filePath);
        this.name = filePath;
    }

    /**
     * {@link Commands#none()} as an {@link AutoCommand}
     */
    protected AutoCommand(){
        super(Commands.none());
    }

    /**
     * Creates an {@code AutoCommand} object based on an existing {@code AutoCommand}
     * @param master
     */
    protected AutoCommand(AutoCommand master) {
        super(new WaitCommand(0));
        this.autoCommand = master.autoCommand;
        this.name = getName();
    }

    /**
     * Creates a PathPlanner-based auto from the indicated file path 
     * @param file path without the .auto extension (ex. "ExamplePreloadTwoGamePieceAuto")
     */
    public static AutoCommand createAutoFromPathPlanner(String file) {
        return new AutoCommand(file) {
            @Override
            public Pose2d getStartPose() {
                return PathPlannerAuto.getStaringPoseFromAutoFile(file);
            }
        };
    }

    public static AutoCommand getDefaultAuto() {
        return new AutoCommand() {
            @Override
            public Pose2d getStartPose() {
                return new Pose2d();
            }

            @Override
            public String toString() {
                this.setAutoName("DEFAULT - DO NOTHING");
                return getAutoName();
            }
        };
    }

    /**
     * Adds commands to the end of the auto
     * *CURRENTLY ONLY WORKS FOR PATHPLANNER AUTOS FOR SOME REASON*
     */
    public AutoCommand append(Command... commands) {
        if (this.autoCommand == null) this.autoCommand = new WaitCommand(0.0);
        this.autoCommand = this.autoCommand.andThen(commands);
        return this;
    }

    /**
     * Get a choreo trajectory by name as a WPILib trajectory.
     *
     * @param name the name of the .traj file; this shouldn't contain the path or
     * the filename extension
     *
     * @return The trajectory converted to WPILib's {@link Trajectory}. Throws a
     * {@code FileNotFoundException} if the name doesn't represent a .traj file
     * located in the {@code choreo} folder in the {@code deploy} folder
     */
    protected static final Trajectory getChoreoTrajectory(String name){
        if(cachedChoreoTrajectories.containsKey(name)){
            return cachedChoreoTrajectories.get(name);
        }
        else{
            Trajectory wpilibTrajectory = fromPathPlannerPath(PathPlannerPath.fromChoreoTrajectory(name));
            cachedChoreoTrajectories.put(name, wpilibTrajectory);
            return wpilibTrajectory;
        }
    }

    /**
     * Get a PathPlanner path by name as a WPILib trajectory.
     *
     * @param name the name of the path in PathPlanner file; this shouldn't contain
     * the file path or the filename extension
     *
     * @return The path converted to WPILib's {@link Trajectory}. Throws a
     * {@code FileNotFoundException} if the name doesn't represent a .path file
     * located in {@code /src/main/deploy/pathplanner/paths}
     */
    protected static final Trajectory getPathPlannerTrajectory(String name){
        if(cachedPathPlannerTrajectories.containsKey(name)){
            return cachedPathPlannerTrajectories.get(name);
        }
        else{
            Trajectory wpilibTrajectory = fromPathPlannerPath(PathPlannerPath.fromPathFile(name));
            cachedPathPlannerTrajectories.put(name, wpilibTrajectory);
            return wpilibTrajectory;
        }
    }

    /**
     * Set the start pose of this auto to the first pose of a Choreo path.
     *
     * @param name the name of the Choreo path to get the start pose from
     */
    protected AutoCommand setStartStateFromChoreoTrajectory(String name){
        if(!cachedChoreoTrajectories.containsKey(name)){
            getChoreoTrajectory(name); // Adds the path to the cached trajectory map
        }
        this.startPose = cachedChoreoTrajectories.get(name).getInitialPose();

        return this;
    }

    /**
     * Set the start pose of this auto to the first pose of a PathPlanner path.
     *
     * @param name the name of the PathPlanner path to get the start pose from
     */
    protected AutoCommand setStartStateFromPathPlannerTrajectory(String name){
        this.startPose = PathPlannerAuto.getStaringPoseFromAutoFile(name);
        return this;
    }

    /**
     * Convert a PathPlanner path into a WPILib trajectory
     *
     * @param path the PathPlannerPath to convert
     * @return the path converted to a WPILib trajectory
     */
    private static Trajectory fromPathPlannerPath(PathPlannerPath path){
        PathPlannerTrajectory pathPlannerTraj = (
            path.getTrajectory(new ChassisSpeeds(), path.getPreviewStartingHolonomicPose().getRotation())
        );

        List<PathPlannerTrajectory.State> pathPlannerStates = pathPlannerTraj.getStates();
        ArrayList<State> wpilibStates = new ArrayList<>();

        // Convert all the PathPlanner states to WPILib trajectory states and add
        // them to the wpilibStates ArrayList
        for (PathPlannerTrajectory.State pathPlannerState : pathPlannerStates) {
            State wpilibState = new State(
                pathPlannerState.timeSeconds,
                pathPlannerState.velocityMps,
                pathPlannerState.accelerationMpsSq,
                pathPlannerState.getTargetHolonomicPose(),
                pathPlannerState.curvatureRadPerMeter
            );
            wpilibStates.add(wpilibState);
        }
        return new Trajectory(wpilibStates);
    }

    public Command getAutoAsCommand() {
        return this.autoCommand;
    }

    public AutoCommand setAutoName(String name) {
        this.name = name;
        return this;
    }

    public String getAutoName() {
        return this.name;
    }

    public abstract Pose2d getStartPose();

    @Override
    public String toString() {
        return getAutoName();
    }
}