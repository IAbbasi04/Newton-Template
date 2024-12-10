package frc.robot;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import lib.team8592.PIDGainsProfile;

public final class Constants {
    public final class SHARED {
        public static final String LOG_FOLDER = "CustomLogs";
    }

    public final class FIELD {
        public static final double RED_WALL_X = 16.542; // distance in meters between red and blue sides
    }

    public final class LOGGER {
        public static final String GAME = "_GAME_";
        public static final String YEAR = "_YEAR_";
        public static final String ROBOT = "_ROBOT_";
        public static final String TEAM = "_TEAM_";
    }

    public final class SIMULATION {
        public static final double SWERVE_TRANSLATE_DELTA = 0.02; // How much to update the translational velocity of the swerve in simulation
        public static final double SWERVE_ROTATE_DELTA = 0.02; // How much to update the rotational velocity of the swerve in simulation
    }

    public final class CONVERSIONS {
        public static final double METERS_SECOND_TO_TICKS_TALONFX = ((2048 * 6.75 * 60) / (200 * Math.PI * 0.0508));

        public static final double RPM_TO_TICKS_100_MS_TALONFX = 2048.0 / 600.0;

        public static final double ANGLE_DEGREES_TO_TICKS_SPARKFLEX = 4096 / 360.0;
        public static final double TICKS_TO_ANGLE_DEGREES_SPARKFLEX = 360.0 / 4096.0;

        public static final double DEG_TO_RAD = 0.0174533;
        public static final double RAD_TO_DEG = 57.2958;
        public static final double IN_TO_METERS = 0.0254;
        public static final double METERS_TO_FEET = 3.28084;
    }

    public final class MEASUREMENTS {
        public static final double FIELD_LENGTH_METERS = 16.5410515;
    }

    public final class CONTROLLERS {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;

        public static final double TRIGGER_PRESS_DEADBAND = 0.1;
    }

    public final class POWER {
        public static final int SWERVE_MAX_VOLTAGE = 12;
        public static final int SWERVE_DRIVE_CURRENT_LIMIT = 80;
        public static final int SWERVE_STEER_CURRENT_LIMIT = 40;
    }

    public final class INTAKE {
        public static final double TOP_ROLLER_INTAKE_RPM = 3000d;
        public static final double TOP_ROLLER_SCORE_HIGH_RPM = -2000d;
        public static final double TOP_ROLLER_SCORE_GRID_RPM = -2500d;

        public static final double BOTTOM_ROLLER_INTAKE_RPM = 3000d;
        public static final double BOTTOM_ROLLER_SCORE_HIGH_RPM = -2000d;
        public static final double BOTTOM_ROLLER_SCORE_GRID_RPM = -2500d;

        public static final PIDGainsProfile TOP_ROLLER_GAINS = new PIDGainsProfile()
            .setP(1E-3)
            .setFF(1E-4);

        public static final PIDGainsProfile BOTTOM_ROLLER_GAINS = new PIDGainsProfile()
            .setP(1E-3)
            .setFF(1E-4);
    }

    public final class SWERVE {
        public static final String LOG_PATH = SHARED.LOG_FOLDER+"/Swerve/";

        public static final double STEER_P = 100;
        public static final double STEER_I = 0;
        public static final double STEER_D = 0.2;
        public static final double STEER_S = 0;
        public static final double STEER_V = 1.5;
        public static final double STEER_A = 0;

        public static final double DRIVE_P = 3;
        public static final double DRIVE_I = 0;
        public static final double DRIVE_D = 0;
        public static final double DRIVE_S = 0;
        public static final double DRIVE_V = 0;
        public static final double DRIVE_A = 0;

        //TODO: Double check that these PID constants still work
        public static final double SNAP_TO_kP = 3.7;
        public static final double SNAP_TO_kI = 0.0;
        public static final double SNAP_TO_kD = 0.1;

        public static final int STEER_STATOR_CURRENT_LIMIT = 60;

        public static final int CALCULATED_SLIP_CURRENT = 150;

        public static final double MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND = 4.73;
        public static final double MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND = Math.toRadians(720);
        public static final double COUPLING_GEAR_RATIO = 3.5714285714285716;
        public static final double DRIVE_GEAR_RATIO = 6.746031746031747;
        public static final double STEER_GEAR_RATIO = 21.428571428571427;
        public static final double WHEEL_RADIUS_INCHES = 2;

        public static final boolean INVERT_LEFT_SIDE = false;
        public static final boolean INVERT_RIGHT_SIDE = true;

        public static final double SIMULATED_STEER_INERTIA = 0.00001;
        public static final double SIMULATED_DRIVE_INERTIA = 0.06;
        public static final double SIMULATION_LOOP_PERIOD = 0.005;
        public static final double STEER_FRICTION_VOLTAGE = 0.25;
        public static final double DRIVE_FRICTION_VOLTAGE = 0.25;

        //TODO: Tone these down appropriately as per BB rules
        public static final double TRANSLATE_POWER_FAST = 1.0;
        public static final double ROTATE_POWER_FAST = 0.75;
        public static final double TRANSLATE_POWER_SLOW = 0.15;
        public static final double ROTATE_POWER_SLOW = 0.15;

        public static final int TRANSLATION_SMOOTHING_AMOUNT = 3;
        public static final int ROTATION_SMOOTHING_AMOUNT = 1;

        public static final double JOYSTICK_EXPONENT = 2;

        public static final Rotation2d BLUE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0);
        public static final Rotation2d RED_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180);

        // public static final double BLACK_FRONT_LEFT_STEER_OFFSET = -0.388427734375;
        // public static final double ORANGE_FRONT_RIGHT_STEER_OFFSET = -0.462646484375;
        // public static final double TEAL_BACK_LEFT_STEER_OFFSET = -0.18017578125;
        // public static final double WHITE_BACK_RIGHT_STEER_OFFSET = -0.4853515625;

        //TODO: Set these
        public static final double BLACK_FRONT_LEFT_STEER_OFFSET = 0.062255859375;
        public static final double ORANGE_FRONT_RIGHT_STEER_OFFSET = -0.37890625;
        public static final double TEAL_BACK_LEFT_STEER_OFFSET = -0.0029296875;
        public static final double WHITE_BACK_RIGHT_STEER_OFFSET = -0.293701171875;

        public static final boolean BLACK_FRONT_LEFT_STEER_INVERT = true;
        public static final boolean ORANGE_FRONT_RIGHT_STEER_INVERT = true;
        public static final boolean TEAL_BACK_LEFT_STEER_INVERT = true;
        public static final boolean WHITE_BACK_RIGHT_STEER_INVERT = true;


        //TODO: Set these
        public static final double BLACK_FRONT_LEFT_X_POSITION = 8.375;
        public static final double BLACK_FRONT_LEFT_Y_POSITION = 8.375;

        //TODO: Set these
        public static final double ORANGE_FRONT_RIGHT_X_POSITION = 8.375;
        public static final double ORANGE_FRONT_RIGHT_Y_POSITION = -8.375;

        //TODO: Set these
        public static final double TEAL_BACK_LEFT_X_POSITION = -8.375;
        public static final double TEAL_BACK_LEFT_Y_POSITION = 8.375;

        //TODO: Set these
        public static final double WHITE_BACK_RIGHT_X_POSITION = -8.375;
        public static final double WHITE_BACK_RIGHT_Y_POSITION = -8.375;

        //TODO: Double check that these still work
        public static final double PATH_FOLLOW_TRANSLATE_kP = 6d;
        public static final double PATH_FOLLOW_TRANSLATE_kI = 0d;
        public static final double PATH_FOLLOW_TRANSLATE_kD = 0d;

        //TODO: Double check that these still work
        public static final double PATH_FOLLOW_ROTATE_kP = 3.7;
        public static final double PATH_FOLLOW_ROTATE_kI = 0d;
        public static final double PATH_FOLLOW_ROTATE_kD = 0.1;

        public static final double PATH_FOLLOW_ROTATE_MAX_VELOCITY = 4 * Math.PI;
        public static final double PATH_FOLLOW_ROTATE_MAX_ACCELLERATION = 2 * Math.PI;

        public static final double PATH_FOLLOW_POSITION_TOLERANCE = 0.1;
        public static final double PATH_FOLLOW_VELOCITY_TOLERANCE = 0.1;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.5;

        public static final double DRIVE_TRAIN_RADIUS = 0.6;
    }

    public final class VISION {
        public static final AprilTagFieldLayout APRIL_TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        public static final Transform3d CAMERA_POSE_TO_ROBOT_POSE = new Transform3d();
    }

    public final class ROBOT {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Robot/";
    }

    public class SUPPLIERS{
        public static final String LOG_PATH = SHARED.LOG_FOLDER+"/Suppliers/";
    }
}
