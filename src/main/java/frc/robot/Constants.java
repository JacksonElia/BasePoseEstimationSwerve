package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class Constants {

  private Constants() {}

  public static final class HardwareConstants {
    public static final String CANIVORE_CAN_BUS_STRING = "Canivore 1";
    public static final String RIO_CAN_BUS_STRING = "rio";

    public static final double FALCON_ENCODER_RESOLUTION = 2048.0;
    public static final double CANCODER_RESOLUTION = 4096.0; 

    public static final double MIN_FALCON_DEADBAND = 0.001;

    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.CTREPCM;

    public static final int TIMEOUT_MS = 30;
  }
  
  public static final class Conversions {
    public static final double DEGREES_TO_CANCODER_UNITS = HardwareConstants.CANCODER_RESOLUTION / 360.0;
    public static final double DEGREES_TO_FALCON_UNITS = HardwareConstants.FALCON_ENCODER_RESOLUTION / 360.0;
  }

  public static final class DriveConstants {
    // Distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(22.25);
    // Distance between front and back wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(28.5);
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Left
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Front Right
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Rear Left
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // Rear Right
    );

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
    public static final int REAR_LEFT_DRIVE_MOTOR_ID = 3;
    public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 4;

    public static final int FRONT_LEFT_TURN_MOTOR_ID = 5;
    public static final int FRONT_RIGHT_TURN_MOTOR_ID = 6;
    public static final int REAR_LEFT_TURN_MOTOR_ID = 7;
    public static final int REAR_RIGHT_TURN_MOTOR_ID = 8;

    public static final int FRONT_LEFT_CANCODER_ID = 11;
    public static final int FRONT_RIGHT_CANCODER_ID = 12;
    public static final int REAR_LEFT_CANCODER_ID = 13;
    public static final int REAR_RIGHT_CANCODER_ID = 14;

    public static final double FRONT_LEFT_ZERO_ANGLE = 169.716796875;
    public static final double FRONT_RIGHT_ZERO_ANGLE = -76.46484375;
    public static final double REAR_LEFT_ZERO_ANGLE = 46.58203125;
    public static final double REAR_RIGHT_ZERO_ANGLE = -78.57421875 + 90;

    public static final boolean FRONT_LEFT_CANCODER_REVERSED = false;
    public static final boolean FRONT_RIGHT_CANCODER_REVERSED = false;
    public static final boolean REAR_LEFT_CANCODER_REVERSED = false;
    public static final boolean REAR_RIGHT_CANCODER_REVERSED = false;
    
    public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = true; 
    public static final boolean REAR_LEFT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean REAR_RIGHT_DRIVE_ENCODER_REVERSED = true;
    
    public static final double TURN_S = 0.77;
    public static final double TURN_V = 0.75;
    public static final double TURN_A = 0;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI * 4;

    public static final double MAX_SPEED_METERS_PER_SECOND = 4.5;
  }
  
  public static final class ModuleConstants { 
    public static final double DRIVE_GEAR_RATIO = 7.36;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVE_TO_METERS = 
      WHEEL_CIRCUMFERENCE_METERS / (DRIVE_GEAR_RATIO * HardwareConstants.FALCON_ENCODER_RESOLUTION);
    public static final double DRIVE_TO_METERS_PER_SECOND =
      (10 * WHEEL_CIRCUMFERENCE_METERS) / (DRIVE_GEAR_RATIO * HardwareConstants.FALCON_ENCODER_RESOLUTION);

    public static final double TURN_P = 7; // 8.1
    public static final double TURN_I = 0;
    public static final double TURN_D = 0;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 10 * Math.PI;
    public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 8 * Math.PI;
    public static final TrapezoidProfile.Constraints TURN_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
        MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED
      );

    public static final double DRIVE_F = 0; // 0.059;
    public static final double DRIVE_P = 0.3; // 0.19;
    public static final double DRIVE_I = 0;
    public static final double DRIVE_D = 0;
    public static final double DRIVE_S = 0.29155 / 12.0;
    public static final double DRIVE_V = 2.3621 / 12.0;
    public static final double DRIVE_A = 0.72606 / 12.0;
  }

  public static final class LimelightConstants {
    public static final int FRAMES_BEFORE_ADDING_VISION_MEASUREMENT = 5;
  
    public static final String FRONT_LIMELIGHT_NAME = "limelight-front";
    public static final String BACK_LIMELIGHT_NAME = "limelight-back";

    public static final double[][] APRIL_TAG_POSITIONS = {
      // { x, y, z}
      {Units.inchesToMeters(610.77), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22)}, // 1
      {Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22)}, // 2
      {Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22)}, // 3
      {Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38)}, // 4
      {Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38)}, // 5
      {Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22)}, // 6
      {Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22)}, // 7
      {Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22)} // 8
    };

    public static final double[][] CAMERA_CROP_LOOKUP_TABLE = {
      // TODO: All of these are placeholder values
      // {x position in meters, limelight lower y crop}
      {0, -1},
      {1, -.5},
      {2, -.25},
      {3, 0},
      {4, .25}
    };

    public static final double[][] ONE_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0, 0.01, 0.01, 10},
      {1.5, 0.01, 0.01, 10},
      {3, 0.145, 1.20, 30},
      {4.5, 0.75, 5.0, 90},
      {6, 1.0, 8.0, 180}
    };

    public static final double[][] TWO_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0, 0.01, 0.01, 5},
      {1.5, 0.02, 0.02, 5},
      {3, 0.04, 0.04, 15},
      {4.5, 0.1, 0.1, 30},
      {6, 0.3, 0.3, 60}
    };
  }

  public static final class TrajectoryConstants {
    public static final double MAX_SPEED = 4;
    public static final double MAX_ACCELERATION = 3;
    public static final double DEPLOYED_X_CONTROLLER_P = .35;
    public static final double DEPLOYED_Y_CONTROLLER_P = .35;
    public static final double DEPLOYED_THETA_CONTROLLER_P = .8;
    public static final double REAL_TIME_X_CONTROLLER_P = 2;
    public static final double REAL_TIME_Y_CONTROLLER_P = 2;
    public static final double REAL_TIME_THETA_CONTROLLER_P = 3;
    public static final double THETA_PROFILED_CONTROLLER_P = 1;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI ;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 2 * Math.PI;
    // The length of the field in the x direction (left to right)
    public static final double FIELD_LENGTH_METERS = 16.54175;
    // The length of the field in the y direction (top to bottom)
    public static final double FIELD_WIDTH_METERS = 8.0137;
  
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
      new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    public static final double X_TOLERANCE = 0.02;
    public static final double Y_TOLERANCE = 0.02;
    public static final double THETA_TOLERANCE = 1.25;
  
    // These are ordered from top left to bottom right from driver perspective
    public static final double[] BLUE_NODE_Y_POSITIONS = 
      {4.9784, 4.4196, 3.8608, 3.302, 2.7432, 2.1844, 1.6256, 1.0668, 0.508};
    public static final double[] RED_NODE_Y_POSITIONS = 
      {0.508, 1.0668, 1.6256, 2.1844, 2.7432, 3.302, 3.8608, 4.4196, 4.9784};

    // Factors in bumper width and wheelbase
    public static final double BLUE_NODE_X_POSITION = 1.92;
    public static final double RED_NODE_X_POSITION = 14.61;
    public static final double BLUE_OUTER_WAYPOINT_X = 5.3;
    public static final double RED_OUTER_WAYPOINT_X = 11.26;
    public static final double BLUE_INNER_WAYPOINT_X = 2.5;
    public static final double RED_INNER_WAYPOINT_X = 14.26;
    public static final double UPPER_WAYPOINT_Y = 4.75;
    public static final double LOWER_WAYPOINT_Y = 0.75;
    public static final Rotation2d BLUE_END_ROTATION = Rotation2d.fromDegrees(0);
    public static final Rotation2d BLUE_HEADING = Rotation2d.fromDegrees(180);
    public static final Rotation2d RED_END_ROTATION = Rotation2d.fromDegrees(180);
    public static final Rotation2d RED_HEADING = Rotation2d.fromDegrees(0);
  }
}
