package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  // This will stay the same throughout the match. These values are harder to test for and tune, so assume this guess is right.
  private final Vector<N3> stateStandardDeviations = VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(1));
  // This will be changed throughout the match depending on how confident we are that the limelight is right.
  private final Vector<N3> visionMeasurementStandardDeviations = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(50));

  private final SwerveModule frontLeftSwerveModule;
  private final SwerveModule frontRightSwerveModule;
  private final SwerveModule rearLeftSwerveModule;
  private final SwerveModule rearRightSwerveModule;

  private final Gyro gyro;
  private final SwerveDrivePoseEstimator odometry;

  private double gyroOffset = 0;
  
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    frontLeftSwerveModule = new SwerveModule(
      DriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
      DriveConstants.FRONT_LEFT_TURN_MOTOR_ID,
      DriveConstants.FRONT_LEFT_CANCODER_ID,
      DriveConstants.FRONT_LEFT_ZERO_ANGLE,
      DriveConstants.FRONT_LEFT_CANCODER_REVERSED,
      DriveConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED,
      "FL"
    );
    
    frontRightSwerveModule = new SwerveModule(
      DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
      DriveConstants.FRONT_RIGHT_TURN_MOTOR_ID,
      DriveConstants.FRONT_RIGHT_CANCODER_ID,
      DriveConstants.FRONT_RIGHT_ZERO_ANGLE,
      DriveConstants.FRONT_RIGHT_CANCODER_REVERSED,
      DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED,
      "FR"
    );
    
    rearLeftSwerveModule = new SwerveModule(
      DriveConstants.REAR_LEFT_DRIVE_MOTOR_ID,
      DriveConstants.REAR_LEFT_TURN_MOTOR_ID,
      DriveConstants.REAR_LEFT_CANCODER_ID,
      DriveConstants.REAR_LEFT_ZERO_ANGLE,
      DriveConstants.REAR_LEFT_CANCODER_REVERSED,
      DriveConstants.REAR_LEFT_DRIVE_ENCODER_REVERSED,
      "RL"
    );
    
    rearRightSwerveModule = new SwerveModule(
      DriveConstants.REAR_RIGHT_DRIVE_MOTOR_ID,
      DriveConstants.REAR_RIGHT_TURN_MOTOR_ID,
      DriveConstants.REAR_RIGHT_CANCODER_ID,
      DriveConstants.REAR_RIGHT_ZERO_ANGLE,
      DriveConstants.REAR_RIGHT_CANCODER_REVERSED,
      DriveConstants.REAR_RIGHT_DRIVE_ENCODER_REVERSED,
      "RR"
    );

    gyro = new AHRS(SPI.Port.kMXP);
  
    odometry = new SwerveDrivePoseEstimator(
      DriveConstants.DRIVE_KINEMATICS,
      getRotation2d(),
      getModulePositions(),
      new Pose2d(), // This is the position for where the robot starts the match
      stateStandardDeviations,
      visionMeasurementStandardDeviations
    );
  }

  @SuppressWarnings("ParameterName")
  /**
   * Drives the robot using the joysticks.
   * @param xSpeed Speed of the robot in the x direction, positive being 
   * forwards.
   * @param ySpeed Speed of the robot in the y direction, positive being
   * left.
   * @param rotationSpeed Angular rate of the robot in radians per second.
   * @param fieldRelative Whether the provided x and y speeds are relative
   * to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
      fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getFieldRelativeRotation2d())
      : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    
    frontLeftSwerveModule.setDesiredState(swerveModuleStates[0]);
    frontRightSwerveModule.setDesiredState(swerveModuleStates[1]);
    rearLeftSwerveModule.setDesiredState(swerveModuleStates[2]);
    rearRightSwerveModule.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Returns the heading of the robot in degrees from 0 to 360. 
   * Counter-clockwise is positive. This factors in gyro offset.
   */
  public double getHeading() {
    return (-gyro.getAngle() + this.gyroOffset) % 360;
  }

  /**
   * Returns a Rotation2d for the heading of the robot.
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  /**
   * Returns a Rotation2d for the heading of the robot relative to the
   * field from the driver's perspective. This method is needed so that the
   * drive command and poseEstimator don't fight each other.
   */
  public Rotation2d getFieldRelativeRotation2d() {
    // Because the field isn't vertically symmetrical, we have the pose coordinates always start from the bottom left
    return Rotation2d.fromDegrees((getHeading() + (DriverStation.getAlliance() == Alliance.Blue ? 0 : 180)) % 360);
  }

  /**
   * Sets the offset of the gyro.
   * @param gyroOffset The number of degrees that will be added to the
   * gyro's angle in getHeading.
   */
  public void setGyroOffset(double gyroOffset) {
    this.gyroOffset = gyroOffset;
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    gyroOffset = DriverStation.getAlliance() == Alliance.Blue ? 0 : 180;
    gyro.reset();
  }

  /**
   * Returns the estimated field-relative pose of the robot. Positive x 
   * being forward, positive y being left.
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  /**
   * Updates the pose estimator with the pose calculated from the swerve
   * modules.
   */
  public void addPoseEstimatorSwerveMeasurement() {
    odometry.update(
      getRotation2d(),
      getModulePositions()
    );
  }
  
  /**
   * Updates the pose estimator with the pose calculated from the april
   * tags. How much it contributes to the pose estimation is set by
   * setPoseEstimatorVisionConfidence.
   * @param visionMeasurement The pose calculated from the april tags
   * @param currentTimeStampSeconds The time stamp in seconds of when the
   * pose from the april tags was calculated.
   */
  public void addPoseEstimatorVisionMeasurement(Pose2d visionMeasurement, double currentTimeStampSeconds) {
    odometry.addVisionMeasurement(visionMeasurement, currentTimeStampSeconds);
  }

  /**
   * Resets the position to the specified pose, but keeps the current 
   * rotation.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Resets the position and rotation rather than just the position.
   */
  public void resetOdometryAndRotation(Pose2d pose, double angle) {
    zeroHeading();
    setGyroOffset(angle);
    odometry.resetPosition(pose.getRotation(), getModulePositions(), pose);
  }

  /**
   * Sets the standard deviations of model states, or how much the april
   * tags contribute to the pose estimation of the robot. Lower numbers
   * equal higher confidence and vice versa.
   * @param xStandardDeviation the x standard deviation in meters
   * @param yStandardDeviation the y standard deviation in meters
   * @param thetaStandardDeviation the theta standard deviation in radians
   */
  public void setPoseEstimatorVisionConfidence(double xStandardDeviation, double yStandardDeviation,
    double thetaStandardDeviation) {
    odometry.setVisionMeasurementStdDevs(VecBuilder.fill(xStandardDeviation, yStandardDeviation, thetaStandardDeviation));
  }
  
  /**
   * Returns the current drivetrain position, as reported by the modules 
   * themselves. The order is: frontLeft, frontRight, backLeft, backRight
   * (should be the same as the kinematics).
   */
   public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] swerveModulePositions = {
      frontLeftSwerveModule.getPosition(),
      frontRightSwerveModule.getPosition(),
      rearLeftSwerveModule.getPosition(),
      rearRightSwerveModule.getPosition()
    };

    return swerveModulePositions;
  }

  /**
   * Sets the modules to the specified states.
   * @param desiredStates The desired states for the swerve modules. The
   * order is: frontLeft, frontRight, backLeft, backRight (should be the 
   * same as the kinematics).
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeftSwerveModule.setDesiredState(desiredStates[0]);
    frontRightSwerveModule.setDesiredState(desiredStates[1]);
    rearLeftSwerveModule.setDesiredState(desiredStates[2]);
    rearRightSwerveModule.setDesiredState(desiredStates[3]);
  }

  @Override
  public void periodic() {
    frontLeftSwerveModule.periodicFunction();
    frontRightSwerveModule.periodicFunction();
    rearLeftSwerveModule.periodicFunction();
    rearRightSwerveModule.periodicFunction();
  }

}
