package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.extras.MultiLinearInterpolator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public abstract class DriveCommandBase extends CommandBase {

  private final MultiLinearInterpolator oneAprilTagLookupTable = 
    new MultiLinearInterpolator(LimelightConstants.ONE_APRIL_TAG_LOOKUP_TABLE);
  private final MultiLinearInterpolator twoAprilTagLookupTable = 
    new MultiLinearInterpolator(LimelightConstants.TWO_APRIL_TAG_LOOKUP_TABLE);

  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private double currentTimestampSeconds = 0;
  private double lastTimestampSeconds = 0;

  /**
   * An abstract class that handles pose estimation while driving.
   * @param driveSubsystem The subsystem for the swerve drive
   * @param visionSubsystem The subsystem for vision measurements
   */
  public DriveCommandBase(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  @Override
  public void execute() {
    // Updates the pose estimator using the swerve modules
    driveSubsystem.addPoseEstimatorSwerveMeasurement();

    if (visionSubsystem.canSeeAprilTags()) {
      currentTimestampSeconds = visionSubsystem.getTimeStampSeconds();
      // Only updates the pose estimator if the limelight pose is new
      if (currentTimestampSeconds > lastTimestampSeconds) {
        double distanceFromClosestAprilTag = visionSubsystem.getDistanceFromClosestAprilTag();
        // Sets the pose estimator confidence in vision based off of number of april tags and distance
        if (visionSubsystem.getNumberOfAprilTags() == 1) {
          double[] standardDeviations = oneAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag);
          driveSubsystem.setPoseEstimatorVisionConfidence(standardDeviations[0], standardDeviations[1], standardDeviations[2]);
        } else if (visionSubsystem.getNumberOfAprilTags() > 1) {
          double[] standardDeviations = twoAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag);
          driveSubsystem.setPoseEstimatorVisionConfidence(standardDeviations[0], standardDeviations[1], standardDeviations[2]);
        }
        // Updates the pose estimator with the april tag vision measurement
        Pose2d limelightVisionMeasurement = visionSubsystem.getPoseFromAprilTags();
        driveSubsystem.addPoseEstimatorVisionMeasurement(limelightVisionMeasurement, Timer.getFPGATimestamp() - visionSubsystem.getLatencySeconds());
      }
    }

    lastTimestampSeconds = currentTimestampSeconds;
  }

}