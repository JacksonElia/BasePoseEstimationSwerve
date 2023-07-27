package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.extras.LimelightHelpers;
import frc.robot.extras.LimelightHelpers.LimelightResults;
import frc.robot.extras.LimelightHelpers.LimelightTarget_Fiducial;

public class VisionSubsystem extends SubsystemBase {

  private LimelightResults currentlyUsedLimelightResults;
  private LimelightResults frontLimelightResults = LimelightHelpers.getLatestResults(LimelightConstants.FRONT_LIMELIGHT_NAME);;
  private LimelightResults backLimelightResults = LimelightHelpers.getLatestResults(LimelightConstants.BACK_LIMELIGHT_NAME);;
  private String currentlyUsedLimelight = LimelightConstants.FRONT_LIMELIGHT_NAME;
  private boolean wasFrontLimelightUsedLast = false;
  
  public VisionSubsystem() {
    currentlyUsedLimelightResults = LimelightHelpers.getLatestResults(LimelightConstants.FRONT_LIMELIGHT_NAME);
  }

  /**
   * Returns true if the limelight(s) can see one or more April Tag.
   */
  public boolean canSeeAprilTags() {
    return LimelightHelpers.getFiducialID(currentlyUsedLimelight) != -1;
  }

  /**
   * Returns the pose of the robot calculated by the limelight. If there
   * are multiple limelights that can see april tags, it uses the limelight
   * that is closest to an april tag. 
   */
  public Pose2d getPoseFromAprilTags() {
    Pose2d botPose = LimelightHelpers.getBotPose2d(currentlyUsedLimelight);
    // The origin of botpose is at the center of the field
    double robotX = botPose.getX() + TrajectoryConstants.FIELD_LENGTH_METERS / 2;
    double robotY = botPose.getY() + TrajectoryConstants.FIELD_WIDTH_METERS / 2;
    Rotation2d robotRotation = botPose.getRotation();
    return new Pose2d(robotX, robotY, robotRotation);
  }

  /**
   * Returns the distance in meters from the limelight(s) to the closest 
   * april tag that they can see.
   */
  public double getDistanceFromClosestAprilTag() {
    if (canSeeAprilTags()) {
      int closestAprilTagID = (int) LimelightHelpers.getFiducialID(currentlyUsedLimelight);
      return getLimelightAprilTagDistance(closestAprilTagID);
    }
    
    // To be safe returns a big distance from the april tags
    return Double.MAX_VALUE;
  }

  /**
   * Returns how many april tags the limelight that is being used for pose
   * estimation can see.
   */
  public int getNumberOfAprilTags() {
    return currentlyUsedLimelightResults.targetingResults.targets_Fiducials.length;
  }

  /**
   * Returns the timestamp in seconds of when the limelight that is being
   * used for pose estimation calculated the robot's pose. 
   */
  public double getTimeStampSeconds() {
    return currentlyUsedLimelightResults.targetingResults.timestamp_LIMELIGHT_publish / 1000.0;
  }

  /**
   * Returns the latency in seconds of when the limelight that is being
   * used for pose estimation calculated the robot's pose. It adds the
   * pipeline latency, capture latency, and json parsing latency.
   */
  public double getLatencySeconds() {
    return (currentlyUsedLimelightResults.targetingResults.latency_capture 
    + currentlyUsedLimelightResults.targetingResults.latency_pipeline 
    + currentlyUsedLimelightResults.targetingResults.latency_jsonParse) / 1000.0;
  }

  /**
   * Crops the limelights to the specified values.
   * @param cropValues A 2D array that with each row containing the crop
   * values for each limelight as {x1, x2, y1, y2}. Values should be from
   * -1 to 1.
   */
  public void cropLimelights(double[][] cropValues) {
    LimelightHelpers.setCropWindow(
      LimelightConstants.FRONT_LIMELIGHT_NAME, 
      cropValues[0][0],
      cropValues[0][1],
      cropValues[0][2],
      cropValues[0][3]
    );
  }

  /**
   * Calculates the distance between the specified robot and april tag.
   * This method should only be called once there has been a check for if
   * the limelights can see april tags.
   */
  private double getLimelightAprilTagDistance(int aprilTagID) {
    if (aprilTagID >= 1) {
      double aprilTagX = LimelightConstants.APRIL_TAG_POSITIONS[aprilTagID - 1][0]; // April tag id starts at 1
      double aprilTagY = LimelightConstants.APRIL_TAG_POSITIONS[aprilTagID - 1][1];
      Pose2d pose = getPoseFromAprilTags();
      double robotX = pose.getX();
      double robotY = pose.getY();
      return Math.sqrt(Math.pow(aprilTagX - robotX, 2) + Math.pow(aprilTagY - robotY, 2));
    }

    // To be safe returns a big distance from the april tags
    return Double.MAX_VALUE;
  }

  @Override
  public void periodic() {
    // Every periodic chooses the limelight to use based off of their distance from april tags
    if (wasFrontLimelightUsedLast) {
      backLimelightResults = LimelightHelpers.getLatestResults(LimelightConstants.BACK_LIMELIGHT_NAME);
    } else {
      frontLimelightResults = LimelightHelpers.getLatestResults(LimelightConstants.FRONT_LIMELIGHT_NAME);
    }

    wasFrontLimelightUsedLast = !wasFrontLimelightUsedLast;

    LimelightTarget_Fiducial[] frontLimelightAprilTags = frontLimelightResults.targetingResults.targets_Fiducials;
    LimelightTarget_Fiducial[] backLimelightAprilTags = backLimelightResults.targetingResults.targets_Fiducials;

    // Gets the distance from the closest april tag. If it can't see one, returns a really big number.
    double frontLimelightDistance = frontLimelightAprilTags.length > 0
      ? getLimelightAprilTagDistance((int) frontLimelightAprilTags[0].fiducialID) : Double.MAX_VALUE;
    double backLimelightDistance = backLimelightAprilTags.length > 0
      ? getLimelightAprilTagDistance((int) backLimelightAprilTags[0].fiducialID) : Double.MAX_VALUE;

    currentlyUsedLimelight = frontLimelightDistance <= backLimelightDistance 
      ? LimelightConstants.FRONT_LIMELIGHT_NAME : LimelightConstants.BACK_LIMELIGHT_NAME;
    currentlyUsedLimelightResults = currentlyUsedLimelight == LimelightConstants.FRONT_LIMELIGHT_NAME
      ? frontLimelightResults : backLimelightResults;

    // Flashes the limelight LEDs if they can't see an april tag
    if (!canSeeAprilTags()) {
      LimelightHelpers.setLEDMode_ForceOn(LimelightConstants.FRONT_LIMELIGHT_NAME);
      LimelightHelpers.setLEDMode_ForceOn(LimelightConstants.BACK_LIMELIGHT_NAME);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(LimelightConstants.FRONT_LIMELIGHT_NAME);
      LimelightHelpers.setLEDMode_ForceOff(LimelightConstants.BACK_LIMELIGHT_NAME);
    }
  }

}