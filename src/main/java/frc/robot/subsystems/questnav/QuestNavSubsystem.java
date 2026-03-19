package frc.robot.subsystems.questnav;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;

/**
 * QuestNav subsystem for vision-based pose estimation.
 *
 * <p>This subsystem integrates QuestNav vision tracking with the robot's pose estimation. It
 * provides methods to get robot pose from QuestNav and set robot pose in QuestNav.
 */
public class QuestNavSubsystem extends SubsystemBase {
  private final QuestNav questNav;
  private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  // Transform from robot center to QuestNav camera position

  // x in meters, y in meters and degrees rotation
  private static final Transform2d ROBOT_TO_QUEST = new Transform2d(0.355, 0.0, new Rotation2d());

  public QuestNavSubsystem(SwerveDrivePoseEstimator swerveDrivePoseEstimator) {
    this.swerveDrivePoseEstimator = swerveDrivePoseEstimator;
    questNav = new QuestNav();
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();

    // Log QuestNav data
    Logger.recordOutput("QuestNav/Connected", questNav.isConnected());
    Logger.recordOutput("QuestNav/Tracking", questNav.isTracking());
    Logger.recordOutput("QuestNav/Latency", questNav.getLatency());

    // Log QuestNav pose data for AdvantageScope visualization
    try {
      Pose2d currentPose = getRobotPose();
      if (currentPose != null && questNav.isTracking()) {
        double timestamp = Timer.getFPGATimestamp() - questNav.getLatency();

        double dx = currentPose.getX() - swerveDrivePoseEstimator.getEstimatedPosition().getX();
        double dy = currentPose.getY() - swerveDrivePoseEstimator.getEstimatedPosition().getY();
        double dTheta =
            Math.abs(
                currentPose
                    .getRotation()
                    .minus(swerveDrivePoseEstimator.getEstimatedPosition().getRotation())
                    .getDegrees());

        double distError = Math.hypot(dx, dy);
        double scale = 1.0;
        if (distError > 0.5) scale *= 6.0; // Si el error es > 0.5m, reducir confianza
        if (distError > 1.0) scale *= 7.0; // Si el error es > 1m, reducir mucho más
        if (dTheta > 30.0) scale *= 4.0; // Si hay diferencia >30°, también penalizar

        Matrix<N3, N1> adaptiveStdDevs =
            VecBuilder.fill(0.02 * scale, 0.02 * scale, Math.toRadians(2) * scale);
        swerveDrivePoseEstimator.addVisionMeasurement(currentPose, timestamp, adaptiveStdDevs);
      }

      // Log complete pose for 2D field visualization
      Logger.recordOutput("QuestNav/Pose", currentPose);

      // Log raw QuestNav pose (before robot transform) for comparison
      PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
      if (poseFrames.length > 0) {
        Pose2d rawQuestPose = poseFrames[poseFrames.length - 1].questPose();
        if (rawQuestPose != null) {
          Logger.recordOutput("QuestNav/RawPose", rawQuestPose);
        }
      }

    } catch (Exception e) {
      Logger.recordOutput("QuestNav/Error", e.getMessage());
    }

    // Log battery percentage if available
    questNav
        .getBatteryPercent()
        .ifPresent(battery -> Logger.recordOutput("QuestNav/BatteryPercent", battery));

    // Log frame count if available
    questNav
        .getFrameCount()
        .ifPresent(frameCount -> Logger.recordOutput("QuestNav/FrameCount", frameCount));

    // Log additional data for plotting and analysis
    try {
      PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
      Logger.recordOutput("QuestNav/PoseFrameCount", poseFrames.length);

      // Log app timestamp if available
      questNav
          .getAppTimestamp()
          .ifPresent(timestamp -> Logger.recordOutput("QuestNav/AppTimestamp", timestamp));

      // Log tracking lost counter if available
      questNav
          .getTrackingLostCounter()
          .ifPresent(counter -> Logger.recordOutput("QuestNav/TrackingLostCounter", counter));

    } catch (Exception e) {
      // Ignore errors for additional logging
    }
  }

  /**
   * Gets the latest robot pose from QuestNav.
   *
   * @return The robot's current pose, or a default pose if no data is available
   */
  public Pose2d getRobotPose() {
    try {
      PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
      if (poseFrames.length > 0) {
        Pose2d questPose = poseFrames[poseFrames.length - 1].questPose();
        if (questPose != null) {
          return questPose.transformBy(ROBOT_TO_QUEST.inverse());
        }
      }
    } catch (Exception e) {
      // Log error and return default pose
      System.err.println("Error getting QuestNav pose: " + e.getMessage());
    }
    return new Pose2d(); // Return default pose if no data is available
  }

  /**
   * Sets the robot's pose in QuestNav.
   *
   * @param robotPose The robot's pose to set in QuestNav
   */
  public void setRobotPose(Pose2d robotPose) {
    Pose2d questPose = robotPose.transformBy(ROBOT_TO_QUEST);
    questNav.setPose(questPose);
  }

  /**
   * Checks if QuestNav is connected and active.
   *
   * @return true if QuestNav is connected, false otherwise
   */
  public boolean isActive() {
    return questNav.isConnected();
  }

  /**
   * Checks if QuestNav is currently tracking.
   *
   * @return true if QuestNav is tracking, false otherwise
   */
  public boolean isTracking() {
    return questNav.isTracking();
  }

  /**
   * Gets all unread pose frames from QuestNav.
   *
   * @return Array of unread pose frames
   */
  public PoseFrame[] getAllUnreadPoseFrames() {
    return questNav.getAllUnreadPoseFrames();
  }

  /**
   * Gets the current latency of QuestNav in seconds.
   *
   * @return Latency in seconds
   */
  public double getLatency() {
    return questNav.getLatency();
  }

  /**
   * Gets the battery percentage of the QuestNav device.
   *
   * @return Optional containing battery percentage if available
   */
  public java.util.OptionalInt getBatteryPercent() {
    return questNav.getBatteryPercent();
  }

  /**
   * Gets the current frame count from QuestNav.
   *
   * @return Optional containing frame count if available
   */
  public java.util.OptionalInt getFrameCount() {
    return questNav.getFrameCount();
  }

  /**
   * Gets the tracking lost counter from QuestNav.
   *
   * @return Optional containing tracking lost counter if available
   */
  public java.util.OptionalInt getTrackingLostCounter() {
    return questNav.getTrackingLostCounter();
  }

  /**
   * Gets the app timestamp from QuestNav.
   *
   * @return Optional containing app timestamp if available
   */
  public java.util.OptionalDouble getAppTimestamp() {
    return questNav.getAppTimestamp();
  }

  /**
   * Gets the current yaw (heading) from QuestNav in degrees.
   *
   * @return Current yaw in degrees, or 0.0 if no data available
   */
  public double getCurrentYawDegrees() {
    try {
      Pose2d currentPose = getRobotPose();
      return currentPose.getRotation().getDegrees();
    } catch (Exception e) {
      System.err.println("Error getting QuestNav yaw degrees: " + e.getMessage());
      return 0.0;
    }
  }

  /**
   * Gets the current yaw (heading) from QuestNav in radians.
   *
   * @return Current yaw in radians, or 0.0 if no data available
   */
  public double getCurrentYawRadians() {
    try {
      Pose2d currentPose = getRobotPose();
      return currentPose.getRotation().getRadians();
    } catch (Exception e) {
      System.err.println("Error getting QuestNav yaw radians: " + e.getMessage());
      return 0.0;
    }
  }

  /**
   * Gets the current rotation from QuestNav.
   *
   * @return Current rotation, or zero rotation if no data available
   */
  public Rotation2d getCurrentRotation() {
    try {
      Pose2d currentPose = getRobotPose();
      return currentPose.getRotation();
    } catch (Exception e) {
      System.err.println("Error getting QuestNav rotation: " + e.getMessage());
      return new Rotation2d();
    }
  }

  /**
   * Gets the current position from QuestNav.
   *
   * @return Current position, or zero position if no data available
   */
  public Translation2d getCurrentPosition() {
    try {
      Pose2d currentPose = getRobotPose();
      return currentPose.getTranslation();
    } catch (Exception e) {
      System.err.println("Error getting QuestNav position: " + e.getMessage());
      return new Translation2d();
    }
  }
}
