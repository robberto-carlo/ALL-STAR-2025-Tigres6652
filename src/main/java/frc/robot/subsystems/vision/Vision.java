package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private final PhotonCamera camera;

  // private double lastYaw = 0.0;
  private double lastDistanceFrontal = 0.0;
  private double lastDistanceLateral = 0.0;
  private double lastYawRotation = 0.0;
  private double lastDetectionTime = 0.0;

  private static final double MAX_DATA_TIME = 0.1;

  public Vision(String cameraName) {
    this.camera = new PhotonCamera(cameraName);
  }

  @Override
  public void periodic() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      // lastYaw = target.getYaw();
      Transform3d camToTarget = target.getBestCameraToTarget();
      lastDistanceLateral = camToTarget.getY(); // izquierda/derecha
      lastDistanceFrontal = camToTarget.getX(); // distancia al frente

      Rotation3d rot = camToTarget.getRotation();
      lastYawRotation = Math.toDegrees(rot.getZ());

      lastDetectionTime = Timer.getFPGATimestamp();
    }
  }

  public double getDistanceLateralMeters() {
    return lastDistanceLateral;
  }

  public double getDistanceFrontalMeters() {
    return lastDistanceFrontal;
  }

  public double getRotationGrados() {
    return lastYawRotation;
  }

  public boolean hasRecentTarget() {
    return (Timer.getFPGATimestamp() - lastDetectionTime) < MAX_DATA_TIME;
  }

  /*public double getYaw() {
    return lastYaw;
  }*/

  public boolean hasTarget() {
    var result = camera.getLatestResult();
    return result != null && result.hasTargets();
  }
}
