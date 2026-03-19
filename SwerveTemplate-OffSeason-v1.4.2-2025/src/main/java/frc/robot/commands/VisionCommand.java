package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class VisionCommand extends Command {
  private final Drive drive;
  private final Vision camara_Right;
  private final Vision camara_Left;
  private final boolean moveRight;

  private Vision activeCamera;

  private final PIDController pidLateral;
  private final PIDController pidFrontal;
  // private final PIDController pidRotation;

  private static final double MAX_LATERAL_SPEED = 0.55; // 0.55;  // m/s máximo
  private static final double MAX_FORWARD_SPEED = 0.50; // 0.50; // m/s máximo
  // private static final double MAX_ANGULAR_SPEED = 0.3; // 0.3; // rad/s maximo

  private static final double KP_LATERAL = 7;
  private static final double KP_FRONTAL = 7;
  // private static final double KP_ROTATION = 7;

  private static final double TOLERANCE_METERS_LATERAL = 0.03; // tolerancia en metros
  private static final double TOLERANCE_METERS_FRONTAL = 0.05; // tolerancia en metros
  // private static final double TOLERANCE_DEGREES_ROTATION = 0.3; // tolerancia en gardos

  private static final double TARGET_LATERAL_METERS_RIGHT_CAMARALEFT = 0.043; // en metros
  private static final double TARGET_FRONTAL_METERS_CAMARA_LEFT = 0.388; // distancia al frente
  // private static final double TARGET_ROTATION_DEGREES_CAMARA_LEFT = -171;

  private static final double TARGET_LATERAL_METERS_LEFT_CAMARARIGHT = 0.021; // en metros
  private static final double TARGET_FRONTAL_METERS_CAMARA_RIGHT = 0.426; // distancia al frente
  // private static final double TARGET_ROTATION_DEGREES_CAMARA_RIGHT = -160;

  private double frontalTarget = 0;
  private double lateralTarget = 0;
  // private double rotationTarget = 0;

  private int stableTicks = 0;
  private boolean lateralSetPoint = false;
  private boolean frontalSetPoint = false;
  private boolean rotationSetPoint = false;

  public VisionCommand(Drive drive, Vision camara_Left, Vision camara_Right, boolean moveRight) {
    this.drive = drive;
    this.camara_Left = camara_Left;
    this.camara_Right = camara_Right;
    this.moveRight = moveRight;
    addRequirements(drive);

    pidLateral = new PIDController(KP_LATERAL, 0.0, 0.0);
    pidLateral.setTolerance(TOLERANCE_METERS_LATERAL);
    pidFrontal = new PIDController(KP_FRONTAL, 0.0, 0.0);
    pidFrontal.setTolerance(TOLERANCE_METERS_FRONTAL);
    // pidRotation = new PIDController(KP_ROTATION, 0.0, 0.0);
    // pidRotation.setTolerance(TOLERANCE_DEGREES_ROTATION);
  }

  @Override
  public void initialize() {
    stableTicks = 0;
    activeCamera = null;
    lateralSetPoint = false;
    frontalSetPoint = false;
  }

  @Override
  public void execute() {
    boolean camaraLeftHasTarget = camara_Left.hasRecentTarget();
    boolean camaraRightHasTarget = camara_Right.hasRecentTarget();

    if (!camaraLeftHasTarget && !camaraRightHasTarget)
      return; // si ninguna camara detecta abriltags

    activeCamera = selectBestCamera(camaraLeftHasTarget, camaraRightHasTarget);

    if (activeCamera == null || !activeCamera.hasRecentTarget()) return;

    updateTargetsForActiveCamera();

    double lateral = activeCamera.getDistanceLateralMeters();
    double frontal = activeCamera.getDistanceFrontalMeters();
    // double rotation = activeCamera.getRotationGrados();

    double outputLateral = calculateMovimiento(lateral);
    double outputFrontal = calculateFrontalOutput(frontal);
    // double outputRotacion = calculateRotationOutput(rotation);

    drive.runVelocity(new ChassisSpeeds(outputFrontal, outputLateral, 0));

    stableTicks = (lateralSetPoint && frontalSetPoint && rotationSetPoint) ? stableTicks + 1 : 0;
  }

  private double calculateMovimiento(double lateral) {
    if ((activeCamera == camara_Left && moveRight == true)
        || (activeCamera == camara_Right && moveRight == false)) {
      return calculateLateralOutput(lateral);
    } else if (activeCamera == camara_Left && moveRight == false) {
      return MAX_LATERAL_SPEED;
    } else if (activeCamera == camara_Right && moveRight == true) {
      return -MAX_LATERAL_SPEED;
    }
    return 0;
  }

  private double calculateLateralOutput(double lateral) {
    if (lateral >= lateralTarget + TOLERANCE_METERS_LATERAL) {
      lateralSetPoint = false;
      return MAX_LATERAL_SPEED; // Mover hacia la izquierda
    } else if (lateral <= lateralTarget - TOLERANCE_METERS_LATERAL) {
      lateralSetPoint = false;
      return -MAX_LATERAL_SPEED; // Mover hacia la derecha
    } else {
      lateralSetPoint = true;
      return 0;
    }

    /*double outputLateral = pidLateral.calculate(lateral, lateralTarget);
    lateralSetPoint = pidLateral.atSetpoint();
    return Math.max(-MAX_LATERAL_SPEED, Math.min(outputLateral, MAX_LATERAL_SPEED));
    */

    /*double error = lateral - lateralTarget;
      double absError = Math.abs(error);
      if (absError <= TOLERANCE_METERS_LATERAL) {
          return 0;
      }
      double speedRatio;
      if (absError > 0.30) {
        speedRatio = 1.0; // 100% --> MAX_LATERAL_SPEED = 0.89;
    } else if (absError > 0.20) {
        speedRatio = 0.80 + 0.20 * ((absError - 0.20) / 0.10); // 100% a 80%
    } else if (absError > 0.10) {
        speedRatio = 0.65 + 0.15 * ((absError - 0.10) / 0.10); // 80% a 65%
    } else{
        speedRatio = 0.50 + 0.15 * ((absError - TOLERANCE_METERS_LATERAL) / (0.10 - TOLERANCE_METERS_LATERAL)); // 65% a 50%
    }

      speedRatio = Math.max(0.5, Math.min(speedRatio, 1.0));
      double outputLateral = MAX_LATERAL_SPEED * speedRatio;
      outputLateral = (error > 0) ? outputLateral : -outputLateral;    //outputLateral = (error > 0) ? outputLateral : -outputLateral;

      double smoothedOutput = (outputLateral * 0.7) + (lastLateralOutput * 0.3);
      lastLateralOutput = smoothedOutput;

      return smoothedOutput;
      */
  }

  private double calculateFrontalOutput(double frontal) {
    if (frontal >= frontalTarget + TOLERANCE_METERS_FRONTAL) {
      frontalSetPoint = false;
      return MAX_FORWARD_SPEED;
    } else {
      frontalSetPoint = true;
      return 0;
    }
  }

  /*private double calculateRotationOutput(double rotation) {
    if (rotation >= 0 && rotation <= 180) {
      if (rotation >= rotationTarget + TOLERANCE_DEGREES_ROTATION) {
        rotationSetPoint = false;
        return -MAX_ANGULAR_SPEED; // Girar hacia la izquierda
      } else if (rotation <= rotationTarget - TOLERANCE_DEGREES_ROTATION) {
        rotationSetPoint = false;
        return MAX_ANGULAR_SPEED; // Girar hacia la derecha
      } else {
        rotationSetPoint = true;
        return 0;
      }
    } else {
      if (rotation >= rotationTarget + TOLERANCE_DEGREES_ROTATION) {
        rotationSetPoint = false;
        return MAX_ANGULAR_SPEED;
      } else if (rotation <= rotationTarget - TOLERANCE_DEGREES_ROTATION) {
        rotationSetPoint = false;
        return -MAX_ANGULAR_SPEED;
      } else {
        rotationSetPoint = true;
        return 0;
      }
    }
  }*/

  private void updateTargetsForActiveCamera() {
    if (activeCamera == camara_Left) {
      lateralTarget = TARGET_LATERAL_METERS_RIGHT_CAMARALEFT;
      frontalTarget = TARGET_FRONTAL_METERS_CAMARA_LEFT;
      // rotationTarget = TARGET_ROTATION_DEGREES_CAMARA_LEFT;
    } else if (activeCamera == camara_Right) {
      lateralTarget = TARGET_LATERAL_METERS_LEFT_CAMARARIGHT;
      frontalTarget = TARGET_FRONTAL_METERS_CAMARA_RIGHT;
      // rotationTarget = TARGET_ROTATION_DEGREES_CAMARA_RIGHT;
    }
  }

  private Vision selectBestCamera(boolean leftHasTarget, boolean rightHasTarget) {
    if (leftHasTarget && !rightHasTarget) return camara_Left;
    if (rightHasTarget && !leftHasTarget) return camara_Right;
    if (!leftHasTarget && !rightHasTarget) return null;

    double leftError_Frontal =
        Math.abs(camara_Left.getDistanceFrontalMeters() - TARGET_FRONTAL_METERS_CAMARA_LEFT);
    double rightError_Frontal =
        Math.abs(camara_Right.getDistanceFrontalMeters() - TARGET_FRONTAL_METERS_CAMARA_RIGHT);

    double TOLERANCIA_FRONTAL = 0.15; // distacia maxima de diferencia en metros (15 cm)
    if (leftError_Frontal > TOLERANCIA_FRONTAL && rightError_Frontal <= TOLERANCIA_FRONTAL) {
      return camara_Right;
    } else if (rightError_Frontal > TOLERANCIA_FRONTAL && leftError_Frontal <= TOLERANCIA_FRONTAL) {
      return camara_Left;
    } else if (leftError_Frontal > TOLERANCIA_FRONTAL && rightError_Frontal > TOLERANCIA_FRONTAL) {
      return leftError_Frontal < rightError_Frontal ? camara_Left : camara_Right;
    }

    return moveRight ? camara_Left : camara_Right;
  }

  @Override
  public boolean isFinished() {
    return stableTicks > 1;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
