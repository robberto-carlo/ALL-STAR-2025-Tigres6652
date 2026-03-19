package frc.robot.subsystems.autoaim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;

public class AutoAim extends SubsystemBase {
  private final Drive drive;
  private Pose2d targetPose;
  private final List<Zone> zones = new ArrayList<>();

  private PIDController xController, yController, rotController;
  private double KP = 3.1, KI = 0.0, KD = 0.0; // KP 3.2

  private static final Pose2d NO_TARGET = new Pose2d(-1, -1, new Rotation2d(0));
  private static final double MAX_OUTPUT = 0.2;
  private static final double ERROR = 0.05; // en metros (x,y)
  private static final double ERROR_DEGREES = 2; // en grados (rotacion)

  private Zone currentZone = null;
  private boolean autoDriving = false;

  public AutoAim(Drive drive) {
    this.drive = drive;
    configurarPID();
    resetPID();
    createZones();
  }

  private void configurarPID() {
    xController = new PIDController(KP, KI, KD);
    yController = new PIDController(KP, KI, KD);
    rotController = new PIDController(KP, KI, KD);
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(ERROR);
    yController.setTolerance(ERROR);
    rotController.setTolerance(Math.toRadians(ERROR_DEGREES));
  }

  private void resetPID() {
    xController.reset();
    yController.reset();
    rotController.reset();
  }

  private void createZones() {
    zones.add(
        new Zone( // Zona 1
            0,
            5,
            0,
            5, // límites xMin, xMax, yMin, yMax
            2,
            2,
            Rotation2d.fromDegrees(45), // Posicion objetivo en RB
            4,
            4,
            Rotation2d.fromDegrees(90) // Posicion objetivo en LB
            ));
    zones.add(
        new Zone( // Zona 2
            5,
            10,
            0,
            5, // límites xMin, xMax, yMin, yMax
            7,
            2,
            Rotation2d.fromDegrees(30), // Posicion objetivo en RB
            9,
            4,
            Rotation2d.fromDegrees(60) // Posicion objetivo en LB
            ));
  }

  public void moveToPoint(int objetivoEspecifico) {
    targetPose = setSpecificPose(objetivoEspecifico);
    moveToPose();
  }

  private Pose2d setSpecificPose(int objetivoEspecifico) {
    switch (objetivoEspecifico) {
      case 1:
        return new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(0));
      default:
        return NO_TARGET;
    }
  }

  public void moveToTarget(int objetivo) {
    targetPose = setTargetPose(objetivo);
    moveToPose();
  }

  private Pose2d setTargetPose(int objetivo) {
    Pose2d currentPose = drive.getPose();
    autoDriving = false;
    for (Zone z : zones) {
      if (z.contiene(currentPose.getX(), currentPose.getY())) {
        if (currentZone != z) {
          resetPID();
          currentZone = z;
        }
        autoDriving = true;
        switch (objetivo) {
          case 1:
            return new Pose2d(z.targetLBX, z.targetLBY, z.targetLBRotation); // LB
          case 2:
            return new Pose2d(z.targetRBX, z.targetRBY, z.targetRBRotation); // RB
          default:
            return NO_TARGET;
        }
      }
    }
    detener();
    return NO_TARGET;
  }

  private void moveToPose() {
    if (targetPose == null) return;
    autoDriving = !targetPose.equals(NO_TARGET);
    if (!autoDriving) {
      detener();
      return;
    }
    if (atSetpointFinal()) {
      detener();
      return;
    }
    OutputDrive output = calculateOutput();
    DriveCommands.joystickDrive(drive, () -> output.dx, () -> output.dy, () -> output.drot)
        .execute();
  }

  private OutputDrive calculateOutput() {
    Pose2d currentPose = drive.getPose();

    if (targetPose.equals(NO_TARGET)) {
      return new OutputDrive(0, 0, 0);
    }

    double errorX = xController.calculate(currentPose.getX(), targetPose.getX());
    double errorY = yController.calculate(currentPose.getY(), targetPose.getY());
    double errorRot =
        rotController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    double outputX = xController.atSetpoint() ? 0 : limitOutput(errorX, MAX_OUTPUT);
    double outputY = yController.atSetpoint() ? 0 : limitOutput(errorY, MAX_OUTPUT);
    double outputRot = rotController.atSetpoint() ? 0 : limitOutput(errorRot, MAX_OUTPUT);

    return new OutputDrive(outputX, outputY, outputRot);
  }

  public void detener() {
    autoDriving = false;
    currentZone = null;
    drive.runVelocity(new ChassisSpeeds(0, 0, 0));
  }

  public boolean atSetpointFinal() {
    return xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint();
  }

  private double limitOutput(double value, double limit) {
    return Math.max(-limit, Math.min(limit, value));
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("AutoAim Setpoint", atSetpointFinal());
    if (targetPose != null) {
      SmartDashboard.putNumber("Target X", targetPose.getX());
      SmartDashboard.putNumber("Target Y", targetPose.getY());
      SmartDashboard.putNumber("Target Rotation", targetPose.getRotation().getDegrees());
    } else {
      SmartDashboard.putNumber("Target X", 0);
      SmartDashboard.putNumber("Target Y", 0);
      SmartDashboard.putNumber("Target Rotation", 0);
    }
  }

  public static class Zone {
    public double xMin, xMax, yMin, yMax;
    public double targetRBX, targetRBY;
    public Rotation2d targetRBRotation;
    public double targetLBX, targetLBY;
    public Rotation2d targetLBRotation;

    public Zone(
        double xMin,
        double xMax,
        double yMin,
        double yMax,
        double targetRBX,
        double targetRBY,
        Rotation2d targetRBRotation,
        double targetLBX,
        double targetLBY,
        Rotation2d targetLBRotation) {
      this.xMin = xMin;
      this.xMax = xMax;
      this.yMin = yMin;
      this.yMax = yMax;

      this.targetRBX = targetRBX;
      this.targetRBY = targetRBY;
      this.targetRBRotation = targetRBRotation;

      this.targetLBX = targetLBX;
      this.targetLBY = targetLBY;
      this.targetLBRotation = targetLBRotation;
    }

    public boolean contiene(double x, double y) {
      return x >= xMin && x <= xMax && y >= yMin && y <= yMax;
    }
  }

  private static class OutputDrive {
    public final double dx, dy, drot;

    public OutputDrive(double dx, double dy, double drot) {
      this.dx = dx;
      this.dy = dy;
      this.drot = drot;
    }
  }
}
