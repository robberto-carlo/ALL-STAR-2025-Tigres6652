package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.OutakeCommands;
import frc.robot.commands.QuitaAlgasCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.autoaim.AutoAim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.mechanism.Elevator;
import frc.robot.subsystems.mechanism.Outake;
import frc.robot.subsystems.mechanism.QuitaAlgas;
import frc.robot.subsystems.questnav.QuestNavSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator elevator = new Elevator();
  private final Outake outake = new Outake();
  private final QuitaAlgas quitaAlgas = new QuitaAlgas();
  private final QuestNavSubsystem questNavSubsystem;
  private final AutoAim autoAim;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick controller2 = new Joystick(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Meta
  public QuestNavSubsystem getQuestNavSubsystem() {
    return questNavSubsystem;
  }

  public Elevator getElevator() {
    return elevator;
  }

  public Drive getDrive() {
    return drive;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Intake
    outake.setDefaultCommand(
        new OutakeCommands(
            outake,
            () -> controller2.getRawAxis(2), // LT
            () -> (controller2.getRawAxis(3) - controller2.getRawAxis(2)))); // RT

    elevator.setDefaultCommand(
        new ElevatorCommand(
            elevator,
            () -> controller2.getRawButton(1), // home     /A
            () -> controller2.getRawButton(3), // Pstn1    /X
            () -> controller2.getRawButton(4), // Pstn2    /Y
            () -> controller2.getRawButton(6), // BlqFree  /RB
            () -> controller2.getRawButton(2), // setZero  /B
            () -> controller2.getRawAxis(1))); // FreeMtn  /LY

    quitaAlgas.setDefaultCommand(
        new QuitaAlgasCommand(
            quitaAlgas,
            () -> controller2.getRawAxis(1), // free     / LY
            () -> controller2.getRawButton(5), // BlqFree  / LB
            () -> controller2.getPOV())); // flechas

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    questNavSubsystem = new QuestNavSubsystem(drive.getPoseEstimator());
    drive.setQuestNavSubsystem(questNavSubsystem);
    autoAim = new AutoAim(drive);

    NamedCommands.registerCommand(
        "OutakeFull", new InstantCommand(() -> outake.OutakeController(0.7), outake));
    NamedCommands.registerCommand(
        "Outake Mid",
        new InstantCommand(() -> outake.OutakeController(0.3), outake).withTimeout(0.3));
    NamedCommands.registerCommand(
        "OutakeStop", new InstantCommand(() -> outake.OutakeController(0), outake));

    NamedCommands.registerCommand(
        "ElevadorHOME",
        new RunCommand(() -> elevator.movimientoElevadorNiveles(0), elevator)
            .until(() -> Math.abs(elevator.lecturaAltura() - 0.12) <= 0.10)
            .withTimeout(3));
    NamedCommands.registerCommand(
        "ElevadorNivel2",
        new RunCommand(() -> elevator.movimientoElevadorNiveles(1), elevator)
            .until(() -> Math.abs(elevator.lecturaAltura() - 1.12) <= 0.10)
            .withTimeout(3));
    NamedCommands.registerCommand(
        "ElevadorNivel3",
        new RunCommand(() -> elevator.movimientoElevadorNiveles(2), elevator)
            .until(() -> Math.abs(elevator.lecturaAltura() - 3.62) <= 0.10)
            .withTimeout(3));
    NamedCommands.registerCommand(
        "ElevadorResetEncoder", new InstantCommand(() -> elevator.resetEncoder(), elevator));

    NamedCommands.registerCommand(
        "AutoAimLeft",
        new RunCommand(() -> autoAim.moveToTarget(1), autoAim, drive)
            .until(() -> autoAim.atSetpointFinal())
            .withTimeout(5));
    NamedCommands.registerCommand(
        "AutoAimRight",
        new RunCommand(() -> autoAim.moveToTarget(2), autoAim, drive)
            .until(() -> autoAim.atSetpointFinal())
            .withTimeout(5));

    /// Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    // Red  +
    // Blue -
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressedzzzzzz
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Auto-apuntado
    controller
        .leftBumper()
        .whileTrue(new AutoAimCommand(autoAim, drive, 1, 0)); // LB - Auto apuntado
    controller
        .rightBumper()
        .whileTrue(new AutoAimCommand(autoAim, drive, 2, 0)); // RB - Auto apuntado
  }

  public Command getAutonomousCommand() {
    // return DriveCommands.drivefor(drive, 3);
    return autoChooser.get();
  }

  public void updateJoystickDashboard() {
    SmartDashboard.putNumber("Joystick 1 - Eje X", -controller.getLeftX());
    SmartDashboard.putNumber("Joystick 1 - Eje Y", -controller.getLeftY());
    SmartDashboard.putNumber("Joystick 2 - Eje X", -controller2.getX());
    SmartDashboard.putNumber("Joystick 2 - Eje Y", -controller2.getY());
  }
}
