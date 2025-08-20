package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.OutakeCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.mechanism.Elevator;
import frc.robot.subsystems.mechanism.Outake;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator elevator = new Elevator();
  private final Outake outake = new Outake();

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick controller2 = new Joystick(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand(
        "OutakeFull", new InstantCommand(() -> outake.OutakeController(1), outake));
    NamedCommands.registerCommand(
        "Outake Mid", new InstantCommand(() -> outake.OutakeController(0.5), outake));
    NamedCommands.registerCommand(
        "OutakeStop", new InstantCommand(() -> outake.OutakeController(0), outake));
    NamedCommands.registerCommand(
        "ElevatorL2", new InstantCommand(() -> elevator.PstDist(-65), elevator));
    NamedCommands.registerCommand(
        "ElevatorHome", new InstantCommand(() -> elevator.PstDist(0), elevator));

    // Intake
    outake.setDefaultCommand(
        new OutakeCommands(
            outake,
            () -> controller2.getRawAxis(2), // LT
            () -> controller2.getRawAxis(3))); // RT

    elevator.setDefaultCommand(
        new ElevatorCommand(
            elevator,
            () -> controller2.getRawButton(1), // home     /A
            () -> controller2.getRawButton(3), // Pstn1    /X
            () -> controller2.getRawButton(4), // Pstn2    /Y
            () -> controller2.getRawButton(6), // BlqFree  /RB
            () -> controller2.getRawAxis(5))); // FreeMtn  /LY

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
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

    /// Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> controller.getLeftY(),
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
  }

  public Command getAutonomousCommand() {
    // return DriveCommands.drivefor(drive, 3);
    return autoChooser.get();
  }
}
