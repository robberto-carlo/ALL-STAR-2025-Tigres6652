package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.autoaim.AutoAim;
import frc.robot.subsystems.drive.Drive;

public class AutoAimCommand extends Command {
  private final AutoAim autoAim;
  private int objetivo, objetivoEspecifico;

  public AutoAimCommand(AutoAim autoAim, Drive drive, int objetivo, int objetivoEspecifico) {
    this.autoAim = autoAim;
    this.objetivo = objetivo;
    this.objetivoEspecifico = objetivoEspecifico;
    addRequirements(autoAim, drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (objetivo != 0) {
      autoAim.moveToTarget(objetivo);
    } else if (objetivoEspecifico != 0) {
      autoAim.moveToPoint(objetivoEspecifico);
    }
  }

  @Override
  public void end(boolean interrupted) {
    autoAim.detener();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
