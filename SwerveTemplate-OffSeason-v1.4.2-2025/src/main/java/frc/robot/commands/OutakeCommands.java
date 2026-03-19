package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanism.Outake;
import java.util.function.Supplier;

public class OutakeCommands extends Command {
  private final Outake outake;
  private Supplier<Double> Disp, Apunt;

  public OutakeCommands(Outake outake, Supplier<Double> Disp, Supplier<Double> Apunt) {
    this.Disp = Disp; // LT
    this.Apunt = Apunt; // RT
    this.outake = outake;
    addRequirements(outake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    outake.OutakeControlAtras(Disp.get() * 0.2); // acomodar hacia atras // 0.4
    outake.OutakeControlAdelante(Apunt.get() * 0.6); // disprar hacia adelante // 0.7 , 0.55
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
