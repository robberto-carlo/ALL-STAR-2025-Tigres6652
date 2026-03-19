package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanism.QuitaAlgas;
import java.util.function.Supplier;

public class QuitaAlgasCommand extends Command {
  private final QuitaAlgas quitaAlgas;
  private Supplier<Boolean> BlqFree;
  private final Supplier<Double> Free;
  private final Supplier<Integer> pov;

  private static final double DETENER = -0.0; // -0.05;
  private boolean ResetPresionado_old = false;

  private static final int HOME = 0;
  private static final int PosicionSegura = 1;
  private static final int PosicionQuitar = 2;

  @Override
  public void initialize() {}

  public QuitaAlgasCommand(
      QuitaAlgas quitaAlgas,
      Supplier<Double> Free,
      Supplier<Boolean> BlqFree,
      Supplier<Integer> pov) {
    this.quitaAlgas = quitaAlgas;
    this.Free = Free;
    this.BlqFree = BlqFree;
    this.pov = pov;
    addRequirements(quitaAlgas);
  }

  @Override
  public void execute() {
    int povValue = pov.get();

    if (povValue == 180) { // abajo
      quitaAlgas.CambiarEstadoQuitaAlgas(PosicionSegura);
    } else if (povValue == 0) { // arriba
      quitaAlgas.CambiarEstadoQuitaAlgas(PosicionQuitar);
    } else if (povValue == 270) { // izquierda
      quitaAlgas.CambiarEstadoQuitaAlgas(HOME);
    } else if (BlqFree.get()) {
      quitaAlgas.setQuitaAlgas(-Free.get());
    } else {
      quitaAlgas.setQuitaAlgas(DETENER);
    }

    boolean ResetPresionado = (povValue == 90);
    if (ResetPresionado && !ResetPresionado_old) {
      quitaAlgas.ResetEncoder();
    }
    ResetPresionado_old = ResetPresionado;
  }

  @Override
  public void end(boolean interrupted) {
    quitaAlgas.setQuitaAlgas(DETENER);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
