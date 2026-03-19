package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanism.Elevator;
import java.util.function.Supplier;

public class ElevatorCommand extends Command {
  private final Elevator elevator;
  private Supplier<Boolean> Home, Pstn1, Pstn2, BlqFree, setZero;
  private Supplier<Double> Free;

  public static final int HOME = 0;
  public static final int NIVEL1 = 1;
  public static final int NIVEL2 = 2;

  private boolean resetPresionado_old = false;
  private static final double DETENER = -0.0; // -0.05

  public ElevatorCommand(
      Elevator elevator,
      Supplier<Boolean> Home,
      Supplier<Boolean> Pstn1,
      Supplier<Boolean> Pstn2,
      Supplier<Boolean> BlqFree,
      Supplier<Boolean> setZero,
      Supplier<Double> Free) {

    this.elevator = elevator;
    this.Home = Home;
    this.Pstn1 = Pstn1;
    this.Pstn2 = Pstn2;
    this.BlqFree = BlqFree;
    this.setZero = setZero;
    this.Free = Free;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if (Home.get()) {
      elevator.movimientoElevadorNiveles(HOME);

    } else if (Pstn1.get()) {
      elevator.movimientoElevadorNiveles(NIVEL1);

    } else if (Pstn2.get()) {
      elevator.movimientoElevadorNiveles(NIVEL2);

    } else if (BlqFree.get()) {
      elevator.movimientoLibre(-Free.get());

    } else {
      elevator.movimientoLibre(DETENER);
    }

    boolean ResetPresionado = setZero.get();
    if (ResetPresionado && !resetPresionado_old) {
      elevator.resetEncoder();
    }
    resetPresionado_old = ResetPresionado;
  }

  @Override
  public void end(boolean interrupted) {
    elevator.movimientoLibre(DETENER);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
