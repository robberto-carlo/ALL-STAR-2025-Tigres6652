package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanism.Elevator;
import java.util.function.Supplier;

public class ElevatorCommand extends Command {
  private final Elevator elevator;
  private Supplier<Boolean> Home, Pstn1, Pstn2, BlqFree;
  private Supplier<Double> Free;

  public ElevatorCommand(
      Elevator elevator,
      Supplier<Boolean> Home,
      Supplier<Boolean> Pstn1,
      Supplier<Boolean> Pstn2,
      Supplier<Boolean> BlqFree,
      Supplier<Double> Free) {
    this.Home = Home;
    this.Pstn1 = Pstn1;
    this.Pstn2 = Pstn2;
    this.BlqFree = BlqFree;
    this.Free = Free;
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.MotorConfig();
    elevator.ResetMode();
  }

  @Override
  public void execute() {
    if (BlqFree.get() == true) {
      elevator.FreeMot(Free.get() * .5);

    } else if (Home.get() == true) {
      elevator.PstDist(0);

    } else if (Pstn1.get() == true) {
      elevator.PstDist(-40);

    } else if (Pstn2.get() == true) {
      elevator.PstDist(-75);

    } else {
      elevator.PstDist(elevator.getElevatorHeight());
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
