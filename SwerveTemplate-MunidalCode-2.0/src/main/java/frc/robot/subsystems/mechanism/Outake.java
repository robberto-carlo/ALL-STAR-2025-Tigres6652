package frc.robot.subsystems.mechanism;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outake extends SubsystemBase {
  WPI_VictorSPX OutakeMotor = new WPI_VictorSPX(16);

  public void OutakeController(double OutakeSpeed) {
    OutakeMotor.set(-OutakeSpeed);
  }

  public void OutakeControl(double OutakeSpeedi) {
    OutakeMotor.set(-OutakeSpeedi);
  }

  public Outake() {}

  @Override
  public void periodic() {}
}
