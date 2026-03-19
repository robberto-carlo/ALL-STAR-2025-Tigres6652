package frc.robot.subsystems.mechanism;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outake extends SubsystemBase {
  SparkMax OutakeMotor = new SparkMax(15, MotorType.kBrushless);

  public void OutakeController(double OutakeSpeed) {
    OutakeMotor.set(-OutakeSpeed);
  }

  public void OutakeControlAdelante(double OutakeSpeedi) {
    OutakeMotor.set(-OutakeSpeedi); // disparar adelante
  }

  public void OutakeControlAtras(double OutakeSpeed) {
    OutakeMotor.set(OutakeSpeed); // disparar atras
  }

  public Outake() {}

  @Override
  public void periodic() {}
}
