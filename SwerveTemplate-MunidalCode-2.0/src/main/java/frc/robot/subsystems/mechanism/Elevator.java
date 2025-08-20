package frc.robot.subsystems.mechanism;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  WPI_TalonSRX TalonELevatorRigt = new WPI_TalonSRX(13);
  WPI_TalonSRX TalonELevatorLeft = new WPI_TalonSRX(14);

  DigitalInput LimitSwich = new DigitalInput(1);

  SparkMaxConfig leaderConfig = new SparkMaxConfig();
  SparkMaxConfig followerConfig = new SparkMaxConfig();
  SparkMaxConfig resetConfig = new SparkMaxConfig();

  PIDController PIDConfig = new PIDController(0.3, 0, 0.0);

  public void FreeMot(double speed) {
    TalonELevatorRigt.set(speed);
    TalonELevatorLeft.set(speed);
  }

  public double getElevatorHeight() {
    return 0;
  }

  public void PstDist(double distancia) {}

  public boolean LimitHome() {
    return LimitSwich.get();
  }

  public void MotorConfig() {}

  public void ResetMode() {}

  public Elevator() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distancia", getElevatorHeight());
    SmartDashboard.putBoolean("limit", LimitHome());
    SmartDashboard.putBoolean("Limitdirect", LimitSwich.get());
  }
}
