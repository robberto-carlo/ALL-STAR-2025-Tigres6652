// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanism;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class QuitaAlgas extends SubsystemBase {
  private final WPI_TalonSRX QuitaAlgas = new WPI_TalonSRX(16);

  private PIDController pidQuitaAlgas;
  private double KP_QUITA_ALGAS = 1.0;
  private double KI_QUITA_ALGAS = 0.0;
  private double KD_QUITA_ALGAS = 0.0;

  private static final double PULSOS_VUELTA = 4096;
  private static final double REDUCCION = 4.5714;
  private static final double DIAMETRO = 1.4;
  private static final double DETENER = -0.0; // 0.0
  private static final double ERROR = 0.05;
  private static final double MAX_ALTURA = 0.62; // 0.62 es el maximo;

  double PosicionObjetivo;
  double posicionActual;
  double salida;

  public QuitaAlgas() {
    ConfiguracionMotor();
    ResetEncoder();
    SmartDashboard.putNumber("KP QUITA_ALGAS", KP_QUITA_ALGAS);
    SmartDashboard.putNumber("KI QUITA_ALGAS", KI_QUITA_ALGAS);
    SmartDashboard.putNumber("KD QUITA_ALGAS", KD_QUITA_ALGAS);
  }

  public void ConfiguracionMotor() {
    pidQuitaAlgas = new PIDController(KP_QUITA_ALGAS, KI_QUITA_ALGAS, KD_QUITA_ALGAS);
    pidQuitaAlgas.setTolerance(ERROR);
    
    QuitaAlgas.configFactoryDefault();

    QuitaAlgas.configReverseSoftLimitThreshold(0, 0);

    QuitaAlgas.configVoltageCompSaturation(12 * 0.7); // Voltaje maximo que permitimos

    QuitaAlgas.configNominalOutputForward(0, 30);
    QuitaAlgas.configNominalOutputReverse(0, 30);
    QuitaAlgas.configPeakOutputForward(0.5, 30); // 0.2
    QuitaAlgas.configPeakOutputReverse(-0.5, 30); // 0.2

    QuitaAlgas.config_kP(0, 1, 30);
    QuitaAlgas.config_kI(0, 0, 30);
    QuitaAlgas.config_kD(0, 0, 30);
    QuitaAlgas.config_kF(0, 0, 30);

    QuitaAlgas.setSensorPhase(true);
    QuitaAlgas.setInverted(true);
  }

  public double getPosicionObjetivo(int estado) {
    switch (estado) {
      case 0:
        return 0.0;
      case 1:
        return 0.3;
      case 2:
        return 0.66;
      default:
        return -1;
    }
  }

  public void CambiarEstadoQuitaAlgas(int estado) {
    PosicionObjetivo = getPosicionObjetivo(estado);
    if (PosicionObjetivo == -1) return;
    posicionActual = LecturaPosicion();

    salida = pidQuitaAlgas.calculate(posicionActual, PosicionObjetivo);
    salida = Math.max(Math.min(salida, 1.0), -1.0);

    if (pidQuitaAlgas.atSetpoint()) {
      QuitaAlgas.set(DETENER);
      Simulacion(0);
    } else {
      if (Math.abs(salida) < 0.1) {
        salida = Math.copySign(0.1, salida);
      }
      QuitaAlgas.set(ControlMode.PercentOutput, salida);
      Simulacion(salida);
    }
  }

  public void setQuitaAlgas(double lecturaControl) {
    posicionActual = LecturaPosicion();
    if (posicionActual >= MAX_ALTURA && lecturaControl > 0) {
      QuitaAlgas.set(DETENER);
      Simulacion(0);
    } else {
      QuitaAlgas.set(ControlMode.PercentOutput, lecturaControl);
      Simulacion(lecturaControl);
    }
  }

  public double LecturaPosicion() {
    double LecturaPulsos = QuitaAlgas.getSelectedSensorPosition();
    double LecturaEncoder = LecturaPulsos / PULSOS_VUELTA;
    double LecturaReduccion = LecturaEncoder / REDUCCION;
    double Posicion = Math.PI * DIAMETRO * LecturaReduccion;
    return Posicion;
  }

  public void ResetEncoder() {
    QuitaAlgas.setSelectedSensorPosition(0);
  }

  public void Simulacion(double salida) {
    if (RobotBase.isSimulation()) {
      var sim = QuitaAlgas.getSimCollection();
      int delta = (int) (salida * 100);
      sim.addQuadraturePosition(delta);
    }
  }

  @Override
  public void periodic() {
    KP_QUITA_ALGAS = SmartDashboard.getNumber("KP QUITA_ALGAS", KP_QUITA_ALGAS);
    KI_QUITA_ALGAS = SmartDashboard.getNumber("KI QUITA_ALGAS", KI_QUITA_ALGAS);
    KD_QUITA_ALGAS = SmartDashboard.getNumber("KD QUITA_ALGAS", KD_QUITA_ALGAS);
    pidQuitaAlgas.setPID(KP_QUITA_ALGAS, KI_QUITA_ALGAS, KD_QUITA_ALGAS);

    SmartDashboard.putNumber("Posicion Algas", LecturaPosicion());
    SmartDashboard.putNumber("Posicion Objetivo", PosicionObjetivo);
    SmartDashboard.putNumber("Salida PID Algas", salida);
    SmartDashboard.putNumber("Error PID Algas", pidQuitaAlgas.getPositionError());
    SmartDashboard.putBoolean("Setpoint Algas", pidQuitaAlgas.atSetpoint());
  }
}
