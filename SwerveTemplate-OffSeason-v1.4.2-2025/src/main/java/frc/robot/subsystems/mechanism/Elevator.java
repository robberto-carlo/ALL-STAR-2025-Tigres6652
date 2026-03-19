package frc.robot.subsystems.mechanism;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  WPI_TalonSRX talonElevatorLeft = new WPI_TalonSRX(13);
  WPI_VictorSPX talonElevatorRight = new WPI_VictorSPX(14);

  private PIDController pidElevador;
  private double KP_ELEVADOR = 1.6;
  private double KI_ELEVADOR = 0.0;
  private double KD_ELEVADOR = 0.0;

  private static final double PULSOS_VUELTA = 4096;
  private static final double REDUCCION = 4.5714;
  private static final double DIAMETRO = 1.4;

  private static final double MAX_ALTURA = 3.62; // es 3.64 el maximo;
  private static final double DETENER = 0.05; // -0.05
  private static final double ERROR = 0.1; // 0.2
  private static final double MAX_SALIDA = 1.0; // velocidad maxima del elevador: 1.0 = 100%

  double salida;
  double alturaObjetivo;
  double alturaActual;

  public Elevator() {
    configurarMotores();
    resetEncoder();
    inicializarPIDDashboard();
  }

  public void configurarMotores() {
    talonElevatorLeft.configFactoryDefault();
    talonElevatorRight.configFactoryDefault();

    pidElevador = new PIDController(KP_ELEVADOR, KI_ELEVADOR, KD_ELEVADOR);
    pidElevador.setTolerance(ERROR);

    talonElevatorLeft.configVoltageCompSaturation(12.0 * 0.8); // Voltaje maximo que permitimos
    talonElevatorLeft.enableVoltageCompensation(true);
    talonElevatorLeft.configContinuousCurrentLimit(20, 30); // límite de corriente continua
    talonElevatorLeft.configPeakCurrentLimit(30, 30); // limitar picos de corriente cortos
    talonElevatorLeft.configPeakCurrentDuration(100, 30); // 100 ms máximo

    talonElevatorLeft.configNominalOutputForward(0, 30);
    talonElevatorLeft.configNominalOutputReverse(0, 30);
    talonElevatorLeft.configPeakOutputForward(1, 30);
    talonElevatorLeft.configPeakOutputReverse(-1, 30);

    talonElevatorLeft.setSensorPhase(false); // true - sentido del encoder
    talonElevatorLeft.setInverted(true); // false -dirección del motor

    talonElevatorRight.follow(talonElevatorLeft);
    talonElevatorRight.setInverted(true); // false - dirección del motor
  }

  public void resetEncoder() {
    talonElevatorLeft.setSelectedSensorPosition(0);
  }

  public void inicializarPIDDashboard() {
    SmartDashboard.putNumber("KP Elevador", KP_ELEVADOR);
    SmartDashboard.putNumber("KI Elevador", KI_ELEVADOR);
    SmartDashboard.putNumber("KD Elevador", KD_ELEVADOR);
  }

  public double lecturaAltura() {
    double LecturaPulsos = talonElevatorLeft.getSelectedSensorPosition();
    double LecturaEncoder = LecturaPulsos / PULSOS_VUELTA;
    double LecturaReduccion = LecturaEncoder / REDUCCION;
    double Altura = Math.PI * DIAMETRO * LecturaReduccion;
    return Altura;
  }

  public double getAlturaObjetivo(int nivel) {
    switch (nivel) {
      case 0:
        return 0.08;
      case 1:
        return 1.12;
      case 2:
        return 3.62;
      default:
        return -1;
    }
  }

  public void movimientoElevadorNiveles(int nivel) {
    alturaObjetivo = getAlturaObjetivo(nivel);
    if (alturaObjetivo == -1) return;
    alturaActual = lecturaAltura();

    // pidElevador.reset(alturaActual);  // VER SI USARLO
    salida = pidElevador.calculate(alturaActual, alturaObjetivo);
    salida = Math.max(Math.min(salida, MAX_SALIDA), -MAX_SALIDA);

    if (pidElevador.atSetpoint()) {
      talonElevatorLeft.set(DETENER);
      simulacion(0);
    } else {
      talonElevatorLeft.set(ControlMode.PercentOutput, salida);
      simulacion(salida);
    }
  }

  public void movimientoLibre(double lecturaControl) {
    alturaActual = lecturaAltura();
    if (alturaActual >= MAX_ALTURA && lecturaControl > 0) {
      talonElevatorLeft.set(DETENER);
      simulacion(0);
    } else {
      talonElevatorLeft.set(ControlMode.PercentOutput, lecturaControl);
      simulacion(lecturaControl);
    }
  }

  @Override
  public void periodic() {
    actualizarPIDDashboard();
    SmartDashboard.putNumber("Altura Elevador", alturaActual);
    SmartDashboard.putNumber("Altura Objetivo", alturaObjetivo);
    SmartDashboard.putNumber("Salida PID", salida);
    SmartDashboard.putNumber("Error PID", pidElevador.getPositionError());

    SmartDashboard.putBoolean("Setpoint Elevador", pidElevador.atSetpoint());
  }

  public void actualizarPIDDashboard() {
    KP_ELEVADOR = SmartDashboard.getNumber("KP Elevador", KP_ELEVADOR);
    KI_ELEVADOR = SmartDashboard.getNumber("KI Elevador", KI_ELEVADOR);
    KD_ELEVADOR = SmartDashboard.getNumber("KD Elevador", KD_ELEVADOR);
    pidElevador.setPID(KP_ELEVADOR, KI_ELEVADOR, KD_ELEVADOR);
  }

  public void simulacion(double salida) {
    if (RobotBase.isSimulation()) {
      var sim = talonElevatorLeft.getSimCollection();
      int delta = (int) (salida * 100);
      sim.addQuadraturePosition(delta);
    }
  }
}
