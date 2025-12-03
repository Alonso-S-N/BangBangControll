package frc.robot.command.Drive;

import frc.robot.SubSystem.BangBangSub;
import frc.robot.SubSystem.Vision;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class BangBangCommand extends Command {

  private final BangBangSub shooter;
  private final Vision vision;
  private final Joystick joy;

  private boolean yMode = false;
  private boolean xMode = false;
  private boolean bracetaTravado = false;

  public BangBangCommand(BangBangSub shooter, Joystick joy, Vision vision) {
    this.shooter = shooter;
    this.vision = vision;
    this.joy = joy;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  /* ===========================
     SEÇÃO DE LEITURA DE BOTÕES
     =========================== */

  private void handleModeButtons() {

    // Y → BangBang manual fixo em 3000 RPM
    if (joy.getRawButtonPressed(Constants.y)) {
      yMode = true;
      xMode = false;
    }

    // X → AutoShooter baseado no distance-to-tag
    if (joy.getRawButtonPressed(Constants.x)) {
      xMode = true;
      yMode = false;
    }

    // B → trava/destrava e força Stop
    if (joy.getRawButtonPressed(Constants.b)) {
      bracetaTravado = !bracetaTravado;
      shooter.StopShooter();
      shooter.ResetPetecoPeteco();
      yMode = false;
      xMode = false;
    }

  }

  /* ===========================
        CONTROLE DO INTAKE
     =========================== */
  private void controleIntake() {
    if (joy.getRawButton(Constants.RB)) {
      shooter.Pegar();
    } else if (joy.getRawButton(Constants.LB)) {
      shooter.Cuspir();
    } else {
      shooter.stopExtensor();
    }
  }

  /* ===========================
        CONTROLE DO TRANSPORT
     =========================== */
  private void controleDoTransporte() {
    double rt = joy.getRawAxis(Constants.RT);
    double lt = joy.getRawAxis(Constants.LT);

    if (rt > 0.5) {
      shooter.PuxarOsCoiso();
    } 
    else if (lt > 0.5) {
      shooter.LargarOsCoiso();
    } 
    else {
      shooter.RodaColetorMotor.set(0);
    }
  }

  /* ===========================
        CONTROLE DO SHOOTER
     =========================== */
  private void controleShooter() {

    // Se travou o braceta → nada funciona
    if (bracetaTravado) {
      shooter.StopShooter();
      return;
    }

    // MODO Y → BangBang fixo
    if (yMode) {
      double rpm = 1000;
      shooter.BangBangBalls(rpm);
      shooter.tryToLauch(rpm);
      return;
    }

    // MODO X → Auto shooter com visão
    if (xMode) {
      shooter.autoShooter();
      return;
    }

    // Se não está em nenhum modo, para
    shooter.StopShooter();
  }

/*  ========================== 
           EXECUTE           
  ========================== */
  @Override
  public void execute() {

    // 1) Lê botões e atualiza modos
    handleModeButtons();

    // 2) Controle do intake
    controleIntake();

    // 3) Controle do transporte
    controleDoTransporte();

    // 4) Controle completo do shooter
    controleShooter();

    // 5) Dashboard pra debug
    SmartDashboard.putNumber("BangUp Out", shooter.MotorUp.getAppliedOutput());
   // SmartDashboard.putNumber("BangDown Out", shooter.MotorDown.getAppliedOutput());
    SmartDashboard.putNumber("Coletor Out", shooter.PegaOsCoisoMotor.getAppliedOutput());
    SmartDashboard.putNumber("Transport Out", shooter.LevarPraTacarOsCoiso.getAppliedOutput());
    SmartDashboard.putBoolean("Y Mode", yMode);
    SmartDashboard.putBoolean("X Mode", xMode);
    SmartDashboard.putBoolean("Travado", bracetaTravado);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
