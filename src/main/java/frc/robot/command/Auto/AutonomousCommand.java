package frc.robot.command.Auto;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SubSystem.Drive;
import frc.robot.SubSystem.Vision;


public class AutonomousCommand extends Command {
  private enum State { procurando,seguindo,segurando }
  private State state;

  private final Drive drive;
  private final Vision vision;
  private final double targetArea;
  private boolean finished;
  public double RightSpeed;

  public double LeftSpeed;
  private Timer timer = new Timer();

  private final double hysteresis = 0.5; 
  private boolean holdingPosition = false;

  public AutonomousCommand(Drive drive, Vision vision, double targetArea) {
      this.drive = drive;
      this.vision = vision;
      this.targetArea = targetArea;
      addRequirements(drive,vision);
  }

  @Override
  public void initialize() {
      drive.reqDrive();
      drive.getPose();
      timer.reset();
      timer.start();
      finished = false;
      holdingPosition = false;
      state = State.procurando;
  }

  @Override
  public void execute() {
        Smart();
      switch (state) {
          case procurando:
              runSearch();
            System.out.println("Procurando");
              break;

          case seguindo:
              runTrack();
             System.out.println("seguindo"); 
              break;

          case segurando:
              setDriveSpeeds(0, 0);
             System.out.println("segurando"); 
              break;
      }

      if (timer.get() >= Constants.autonomousTime) {
          finished = true;
      }

      if (drive.driveSim != null) {
          drive.simulationPeriodic();
      }
  }

  private void runSearch() {
      double t = timer.get();

      if (vision.hasTarget()) {
          state = State.seguindo;
          return;
      }

      if (t < 2.0) {
          setDriveSpeeds(-0.2, 0.2); 
      } else if (t < 4.0) {
          setDriveSpeeds(0.2, -0.2); 
      } else {
          setDriveSpeeds(0, 0); 
          state = State.segurando;
      }
  }

  private void runTrack() {
      if (!vision.hasTarget()) {
         
          state = State.procurando;
          return;
      }

      double tx = vision.getTx();
      double ta = vision.getTa();
      
      double kP_turn = 0.03;
      double kP_forward = 0.1;

      double turn = tx * kP_turn; 
      double forward = (targetArea - ta) * kP_forward;

      forward = Math.max(-0.5, Math.min(0.5, forward));

      double left = forward + turn;
      double right = forward - turn;

      left = Math.max(-0.5, Math.min(0.5, left));
      right = Math.max(-0.5, Math.min(0.5, right));

      RightSpeed = right;
      LeftSpeed = left;

      if (ta >= (targetArea - hysteresis)) {
          state = State.segurando; 
          setDriveSpeeds(0, 0);
      } else {
          setDriveSpeeds(left, right);
      }
  }

  @Override
  public boolean isFinished() {
      return finished;
  }

  @Override
  public void end(boolean interrupted) {
      setDriveSpeeds(0, 0);
  }

  private void setDriveSpeeds(double left, double right) {
    drive.rawTank(left, right);
  }

  public void Smart(){
    SmartDashboard.putString("Auto State", state.toString());
    SmartDashboard.putNumber("TX", vision.getTx());
    SmartDashboard.putNumber("TA", vision.getTa());
    SmartDashboard.putBoolean("finished:", finished);
    SmartDashboard.putBoolean("Targeted", vision.hasTarget());
  }
}
