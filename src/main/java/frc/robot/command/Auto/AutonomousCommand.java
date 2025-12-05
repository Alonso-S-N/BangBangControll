package frc.robot.command.Auto;
import com.google.flatbuffers.ShortVector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SubSystem.Drive;
import frc.robot.SubSystem.Vision;
import frc.robot.SubSystem.BangBangSub;


public class AutonomousCommand extends Command {
  public enum State { Auto1,Auto2,Auto3,Auto4}
  private State state;

  private final Drive drive;
  private final Vision vision;
  private final double targetArea;
  private final BangBangSub shooter;
  private boolean finished;
  public double RightSpeed;

  public double LeftSpeed;
  private Timer timer = new Timer();

  private final double hysteresis = 0.5; 
  private boolean holdingPosition = false;

  public AutonomousCommand(Drive drive, Vision vision, double targetArea,BangBangSub shooter,State selectedState) {
      this.drive = drive;
      this.vision = vision;
      this.shooter = shooter;
      this.targetArea = targetArea;
        this.state = selectedState;
      addRequirements(drive,vision,shooter);
  }

  @Override
  public void initialize() {
      drive.reqDrive();
      drive.getPose();
      timer.reset();
      timer.start();
      finished = false;
      holdingPosition = false;
    
  }

  @Override
  public void execute() {
        Smart();
        switch (state) {
            case Auto1:
             vision.Perseguir();
                break;

            case Auto2:
            vision.perseguirAlgae();
                break;

            case Auto3:
            vision.perseguirCoral();
                break;
            case Auto4:
            vision.perseguirCargo();
                break;

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
