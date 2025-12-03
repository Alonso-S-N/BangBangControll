package frc.robot.command.Drive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.SubSystem.BangBangSub;
import frc.robot.SubSystem.Drive;
import frc.robot.SubSystem.ObjectSim;
import frc.robot.SubSystem.TrajectoryFollower;
import frc.robot.SubSystem.Vision;
import frc.robot.command.Auto.AutonomousCommand;
import frc.robot.Calcs;
import frc.robot.Calcs.DriveSpeeds;

public class Loc extends Command {
  
  private final Drive driveSubsystem;
  private final Joystick joyDeliciu;
  private final Joystick joyDelicioso;
  private final BangBangSub baby;
  private final Vision vision;
  DriveSpeeds speeds;
  private final TrajectoryFollower traj;
  
    private double B_Speed = 0;
    private boolean a, b, x, a2;
    private boolean joyDeliciosoA2Pressed;
    private double POV;

    public enum DriveMode {
      MANUAL,
      VISION,
      POV,
      TRAJECTORY,
      VISION_C,
      VISION_A,
      VISION_CA,
      STOPPED
  }  

  private DriveMode currentMode = DriveMode.STOPPED;
    
  
    public Loc(Drive driveSubsystem,Joystick joyDeliciu,BangBangSub baby,Vision vision,Joystick joyDelicioso,TrajectoryFollower traj) {
      this.driveSubsystem = driveSubsystem;
      this.vision = vision;
      this.traj = traj;
      this.joyDelicioso = joyDelicioso;
      this.joyDeliciu = joyDeliciu;
      this.baby = baby;
    addRequirements(driveSubsystem,vision);
  }

  @Override
  public void initialize() {
    driveSubsystem.reqDrive();
  }

  @Override
  public void execute() {
    Smart();
    button();
    MainControl();
    launch();
    SmartDashboard.putString("Loc Status", "Rodando");
  }

private void setDriveSpeeds(double left, double right) {
        driveSubsystem.rawTank(left, right);
}

private void setMode(DriveMode mode) {
  if (mode != currentMode) {
      System.out.println("Mudando DriveMode para: " + mode);
      currentMode = mode;
  }
}

  private void stopDrive() {
    setDriveSpeeds(0, 0);
  }

  public void launch(){
    POV = joyDelicioso.getPOV();
    if (POV == 0){
       
    }
  }

  public void button(){
    a = joyDeliciu.getRawButton(Constants.a);
    b = joyDeliciu.getRawButton(Constants.b);
    x = joyDeliciu.getRawButton(Constants.x);
    a2 = joyDelicioso.getRawButtonPressed(Constants.a);
    
    
           
    if (a2){
      joyDeliciosoA2Pressed = !joyDeliciosoA2Pressed;
      vision.resetTime();
    }

    if (a) B_Speed = 0.25;
    else if (b) B_Speed = 0.5;
    else if (x) B_Speed = 1.0;
  
  }

  public void MainControl() {

    double X  = joyDeliciu.getX();
    double Y  = joyDeliciu.getY();
    double X1 = joyDeliciu.getRawAxis(Constants.X1);
    double Y2 = joyDeliciu.getRawAxis(Constants.Y2);

    // ----- TRAJETÓRIA -----//
    if (joyDelicioso.getPOV() == 0) {
        setMode(DriveMode.TRAJECTORY);

        CommandScheduler.getInstance().schedule(
            traj.followTrajectoryCommand()
            .andThen(() -> currentMode = DriveMode.STOPPED)
        );

        speeds = null;  // Loc NÃO controla os motores
        return;
    }

    // ----- VISÃO -----//
    if (joyDeliciosoA2Pressed) {
        setMode(DriveMode.VISION);

        vision.Perseguir();
        speeds = new DriveSpeeds(vision.LeftSpeed, vision.RightSpeed);

        setDriveSpeeds(speeds.left, speeds.right);
        return;
    }
    // ----- VISÃO CORAL-----//
     if (joyDelicioso.getPOV() == 90) {
        setMode(DriveMode.VISION_C);

        vision.perseguirCoral();
        speeds = new DriveSpeeds(vision.LeftSpeed, vision.RightSpeed);

        setDriveSpeeds(speeds.left, speeds.right);
        return;
     }
      // ----- VISÃO ALGAE-----//
      if (joyDelicioso.getPOV() == 270) {
          setMode(DriveMode.VISION_A);
  
          vision.perseguirAlgae();
          speeds = new DriveSpeeds(vision.LeftSpeed, vision.RightSpeed);
  
          setDriveSpeeds(speeds.left, speeds.right);
          return;
      }

      // ----- VISÃO Cargo -----//
      if (joyDelicioso.getPOV() == 180) {
          setMode(DriveMode.VISION_CA);
  
          vision.perseguirCargo();
          speeds = new DriveSpeeds(vision.LeftSpeed, vision.RightSpeed);
  
          setDriveSpeeds(speeds.left, speeds.right);
          return;
      }
    // ----- POV DRIVE -----//
    if (joyDeliciu.getPOV() != Constants.povDeadZone) {
        setMode(DriveMode.POV);

        speeds = Calcs.calculatePovDrive(joyDeliciu, B_Speed);
        setDriveSpeeds(speeds.left, speeds.right);
        return;
    }

    // ----- ANALÓGICO NORMAL -----//
    if (Math.abs(X)  >= Constants.deadZone ||
        Math.abs(Y)  >= Constants.deadZone ||
        Math.abs(X1) >= Constants.deadZone ||
        Math.abs(Y2) >= Constants.deadZone) {

        setMode(DriveMode.MANUAL);

        speeds = Calcs.calculateAnalogDrive(joyDeliciu, B_Speed);
        setDriveSpeeds(speeds.left, speeds.right);
        return;
    }

    // ----- NINGUÉM CONTROLANDO ----- //
    setMode(DriveMode.STOPPED);
    stopDrive();
    speeds = null;
}



  public void Smart(){
    if (speeds != null) {
      SmartDashboard.putNumber("Left Speed", speeds.left);
      SmartDashboard.putNumber("Right Speed", speeds.right);
    } else {
      SmartDashboard.putNumber("Left Speed", 0);
      SmartDashboard.putNumber("Right Speed", 0);
    }
    SmartDashboard.putBoolean("Button A", a);
    SmartDashboard.putBoolean("Button B", b);
    SmartDashboard.putBoolean("Button X", x);
    SmartDashboard.putNumber("VelB", B_Speed);
  }



  @Override
  public void end(boolean interrupted) {
    stopDrive();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}