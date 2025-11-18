package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.SubSystem.Drive;
import frc.robot.SubSystem.ObjectSim;
import frc.robot.SubSystem.Vision;
import frc.robot.SubSystem.BangBangSub;
import frc.robot.command.Auto.AutonomousCommand;
import frc.robot.command.Drive.Loc;
import frc.robot.command.Drive.BangBangCommand;

public class RobotContainer {

  // Subsystems
  private final ObjectSim object = new ObjectSim("ObjectSim");
  private final Drive driveSubsystem = new Drive(object);
  private final Vision vision = new Vision(driveSubsystem,Constants.targetArea);
  public final BangBangSub baby = new BangBangSub(vision,driveSubsystem,object);
  


  // Input
  public final Joystick joyDeliciu = new Joystick(Constants.joy);
  public final Joystick JoyDelicioso = new Joystick(Constants.JoyDelicioso);

  // Commands
  private final Loc locCommand;
  
  private final AutonomousCommand auto;
  private final BangBangCommand Pdiddy;
  


  public RobotContainer() { 

    Pdiddy = new BangBangCommand(baby,JoyDelicioso,vision);

    CommandScheduler.getInstance().registerSubsystem(driveSubsystem);

    // Initialize Loc command with drive subsystem and joystick
    auto = new AutonomousCommand(driveSubsystem,vision,Constants.targetArea);

    locCommand = new Loc(driveSubsystem,joyDeliciu,baby,vision,JoyDelicioso);

    // Set default command
    driveSubsystem.setDefaultCommand(locCommand);

    baby.setDefaultCommand(Pdiddy);

  }

  public Command getAutonomousCommand(){
      return auto;
  
  }

  public Command getBracinCommand(){
    return Pdiddy;
  }

}
