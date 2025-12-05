package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.SubSystem.Drive;
import frc.robot.SubSystem.ObjectSim;
import frc.robot.SubSystem.TrajectoryFollower;
import frc.robot.SubSystem.Vision;
import frc.robot.SubSystem.BangBangSub;
import frc.robot.command.Auto.AutonomousCommand;
import frc.robot.command.Auto.AutonomousCommand.State;
import frc.robot.command.Drive.Loc;
import frc.robot.command.Drive.BangBangCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Subsystems
  private final ObjectSim object = new ObjectSim("ObjectSim");
  private final Drive driveSubsystem = new Drive(object);
  private final Vision vision = new Vision(driveSubsystem,Constants.targetArea);
  public final BangBangSub baby = new BangBangSub(vision,driveSubsystem,object);
  private final TrajectoryFollower traj = new TrajectoryFollower(driveSubsystem);
  


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
    auto = new AutonomousCommand(driveSubsystem,vision,Constants.targetArea,baby,State.Auto1);

    locCommand = new Loc(driveSubsystem,joyDeliciu,baby,vision,JoyDelicioso,traj);

    // Set default command
    driveSubsystem.setDefaultCommand(locCommand);

    baby.setDefaultCommand(Pdiddy);

  }

    public void configAuto() {

      autoChooser.setDefaultOption(
          "Autônomo 1",
          new AutonomousCommand(driveSubsystem, vision, Constants.targetArea, baby, State.Auto1)
      );
      autoChooser.addOption(
          "Autônomo 2",
          new AutonomousCommand(driveSubsystem, vision, Constants.targetArea, baby, State.Auto2)
      );
      autoChooser.addOption(
          "Autônomo 3",
          new AutonomousCommand(driveSubsystem, vision, Constants.targetArea, baby, State.Auto3)
      );
      autoChooser.addOption(
          "Autônomo 4",
          new AutonomousCommand(driveSubsystem, vision, Constants.targetArea, baby, State.Auto4)
      );
  
      SmartDashboard.putData("Modo Autônomo", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
}

  public Command getBracinCommand(){
    return Pdiddy;
  }

}
