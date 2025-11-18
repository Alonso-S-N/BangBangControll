// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubSystem.BangBangSub;
import frc.robot.SubSystem.Vision;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BangBangCommand extends Command {
  private final BangBangSub PetecoPeteco;
  private Boolean YPressed = false;
  private boolean indoPraCima = false;
  private boolean ParouOBracin = false;
  private boolean Xpressed = false;
  private Vision vision;
  
    private Joystick joyDelicioso = new Joystick(Constants.JoyDelicioso);
    
      public BangBangCommand(BangBangSub PetecoPeteco,Joystick joyDelicioso,Vision vision) {
       this.PetecoPeteco = PetecoPeteco;
       this.vision = vision;
       this.joyDelicioso = joyDelicioso;
     addRequirements(PetecoPeteco);
    }
  
  
    @Override
    public void initialize() {
    }
  
    public void mexerBracinSlk(){
    if (joyDelicioso.getRawAxis(Constants.RT) > 0.1){
      PetecoPeteco.MexePruLado();
    } else if (joyDelicioso.getRawAxis(Constants.LT) > 0.1){
      PetecoPeteco.MexePruOutro();
    } else {
      PetecoPeteco.StopBraceta();
    }
     //if (joyDelicioso.getRawButton(Constants.RB)){
       //braceta.MexePru(ANgulinQnoisQuer);
     // } else if (joyDelicioso.getRawButton(Constants.LB)){
       // braceta.MexePru(ANgulinQnoisQuer2);
      //}  
    }  
    public void mexerIntakeSlk(){
      if (joyDelicioso.getRawButton(Constants.RB)){
        PetecoPeteco.Pegar();
      } else if (joyDelicioso.getRawButton(Constants.LB)){
        PetecoPeteco.Cuspir();
      } else {
        PetecoPeteco.stopExtensor();
      }
    }

    public void PerseguirEAtirar(){
      if (vision.hasTarget()){
        if (vision.getTa() >= Constants.targetArea - vision.hysteresis){
          PetecoPeteco.autoShooter();
        } else {
          PetecoPeteco.StopBraceta();
        }
      }
    }
  
    public void MexerComPIDSLK(){
     // if (YPressed){
       // braceta.AcertaOCantoAi(braceta.posSubino);  
       // if (braceta.ArmController.atSetpoint()){
       // braceta.MoveTo(braceta.posIntakeExtend);
     // }        
     // } else if (Xpressed){ 
       // braceta.MoveTo(braceta.posIntakeRetract);
       // if (braceta.ExtensorController.atSetpoint()){
       // braceta.AcertaOCantoAi(braceta.posDesceno);
    }
 // }
//}

  public void resetBangBang(){
    if (joyDelicioso.getPOV() == 0){
    PetecoPeteco.ResetPetecoPeteco();
    }
  }

   public void ButtonYgetPressed(){ 
    if (joyDelicioso.getRawButtonPressed(Constants.y)){
       YPressed = true; 
        Xpressed = false;
        double rpm = 3000;
        PetecoPeteco.BangBangBalls(rpm);
        PetecoPeteco.tryToLauch(rpm);
       } 
     }

    public void buttonXgetPressed(){
      if (joyDelicioso.getRawButtonPressed(Constants.x)){
        Xpressed = true;
        YPressed = false;
        PetecoPeteco.autoShooter();
        }
    }


  public void StopBraceta(){
    if (joyDelicioso.getRawButtonPressed(Constants.b)){
      ParouOBracin = !ParouOBracin;
   PetecoPeteco.StopBraceta();
    YPressed = false; // Cancela o modo PID
    Xpressed = false; // Cancela o modo PID
    }
  }

  
  @Override
  public void execute() {
    joyDelicioso.getRawButton(Constants.RB);
    joyDelicioso.getRawButton(Constants.LB);
    joyDelicioso.getRawAxis(Constants.RT);
    joyDelicioso.getRawAxis(Constants.LT);
    joyDelicioso.getRawButton(Constants.y);
    joyDelicioso.getRawButton(Constants.b);
    joyDelicioso.getRawButton(Constants.x);


   ButtonYgetPressed();
   buttonXgetPressed();
   StopBraceta();
   
   if (Xpressed || YPressed){
    MexerComPIDSLK();
   } else {
   mexerBracinSlk();
   mexerIntakeSlk();
   }
   PerseguirEAtirar();

   SmartDashboard.putNumber("velocidade Bang",PetecoPeteco.JogaOsCoisoMotor.getAppliedOutput());
   SmartDashboard.putNumber("velocidade Coleta",PetecoPeteco.PegaOsCoisoMotor.getAppliedOutput());
   SmartDashboard.putBoolean("Button Y", joyDelicioso.getRawButton(Constants.y));
   SmartDashboard.putBoolean("Y Pressed", YPressed);
  }


  @Override
  public void end(boolean interrupted) {
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
