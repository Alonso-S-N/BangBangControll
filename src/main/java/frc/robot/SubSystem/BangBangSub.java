// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SubSystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calcs;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class BangBangSub extends SubsystemBase {

//Encoders de simulação:
  public final SparkMax JogaOsCoisoMotor = new SparkMax(Constants.m_peteco_peteco, MotorType.kBrushless);
  public final SparkMax PegaOsCoisoMotor = new SparkMax(Constants.m_coletor, MotorType.kBrushless);
  public final Encoder JogaOsCoisoEncoder = new Encoder(0, 1);
  public final EncoderSim JogaOsCoisoEncoderSim = new EncoderSim(JogaOsCoisoEncoder);
  BangBangController controller = new BangBangController();
  private Drive drive;
  private Vision vision;
  private ObjectSim objectSim;
  private Pose3d robotPose3d = new Pose3d();
  private double shooterRaio = 0.045; // raio em metros.
  private double shooterAng = Math.toRadians(20);
 
  

  public BangBangSub(Vision vision, Drive drive, ObjectSim objectSim) {
    this.vision = vision;
    this.drive = drive;
    this.objectSim = objectSim;
    JogaOsCoisoMotor.setInverted(false);
    PegaOsCoisoMotor.setInverted(false);
    final  SparkMaxConfig kCoast = new SparkMaxConfig();
    kCoast.idleMode(SparkBaseConfig.IdleMode.kCoast);
    kCoast.openLoopRampRate(0.2);
    controller.setTolerance(50);
    
    JogaOsCoisoMotor.configure(kCoast,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    PegaOsCoisoMotor.configure(kCoast,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  public void MexePruLado() {
   JogaOsCoisoMotor.set(0.10);
     if (RobotBase.isSimulation()){
     JogaOsCoisoMotor.set(1);
     }
  }

  public void ResetPetecoPeteco(){
   controller.setSetpoint(0);
   StopBraceta();
  }

  public void BangBangBalls(double setpoint){
    controller.setSetpoint(setpoint);
    double RPM = JogaOsCoisoMotor.getEncoder().getVelocity();
    double output = controller.calculate(RPM);
    JogaOsCoisoMotor.set(output);
  } 
  public void autoShooter() {
    if (vision.hasTarget()) {
        double distance = getDistanceToTag();
        double setpointRPM = getRPMFromDistance(distance);

        double currentRPM = JogaOsCoisoMotor.getEncoder().getVelocity();
        double output = controller.calculate(currentRPM, setpointRPM);

        JogaOsCoisoMotor.set(output);
        tryToLauch(setpointRPM);
        
        

        SmartDashboard.putNumber("Shooter Distance (m)", distance);
        SmartDashboard.putNumber("Setpoint RPM", setpointRPM);
        SmartDashboard.putNumber("Current RPM", currentRPM);
        SmartDashboard.putNumber("BangBang Output", output);

        if (RobotBase.isSimulation()){
          Logger.recordOutput("BangBang/SetpointRPM", setpointRPM);
          Logger.recordOutput("BangBang/CurrentRPM", currentRPM);
          Logger.recordOutput("BangBang/Output", output);
          Logger.recordOutput("BangBang/DistanceMeters", distance);
         
        }
    } else {
        StopBraceta(); // sem alvo
    }
     
}

public Translation3d calculateLaunchVelocity(double rpm) {
    double vt = (rpm * Math.PI * shooterRaio) / 60.0;

    double vx = vt * Math.cos(shooterAng);
    double vz = vt * Math.sin(shooterAng);

    return new Translation3d(vx, 0.0, vz);
}

 public void tryToLauch(double TargetRpm){
  if (!objectSim.isHeld()) {
    return; 
}
  double currentRPM = JogaOsCoisoMotor.getEncoder().getVelocity();
  //if (Math.abs(currentRPM - TargetRpm) < 50.0) {
    
    Pose3d robotPose = drive.getPose3d();
    Translation3d robotRelativeVelocity = calculateLaunchVelocity(TargetRpm);

    Pose3d launchPose = objectSim.getObjectPose();

    objectSim.launch(
            launchPose, 
            robotRelativeVelocity,
            robotPose.getRotation()
        );
  }
 //}
private double getRPMFromDistance(double distanceMeters) {
  // 1 metro → 2500 RPM, 4 metros → 4000 RPM
  return 2500 + (distanceMeters) * 500;
}  

  public void setRobotPose3d(Pose3d pose) {
    this.robotPose3d = pose;
  }
  

  public void MexePruOutro(){
   JogaOsCoisoMotor.set(-0.10);
    if (RobotBase.isSimulation()){
     JogaOsCoisoMotor.set(-1);
    }
  }

  public void StopBraceta(){
   JogaOsCoisoMotor.set(0.0);
   PegaOsCoisoMotor.set(0.0);
  }
  
  public void Cuspir(){
   PegaOsCoisoMotor.set(0.7);
  }
  public void Pegar(){
   PegaOsCoisoMotor.set(-0.7);
  }
  public void stopExtensor(){
   PegaOsCoisoMotor.set(0.0);
  }

  public double getDistanceToTag() {
    double cameraHeight = 0.6;
    double targetHeight = 1.5;   
    double cameraAngle = 25.0;    
    double ty = vision.getTy();         

 
    return (targetHeight - cameraHeight) /
           Math.tan(Math.toRadians(cameraAngle + ty));
}



  @Override
  public void simulationPeriodic() {
    double motorOutput = JogaOsCoisoMotor.get();
    double simulatedRPM = motorOutput * 5000; 
   JogaOsCoisoEncoderSim.setRate(simulatedRPM / 60.0); 
  }

  @Override
  public void periodic() {
  
  }
}
