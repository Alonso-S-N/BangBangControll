// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SubSystem;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class TrajectoryFollower extends SubsystemBase {
  Drive drive;
  private final Trajectory m_trajectory;
  
  public TrajectoryFollower(Drive drive) {
    this.drive = drive;

   
    
    TrajectoryConfig config = new TrajectoryConfig(1.5, 1.0);

    var PontoDePartida = drive.getPose();

    var WayPoint1 = new Pose2d(1.0 ,1.0, Rotation2d.fromDegrees(90));

    var WayPoint2 = new Pose2d(2.0 ,0, Rotation2d.fromDegrees(-90));

    var PontoFinal = new Pose2d(3.0,0.0,Rotation2d.fromDegrees(0));
    
   
    m_trajectory = TrajectoryGenerator.generateTrajectory(
      // Faz Uma curva em S
      List.of(
        PontoDePartida,
        WayPoint1,
        WayPoint2,
        PontoFinal
      ),
      config);

    Logger.recordOutput("trajectory",m_trajectory);
  }

 public Command followTrajectoryCommand() {
  RamseteCommand ramseteCommand = new RamseteCommand(
    m_trajectory,
    drive::getPose,
    new RamseteController(2.0, 0.7),
    drive.m_kinematics,
    drive::tankDriveVolts,
    drive
    );
// 2. Retorna a sequência de comandos: Resetar -> Seguir Trajetória -> Parar
return Commands.sequence(
    // Reseta o odômetro para o ponto inicial da trajetória
    Commands.runOnce(() -> drive.resetOdometry(m_trajectory.getInitialPose()), drive),
    
    // Segue a trajetória
    ramseteCommand,
    
    // Para os motores (0 Volts) no fim
    Commands.runOnce(() -> drive.tankDriveVolts(0, 0), drive)
);
}
   


  @Override
  public void periodic() {
  }
}
