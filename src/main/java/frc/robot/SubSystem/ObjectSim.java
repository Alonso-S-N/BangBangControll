// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SubSystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;

public class ObjectSim extends SubsystemBase {
  private Pose3d objectPose;
  private final double gravity;
  private boolean isStatic;
  private Translation3d velocity;
  private boolean isHeld = true;
  private Pose3d robotLaunchOffset;

  private static double timeStep = 0.02; // 20 ms time step
   
  private String logkey;


  public ObjectSim(String logkey) {
    this.objectPose = new Pose3d();
    this.gravity = -9.81; // m/s^2
    this.isStatic = false;
    this.velocity = new Translation3d(0, 0, 0);
    this.logkey = logkey;
    this.isHeld = true;
    this.robotLaunchOffset = new Pose3d(
        new Translation3d(0.0, 0, 0.3), 
        new Rotation3d(0, 0, 0) 
    );
    Logger.recordOutput(this.logkey, objectPose);
  }

  public void launch(Pose3d startPose, Translation3d robotRelativeVelocity, Rotation3d robotRotation) {
 if (!isHeld) {
      return; // Só lança se estiver sendo segurado
    }
    Translation3d fieldRelativeVelocity = robotRelativeVelocity.rotateBy(robotRotation);
    
    this.objectPose = new Pose3d(
      startPose.getTranslation(),
      startPose.getRotation()
  );
  
    this.velocity = fieldRelativeVelocity; 
    this.isStatic = false;
    isHeld = false;
}
  public void update() {
    if (isStatic) {
      return;
    }
    double deltaTime = timeStep;

    double newZ = objectPose.getZ() + velocity.getZ() * deltaTime + 0.5 * gravity * deltaTime * deltaTime;
    double newX = objectPose.getX() + velocity.getX() * deltaTime;
    double newY = objectPose.getY() + velocity.getY() * deltaTime;

    velocity = new Translation3d(
        velocity.getX(),
        velocity.getY(),
        velocity.getZ() + gravity * deltaTime
    );

    if (newZ <= 0) {
      newZ = 0;
      velocity = new Translation3d(0, 0, 0);
      isStatic = true;
    }

    objectPose = new Pose3d(
        new Translation3d(newX, newY, newZ),
        objectPose.getRotation()
    );

    Logger.recordOutput(this.logkey, objectPose);
  }

  public Pose3d setRobotPose(Pose3d robotPose) {
    if (isHeld) {
        // Usa o método plus() para posicionar o objeto
        this.objectPose = robotPose.plus(
            new Transform3d(robotLaunchOffset.getTranslation(), robotLaunchOffset.getRotation())
        );
    }
        return objectPose;
}
  public boolean isStatic() {
    return isStatic;
  }

  public Pose3d getObjectPose() {
    return objectPose; // Retorna a pose atual
}
public boolean isHeld() {
    return isHeld;
}

  @Override
  public void periodic() {
 
  }

  @Override
  public void simulationPeriodic() {
    if (isHeld){
      Logger.recordOutput(this.logkey, objectPose);
      return;
    }
    update();
  
  }
}
