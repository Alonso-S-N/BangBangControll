package frc.robot.SubSystem;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calcs.DriveSpeeds;
import frc.robot.LimelightHelpers;

import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


public class Vision extends SubsystemBase {

        private double tx,ta,ty;
        private boolean tv;
        private final double targetArea;
  private boolean finished;
  private int [] validIds = {1,18};
  public double RightSpeed;
  public LimelightHelpers.PoseEstimate mt2;
  public double DistanceTolerance = 0.2;

  private Drive drive;
  
    public double LeftSpeed;
    private Timer timer = new Timer();
  
    public final double hysteresis = 0.5; 
    private boolean holdingPosition = false;
  
  
          
          private final PhotonCamera camera;
         private PhotonCameraSim cameraSim;
         private VisionSystemSim visionSim;
  
          private final NetworkTable limelight;           
  
        public Vision(Drive drive, double targetArea) {
          this.targetArea = targetArea;
          this.drive = drive;
                 timer.reset();
                drive.getPose();
                finished = false;
                holdingPosition = false;
                camera = new PhotonCamera("camera");

limelight = NetworkTableInstance.getDefault().getTable("limelight-front");
LimelightHelpers.SetFiducialIDFiltersOverride("limelight-front", validIds);

AprilTagFieldLayout tagLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();

if (RobotBase.isSimulation()) {

  SimCameraProperties props = new SimCameraProperties();
  props.setCalibration(1920, 1080, Rotation2d.fromDegrees(90)); 
  props.setFPS(30);
  props.setAvgLatencyMs(30);
  props.setLatencyStdDevMs(5);
      
                   
  visionSim = new VisionSystemSim("MainVisionSim");
  visionSim.addAprilTags(tagLayout);

  Transform3d robotToCamera = new Transform3d(
    new Translation3d(-0.2, 0.6 , 0.4),
    new Rotation3d(0, Math.toRadians(25), 0)
                );
    
                
                cameraSim = new PhotonCameraSim(camera, props);
                visionSim.addCamera(cameraSim, robotToCamera);
    }   
  }

         public double getAITx() {
         return limelight.getEntry("tx_ai").getDouble(0.0); 
        }
        public double getAITXCA() {
          return limelight.getEntry("tx_aiCa").getDouble(0.0); 
         }

        public double DistanceCa(){
          return limelight.getEntry("distancia_cargo" ).getDouble(0.0);
        }

        public double DistanceC(){
          return limelight.getEntry("distancia_coral" ).getDouble(0.0);
        }

        public double DistanceA(){
          return limelight.getEntry("distancia_algae" ).getDouble(0.0);
        }    
        public boolean hasTarget() {
         return limelight.getEntry("tv").getDouble(0.0) == 1.0;
        }
        public double getTx() {
         return limelight.getEntry("tx").getDouble(0.0);
        }
    
        public double getTa() {
         return limelight.getEntry("ta").getDouble(0.0);
        }
        
        public double getTy(){
          return limelight.getEntry("ty").getDouble(0.0);
        }
        @Override
        public void periodic() {
          LimelightHelpers.SetRobotOrientation("limelight-front",drive.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
          LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
           
          if (mt2 != null && mt2.tagCount > 0) {
            if (mt2.timestampSeconds > 0) {
              drive.addVisionMeasurement(
                  mt2.pose,
                  mt2.timestampSeconds
              );
          }
          System.out.println("Tipo de pose: " + mt2.pose.getClass());
            }
        

            SmartDashboard.putBoolean("Limelight Target", hasTarget());
            SmartDashboard.putNumber("Limelight TX", getTx());
            SmartDashboard.putNumber("Limelight TA", getTa());

         
      }  
      public DriveSpeeds perseguirCargo(){
        timer.start();
          if (DistanceCa() != 0){
            double currentDistance = DistanceCa();
            getAITXCA();
            double kP_turn = 0.03;
            double kP_forward = 0.3;
            
            double turn = getAITXCA() * kP_turn;
            double forward = (DistanceTolerance - DistanceCa()) * kP_forward;

            forward = Math.max(-0.5, Math.min(0.5, forward));
            turn = Math.max(-0.5, Math.min(0.5, turn));

            double left = forward + turn;
            double right = forward - turn;

            left = Math.max(-0.5, Math.min(0.5, left));
            right = Math.max(-0.5, Math.min(0.5, right));

            RightSpeed = right;
            LeftSpeed = left;

            setDriveSpeeds(left, right);

            if (DistanceCa() - hysteresis <= currentDistance) {
                setDriveSpeeds(0, 0);
            } else {
                setDriveSpeeds(left, right);
            } 
          }
         else if (DistanceC() == 0) {
          if (getTime() < 2.0) {
            setDriveSpeeds(-0.3, 0.3); 
            LeftSpeed = -0.3; RightSpeed = 0.3;
        } else if (getTime() < 4.0) {
            setDriveSpeeds(0.3, -0.3); 
            LeftSpeed = 0.3; RightSpeed = -0.3;
        } else {
            setDriveSpeeds(0, 0);
            LeftSpeed = 0; RightSpeed = 0;
        }
      }
        return new DriveSpeeds(LeftSpeed, RightSpeed);
      }

      public DriveSpeeds perseguirAlgae(){
        timer.start();
          if (DistanceA() != 0){
            double currentDistance = DistanceA();
            getAITx();
            double kP_turn = 0.03;
            double kP_forward = 0.3;
            
            double turn = getAITx() * kP_turn;
            double forward = (DistanceTolerance - DistanceA()) * kP_forward;

            forward = Math.max(-0.5, Math.min(0.5, forward));
            turn = Math.max(-0.5, Math.min(0.5, turn));

            double left = forward + turn;
            double right = forward - turn;

            left = Math.max(-0.5, Math.min(0.5, left));
            right = Math.max(-0.5, Math.min(0.5, right));

            RightSpeed = right;
            LeftSpeed = left;

            setDriveSpeeds(left, right);

            if (DistanceA() - hysteresis <= currentDistance) {
                left = 0; right = 0;
            } else {
              LeftSpeed = left; RightSpeed = right;
            } 
          } else if (DistanceA() == 0) {
          if (getTime() < 2.0) {
            setDriveSpeeds(-0.3, 0.3); 
            LeftSpeed = -0.3; RightSpeed = 0.3;
        } else if (getTime() < 4.0) {
            setDriveSpeeds(0.3, -0.3); 
            LeftSpeed = 0.3; RightSpeed = -0.3;
        } else {
            LeftSpeed = 0; RightSpeed = 0;
        }
      }
        return new DriveSpeeds(LeftSpeed, RightSpeed);
      }

      public DriveSpeeds perseguirCoral(){
        timer.start();
          if (DistanceC() != 0){
            double currentDistance = DistanceC();
            getAITx();
            double kP_turn = 0.03;
            double kP_forward = 0.3;
            
            double turn = getAITx() * kP_turn;
            double forward = (DistanceTolerance - DistanceC()) * kP_forward;

            forward = Math.max(-0.5, Math.min(0.5, forward));

            double left = forward + turn;
            double right = forward - turn;

            left = Math.max(-0.5, Math.min(0.5, left));
            right = Math.max(-0.5, Math.min(0.5, right));

            RightSpeed = right;
            LeftSpeed = left;


            if (DistanceC() - hysteresis <= currentDistance) {
            } else {
              LeftSpeed = left; RightSpeed = right;
            }
            } 
              else if (DistanceC() == 0) {
              if (getTime() < 2.0) {
                LeftSpeed = -0.3; RightSpeed = 0.3;
            } else if (getTime() < 4.0) {
                LeftSpeed = 0.3; RightSpeed = -0.3;
            } else {
                LeftSpeed = 0; RightSpeed = 0;
          }
        }
      
        return new DriveSpeeds(LeftSpeed, RightSpeed);
      }
  

   
        public DriveSpeeds Perseguir(){
          timer.start();
          if (hasTarget()) {
        double tx = getTx();
        double ta = getTa();
        
        double kP_turn = 0.03;
        double kP_forward = 0.3;
  
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
          LeftSpeed = 0; RightSpeed = 0;
        } else {
          LeftSpeed =  left; RightSpeed = right;
        }
    } else if (!hasTarget()) {
      if (getTime() < 2.0) {
        LeftSpeed = -0.3; RightSpeed = 0.3;
    } else if (getTime() < 4.0) {
        LeftSpeed = 0.3; RightSpeed = -0.3;
    } else {
        LeftSpeed = 0; RightSpeed = 0; 
    }
    }
              return new DriveSpeeds(LeftSpeed, RightSpeed);
 }
        private void setDriveSpeeds(double left, double right) {
          drive.rawTank(left, right);
        }

        public double getTime(){
          return timer.get();
        }
        public void resetTime(){
          timer.reset();
        }
    
        @Override
        public void simulationPeriodic(){

    if (visionSim != null) {
      Pose3d robotPose3d = drive.getPose3d(); 
      visionSim.update(robotPose3d);
  }
  double txLocal = 0, tyLocal = 0, taLocal = 0;
  boolean tvLocal = false;

  if (camera != null) {
      try {
          var result = camera.getLatestResult();
          if (result.hasTargets()) {
              var best = result.getBestTarget();
              txLocal = best.getYaw();       // graus
              tyLocal = best.getPitch();     // graus
              taLocal = best.getArea();      // confirmar unidade
              tvLocal = true;
          }
      } catch (Exception e) {
          System.out.println("[Vision] getLatestResult erro: " + e.getMessage());
      }
  }

  this.tx = txLocal; this.ty = tyLocal; this.ta = taLocal; this.tv = tvLocal;
  limelight.getEntry("tv").setNumber(tvLocal ? 1.0 : 0.0);
  limelight.getEntry("tx").setNumber(txLocal);
  limelight.getEntry("ty").setNumber(tyLocal);
  limelight.getEntry("ta").setNumber(taLocal);

  SmartDashboard.putBoolean("Sim Target", tvLocal);
  SmartDashboard.putNumber("Sim TX", txLocal);
  SmartDashboard.putNumber("Sim TA", taLocal);


  Logger.recordOutput("Vision/tx", txLocal);
  Logger.recordOutput("Vision/ta", taLocal);
  Logger.recordOutput("Vision/tv", tvLocal ? 1.0 : 0.0);
}
    }


