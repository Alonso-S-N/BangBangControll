  package frc.robot.SubSystem;

  import com.ctre.phoenix.motorcontrol.InvertType;
  import com.ctre.phoenix.motorcontrol.NeutralMode;
  import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
  import frc.robot.Constants;
import pabeles.concurrency.ConcurrencyOps.NewInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.Odometry3d;
import edu.wpi.first.networktables.NetworkTableEntry;
  import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
  import edu.wpi.first.wpilibj.Encoder;
  import edu.wpi.first.wpilibj.RobotBase;
  import edu.wpi.first.wpilibj.Timer;
  import edu.wpi.first.wpilibj.smartdashboard.Field2d;
  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
  import edu.wpi.first.wpilibj2.command.SubsystemBase;
  import edu.wpi.first.wpilibj.drive.DifferentialDrive;
  import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
  import edu.wpi.first.wpilibj.simulation.EncoderSim;
  import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
  import edu.wpi.first.math.system.plant.DCMotor;
  import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.Arena2024Crescendo;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import edu.wpi.first.math.geometry.Pose3d;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeProcessorSimulation;
import org.littletonrobotics.junction.Logger;


  public class Drive extends SubsystemBase {
    private final Field2d field = new Field2d();
    private final NetworkTableEntry poseEntry;
    public EncoderSim leftEncoderSim;
    public EncoderSim rightEncoderSim;
    public ADXRS450_GyroSim gyroSim;
    public DifferentialDrivetrainSim driveSim;
    private double _debugX = 0.0;
    private double _lastTime = 0.0;
    private double simYaw = 0.0;
    private final  Arena2025Reefscape arena = new Arena2025Reefscape();

    private final Pose3d poseA = new Pose3d();
    private final Pose3d poseB = new Pose3d();
    
    private final WPI_VictorSPX m_leftLeader  = new WPI_VictorSPX(Constants.LMot);
    private final WPI_VictorSPX m_rightLeader = new WPI_VictorSPX(Constants.RMot);
    private final WPI_VictorSPX m_leftFollower  = new WPI_VictorSPX(Constants.LMot2);
    private final WPI_VictorSPX m_rightFollower = new WPI_VictorSPX(Constants.RMot2);
    
    public final Encoder leftEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
    private final Encoder rightEncoder = new Encoder(6, 7, true, Encoder.EncodingType.k4X);
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private BangBangSub braceta = null;
    private ObjectSim object;
    
        private final double diametroRoda = 0.06; // 6 cm
        private final DifferentialDriveOdometry odometry;
          public Drive(ObjectSim object) {
            this.object = object;
           resetEncoders();
           gyro.reset();
    
        this.odometry = new DifferentialDriveOdometry(
            getHeading(),
            leftEncoder.getDistance(),
            rightEncoder.getDistance()
        );
    
        SmartDashboard.putData("Field", field);
        poseEntry = NetworkTableInstance.getDefault()
                    .getTable("SmartDashboard")
                    .getEntry("RobotPose");
    
        if (RobotBase.isSimulation()) {
            leftEncoderSim  = new EncoderSim(leftEncoder);
            rightEncoderSim = new EncoderSim(rightEncoder);
            gyroSim         = new ADXRS450_GyroSim(gyro);
    
            driveSim = DifferentialDrivetrainSim.createKitbotSim(
                DifferentialDrivetrainSim.KitbotMotor.kDualCIMPerSide,
                DifferentialDrivetrainSim.KitbotGearing.k10p71,
                DifferentialDrivetrainSim.KitbotWheelSize.kSixInch,
                null
            );
          

             // Inicializa a simulação
       SimulatedArena.getInstance();
          SimulatedArena.overrideInstance(arena);

       SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
    new Pose2d(2, 2, Rotation2d.fromDegrees(90))));

     SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2,2)));
    
            _lastTime = Timer.getFPGATimestamp();
              
        }
    
        reqDrive();   
    }
    @Override
    public void simulationPeriodic() {
      if (driveSim != null) {
         //aplica tensões (motores [-1..1] -> volts)
        driveSim.setInputs(m_leftLeader.get() * 12.0, m_rightLeader.get() * 12.0);
        
        Pose3d currentRobotPose = getPose3d();
        object.setRobotPose(currentRobotPose);
        object.update();
    
        // atualiza modelo físico (20ms)
        driveSim.update(0.02);

        double distancePerPulse = (Math.PI * diametroRoda)/2048;
    
        leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
        rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
        leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
    
        gyroSim.setAngle(driveSim.getHeading().getDegrees());

        Logger.recordOutput("MyPose", poseA);
        Logger.recordOutput("MyPoseArray", poseA, poseB);
        Logger.recordOutput("MyPoseArray", new Pose3d[] {poseA, poseB});
        
      }
    }

    @Override
    public void periodic() {
      odometry.update( getHeading(),leftEncoder.getDistance(),rightEncoder.getDistance());
      Pose2d pose = odometry.getPoseMeters();
      Logger.recordOutput("Drive/Pose", new Pose2d(pose.getX(), pose.getY(), pose.getRotation()));
      
      
      field.setRobotPose(pose);
      poseEntry.setDoubleArray(new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
      if (pose.getX() != _debugX) {
        _debugX = pose.getX();
        double now = Timer.getFPGATimestamp();
        double rate = 1.0 / (now - _lastTime);
        _lastTime = now;

        Pose3d pose3d = new Pose3d(
          new Translation3d(pose.getX(), pose.getY(), 0.0), // z = 0
          new Rotation3d(0.0, 0.0, pose.getRotation().getRadians()) // só yaw
      );

      if (RobotBase.isSimulation()) {
        Logger.recordOutput("Robot/Pose3d", pose3d);
      if (braceta != null){
        braceta.setRobotPose3d(pose3d);
      }
     }
    }
  }


  public void startProjectileLaunch() {
    if (!RobotBase.isSimulation()) {
        return; // Só deve rodar na simulação
    }
    Pose3d robotPose = getPose3d();
    // Posição inicial: 0.5m acima do robô
    Pose3d startPose = robotPose.plus(
            new Transform3d(
                new Translation3d(0.0, 0.0, 0.5),
                new Rotation3d()
            )
        );

    // Exemplo: Velocidade de 5 m/s, 1m/s na lateral, 4 m/s para cima
    Translation3d initialVelocity = new Translation3d(5.0, 1.0, 4.0); 

    object.launch(startPose, initialVelocity,robotPose.getRotation());
}

    public Pose3d getPose3d(){
      return new Pose3d(
        new Translation3d(getPose().getX(), getPose().getY(), 0.0),
        new Rotation3d(0.0, 0.0, getHeading().getRadians())
      );
    }

    public Rotation2d getHeading() {
      return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public Pose2d getPose() {
      return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
      resetEncoders();
      gyro.reset(); 
      odometry.resetPosition(getHeading(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
    }

    public void resetEncoders() {
      leftEncoder.reset();
      rightEncoder.reset();
    }

    public void reqDrive() {
      resetOdometry(getPose());
      m_leftFollower.follow(m_leftLeader);
      m_rightFollower.follow(m_rightLeader);

      m_rightLeader.setInverted(true);
      m_leftLeader.setInverted(false);

      m_leftFollower.setInverted(InvertType.FollowMaster);
      m_rightFollower.setInverted(InvertType.FollowMaster);

      m_leftLeader.setNeutralMode(NeutralMode.Brake);
      m_leftFollower.setNeutralMode(NeutralMode.Brake);
      m_rightLeader.setNeutralMode(NeutralMode.Brake);
      m_rightFollower.setNeutralMode(NeutralMode.Brake);

      double distancePerPulse = (Math.PI * diametroRoda) / 2048;
      leftEncoder.setDistancePerPulse(distancePerPulse);
      rightEncoder.setDistancePerPulse(distancePerPulse);
    }

    public void rawTank(double left, double right) {
      m_leftLeader.set(left);
      m_rightLeader.set(right);

      Logger.recordOutput("Drive/LeftSetpoint", left);
      Logger.recordOutput("Drive/RightSetpoint", right);
    }

    public void stop() {
      m_leftLeader.stopMotor();
      m_rightLeader.stopMotor();
    }
  }