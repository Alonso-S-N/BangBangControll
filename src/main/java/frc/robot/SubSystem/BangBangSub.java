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
  import edu.wpi.first.wpilibj.DigitalInput;
  import edu.wpi.first.wpilibj.DutyCycle;
  import edu.wpi.first.wpilibj.Encoder;
  import edu.wpi.first.wpilibj.Joystick;
  import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
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
  import edu.wpi.first.wpilibj.Timer;
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

  //Sensors and motors:
    public final SparkMax MotorUp = new SparkMax(Constants.m_peteco_peteco, MotorType.kBrushed);
    public final SparkMax MotorDown = new SparkMax(Constants.m_peteco_peteco2, MotorType.kBrushed);
    public final SparkMax LevarPraTacarOsCoiso = new SparkMax(Constants.m_levar_para_lancamento, MotorType.kBrushed);
    public final SparkMax PegaOsCoisoMotor = new SparkMax(Constants.m_coletor, MotorType.kBrushless);
    public final SparkMax RodaColetorMotor = new SparkMax(Constants.m_rodaColetor, MotorType.kBrushed);

    public final DigitalInput InfraRed = new DigitalInput(Constants.InfraDio);
    public final Encoder ShooterEncoder = new Encoder(0, 1);
    public final EncoderSim ShooterEncoderSim = new EncoderSim(ShooterEncoder);
    public Timer time = new Timer();

    private DIOSim irSim;

  // Angulo, duty cycle limites //
    private final double DUTY_MIN = 0.02;  
    private final double DUTY_MAX = 0.98;
    private final double ANGLE_MIN = 0.0;  
    private final double ANGLE_MAX = 90;

    private double targetAngleDeg = 45.0;
  // method usuals for the subsystem //
    boolean ballDetected = false;
    boolean shotFinished = false;
    private double lastValidDistance = 0.0;
    private boolean hadVision = false;
    double launchTimer = 0;

    double g = 9.81;

  // Controllers and subsystems//

    BangBangController controller = new BangBangController();
    private Drive drive;
    private Vision vision;
    private ObjectSim objectSim;
    // Distance calculations //
    private Pose3d robotPose3d = new Pose3d();
    private double shooterRaio = 0.045; // raio em metros.
    private double shooterAng = Math.toRadians(45);
  
    

    public BangBangSub(Vision vision, Drive drive, ObjectSim objectSim) {
      this.vision = vision;
      this.drive = drive;
      this.objectSim = objectSim;
      

      irSim = new DIOSim(InfraRed);
      ShooterEncoder.setDistancePerPulse(60.0 / (2048.0 * 4.0));

      final  SparkMaxConfig kCoast = new SparkMaxConfig();
      kCoast.idleMode(SparkBaseConfig.IdleMode.kCoast);
      //kCoast.openLoopRampRate(1.0);
      controller.setTolerance(50);
      final SparkMaxConfig Kbrake = new SparkMaxConfig();
      Kbrake.idleMode(SparkBaseConfig.IdleMode.kBrake);
      Kbrake.openLoopRampRate(0.4);

      ShooterEncoder.setReverseDirection(false);
    /* ============================== Motores ========================================*/
      RodaColetorMotor.configure(kCoast,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      MotorUp.configure(kCoast,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      MotorDown.configure(kCoast,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      PegaOsCoisoMotor.configure(Kbrake,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      LevarPraTacarOsCoiso.configure(kCoast,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      

    }

  

    public void ResetPetecoPeteco(){
    controller.setSetpoint(0);
    }

    public void BangBangBalls(double setpoint){
      double RPM = ShooterEncoder.getRate();
      double output = controller.calculate(RPM, setpoint);
      MotorDown.set(-output);
      MotorUp.set(-output);
      System.out.println("Rpm Encoder" + ShooterEncoder.getRate());

      if (Math.abs(RPM- setpoint) < 75.0) {
        LevarPraTacarOsCoiso.set(1.0);
      } else {
        stopLevarOsCoiso();
      }
      
    } 
  public void autoShooter() {

      if (!InfraRed.get()) {
          ballDetected = true;
          shotFinished = false;
          launchTimer = 0;
      }

      if (!ballDetected) {
          StopShooter();
          stopLevarOsCoiso();
          return;
      }

      // --- LÓGICA DE VISÃO --- //
      boolean isReadyToShoot = vision.alignWithTag_P();
      double distance;
        
    if (isReadyToShoot){
      if (vision.hasTarget()) {
          distance = getDistanceToTag();
          lastValidDistance = distance;
          hadVision = true;
      } else if (hadVision) {
          // perdeu visão, mas já tínhamos uma distância
          distance = lastValidDistance;
      } else {
          // nunca teve visão, não dá pra calcular RPM
          StopShooter();
          return;
      }

      // calcula RPM baseado na distância (real ou "lembrada")
      double setpointRPM = getRPMFromDistance(distance);
      //double setpointRPM = 500; // valor fixo para testes
      double currentRPM = ShooterEncoder.getRate();

      double output = controller.calculate(currentRPM, setpointRPM);

      MotorUp.set(-output);
      MotorDown.set(-output);

      // alimenta a bola se o RPM estiver ok
      if (!shotFinished && Math.abs(currentRPM - setpointRPM) < 75.0) {
          LevarPraTacarOsCoiso.set(1.0);
          tryToLauch(setpointRPM);
      } else {
          stopLevarOsCoiso();
      }

      // detecta se já atirou
      if (!InfraRed.get() && LevarPraTacarOsCoiso.get() > 0) {
          shotFinished = true;
      }

      // cooldown
      if (shotFinished) {
          launchTimer += 0.02;

          if (launchTimer > 0.3) {
              StopShooter();
              stopLevarOsCoiso();
              ballDetected = false;
              shotFinished = false;
              isReadyToShoot = false;
              vision.Resetalighted();
          }
      }
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
    double currentRPM = MotorUp.getEncoder().getVelocity();
    if (Math.abs(currentRPM - TargetRpm) < 75.0) {
      
      Pose3d robotPose = drive.getPose3d();
      Translation3d robotRelativeVelocity = calculateLaunchVelocity(TargetRpm);

      Pose3d launchPose = objectSim.getObjectPose();

      objectSim.launch(
              launchPose, 
              robotRelativeVelocity,
              robotPose.getRotation()
          );
    }
  }
  private double getRPMFromDistance(double distanceMeters) {
    double shooterHeight = 0.64; // altura do shooter (m)
    double targetHeight = 2.64;  // m (hub 2022)
    double deltaH = targetHeight - shooterHeight; // 2.00 m

    double angleDeg = 45.0; // ângulo fixo
    double angleRad = Math.toRadians(angleDeg);

    double r = 0.05; // raio da roda do shooter (m)
    double x = distanceMeters;

    // Denominador: 2 * cos^2(angle) * ( x * tan(angle) - deltaH )
    double cos2 = Math.cos(angleRad) * Math.cos(angleRad);
    double inner = x * Math.tan(angleRad) - deltaH;
    double denom = 2.0 * cos2 * inner;

    // Se denom <= 0, não é possivel o arremessar com esse ângulo.
    if (denom <= 0.0) {
        StopShooter();
        System.out.println("impossivel acertar o alvo desta posição");
    }

    double numerator = g * x * x;
    double v = Math.sqrt(numerator / denom); // velocidade inicial necessária (m/s)

    double wheelCircumference = 2.0 * Math.PI * r; // m/rev
    double revPerSec = v / wheelCircumference;
    double rpm = revPerSec * 60.0;


    double speedFactor = 1.10; // 1.0 = sem correção, 1.10 = +10% de velocidade (eficiencia do motor redLine)
    rpm *= speedFactor;

    // limites práticos
    rpm = MathUtil.clamp(rpm, 0.0, 5000);

    return rpm;

  }  

    public void setRobotPose3d(Pose3d pose) {
      this.robotPose3d = pose;
    }
    public void stopLevarOsCoiso(){
      LevarPraTacarOsCoiso.set(0.0);
    }

    public void StopShooter(){
    MotorUp.set(0.0);
    MotorDown.set(0);
    }
    
    public void Cuspir(){
    PegaOsCoisoMotor.set(0.3);
    }
    public void Pegar(){
    PegaOsCoisoMotor.set(-0.3);
    }
    public void stopExtensor(){
    PegaOsCoisoMotor.set(0.0);
    }

    public void PuxarOsCoiso(){
      RodaColetorMotor.set(0.3);
    }
    public void LargarOsCoiso(){
      RodaColetorMotor.set(-0.3);
    }

    public double getDistanceToTag() {
      double cameraHeight = 0.6;
      double targetHeight = 2.64;   //2.64 metros (hub 2022)
      double cameraAngle = 20;  
      double ty = vision.getTy();
  
      return (targetHeight - cameraHeight) /
            Math.tan(Math.toRadians(cameraAngle + ty));
  }



    @Override
    public void simulationPeriodic() {
      double motorOutput = MotorUp.get();
      double simulatedRPM = motorOutput * 5000; 
      ShooterEncoderSim.setRate(simulatedRPM / 60.0);
      irSim.setValue(true);
    }

    @Override
    public void periodic() {
     // System.out.println("Encoder RPM" + ShooterEncoder.getRate());
      System.out.println("InfraRed" + InfraRed.get());
    }
  }
