package frc.robot;

public final class Constants {
 /* ======= Motores ======== */
  public static int LMot = 3;
  public static int RMot = 2;
  public static int LMot2 = 4;
  public static int RMot2 = 1;
  public static double deadZone = 0.04;
  public static double NegativeDeadZone = -0.04;
  public static int povDeadZone = -1;
  /*  ====================== */

 /* ======= Buttons && Axis ======== */
  public static int RT = 3;
  public static int LT = 2;
  public static int a = 1;
  public static int b = 2;
  public static int x = 3;
  public static int y = 4;
  public static int joy = 0;
  public static int JoyDelicioso = 1;
  public static int Y2 = 5;
  public static int X1 = 4;
  public static int RB = 6;
  public static int LB = 5;
 /* ====================== */

  /* ======= Sparks ======== */
  public static int m_peteco_peteco = 8;
  public static int m_peteco_peteco2 = 9;
  public static int m_levar_para_lancamento = 7;
  public static int m_angulador = 10;
  public static int m_coletor = 5;
  public static int m_rodaColetor = 6;
  /* ====================== */

   /* ======= DIO ======== */
  public static int sensorPos = 0;
  public static int InfraDio = 2;
  /* ====================== */

  /* ======= Angulin ======== */
  public static double angulinMax = 90.0;
  public static double angulinMin = 0.0;
  /* ====================== */

  /* ======= Auto ======= */
  public static double autonomousTime = 15.0;
  public static double autonomousLocNeg = -0.25;
  public static double LocTimeUltra = 4;
  public static double autonomousBraceta = 3.0;
  public static double autonomousLoc = 0.25;
  public static double autonomousTaxi = 2.0;
  /* ====================== */
/* ======= limelight ======= */
  public static double targetArea = 1.0;
  /* ====================== */

  public static double tolerancia = 0.05;
  public static double GrausMax = 360.0;
  public static final double MAX_EXTENSION_METERS = 0.6;

   /* ======= FeedFoward ======== */
  public static double g = 9.81;
  public static double ks = 0.21;
  public static double kg = 0.86;
  public static double kv = 2.15;
  public static double ka = 0;
  /* ====================== */

   /* ======= Voltage ======== */
  public static double MaxVoltage = 12;
  public static double MinVoltage = -12;
  public static double batteryV = 12.0;
  /* ====================== */

/* ======= Info Robo ======= */
  public static  double massKg = 7.75;
  public static double gearRatio = 100.0;
  public static double motorStallNm = 2.6;
  public static double gearboxEff = 0.9;
  public static double wheelRadiusMeters = 0.6;
  /* ====================== */
}
