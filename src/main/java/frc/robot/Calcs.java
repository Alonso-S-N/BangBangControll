package frc.robot;

import edu.wpi.first.wpilibj.Joystick;


public class Calcs {
    

   Joystick joyDeliciu;

    public static class DriveSpeeds {
        public final double left;
        public final double right;
        public DriveSpeeds(double left, double right) {
            this.left = left;
            this.right = right;
        }
    }

    public static DriveSpeeds calculateAnalogDrive(Joystick joyDeliciu, double B_Speed) {
        double X = joyDeliciu.getX();
        double Y = -joyDeliciu.getY();
        

        double magnitude = Math.hypot(X, Y);
        if (magnitude < Constants.deadZone) return new DriveSpeeds(0, 0);

        magnitude = Math.min(1, Math.max(-1, magnitude));
        double seno = Y / magnitude;
        double LS = 0, Rs = 0;
      

        if (X >= 0 && Y >= 0) {
            LS = magnitude * B_Speed;
            Rs = (2 * seno - 1) * magnitude * B_Speed;
        } else if (X < 0 && Y >= 0) {
            LS = (2 * seno - 1) * magnitude * B_Speed;
            Rs = magnitude * B_Speed;
        } else if (X >= 0 && Y < 0) {
            LS = -magnitude * B_Speed;
            Rs = (2 * seno + 1) * magnitude * B_Speed;
        } else if (X < 0 && Y < 0) {
            LS = (2 * seno + 1) * magnitude * B_Speed;
            Rs = -magnitude * B_Speed;
        }

        return new DriveSpeeds(LS, Rs);
    }

    public static DriveSpeeds calculateAnalogDrive2(Joystick joyDeliciu, double B_Speed){
        double X1 = joyDeliciu.getRawAxis(4);
        double Y2 = joyDeliciu.getRawAxis(5);
        
        double magnitude2 = Math.hypot(X1, Y2);
        magnitude2 = Math.max(-1, Math.min(1, magnitude2));
        
        double seno2 = Y2 / magnitude2;

        double rapidao = 0, rapidao2 = 0;
        //quadrante 1
      if (X1 >= 0 && Y2 >= 0) {
        rapidao = magnitude2 * B_Speed;
        rapidao2 = (2 * seno2 -1) * magnitude2 * B_Speed;	
       } 
      //quadrante 2
      else if (X1 < 0 && Y2 >= 0) {  
        rapidao = (2 * seno2 - 1) * magnitude2 * B_Speed;
        rapidao2 = magnitude2 * B_Speed;
       } 
      //quadrante 3
      else if (X1 >= 0 && Y2 < 0) {
        rapidao = magnitude2 * B_Speed * -1;	
        rapidao2 = (2 * seno2 + 1) * magnitude2 * B_Speed;
       } 
      //quadrante 4
       else if (X1 < 0 && Y2 < 0) {
        rapidao = (2 * seno2 + 1) * magnitude2 * B_Speed;
        rapidao2 = magnitude2 * B_Speed * -1;
       }

       return new DriveSpeeds(rapidao,rapidao2);
    }

    public static DriveSpeeds calculateTriggerDrive(Joystick joyDeliciu, double B_Speed) {
        double RT = joyDeliciu.getRawAxis(Constants.RT);
        double LT = joyDeliciu.getRawAxis(Constants.LT);

        if (RT > Constants.deadZone) {
            double val = RT * B_Speed;
            return new DriveSpeeds(val, val);
        } else if (LT > Constants.deadZone) {
            double val = -LT * B_Speed;
            return new DriveSpeeds(val, val);
        }

        return new DriveSpeeds(0, 0);
    }

    public static DriveSpeeds calculatePovDrive(Joystick Joydeliciu, double B_Speed) {
        switch (Joydeliciu.getPOV()) {
            case 0: return new DriveSpeeds(0.5, 0.5);
            case 45: return new DriveSpeeds(0.5, 0.25);
            case 90: return new DriveSpeeds(0.5, -0.5);
            case 135: return new DriveSpeeds(-0.5, -0.25);
            case 180: return new DriveSpeeds(-0.5, -0.5);
            case 225: return new DriveSpeeds(-0.25, -0.5);
            case 270: return new DriveSpeeds(-0.5, 0.5);
            case 315: return new DriveSpeeds(-0.25, 0.5);
            default: return new DriveSpeeds(0, 0);
        }
    }
}