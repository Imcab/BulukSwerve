package frc.robot;

public class Constants {

    public static final double meterspersecond = 5.7912;
    // RPM 5820

    public static final class frontLeft{

        public static final int DrivePort = 3; //3 
        public static final int TurnPort = 4; //4
        public static final int EncPort = 0;
        public static final double offset = 138; //48 o 138 o 228
 
        public static final boolean DrivemotorReversed = false;
        public static final boolean TurnmotorReversed = false;
        public static final double PIDSTATUS = -1.0;
 
        public static final double kP = 0.008;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static final class frontRight{

        public static final int DrivePort = 1; //1
        public static final int TurnPort = 11; //11
        public static final int EncPort = 1; //1
        public static final double offset = 92; //2 o 92 o 182
 
        public static final boolean DrivemotorReversed = false;
        public static final boolean TurnmotorReversed = true; //false
        public static final double PIDSTATUS = 1.0; //-1
 
        public static final double kP = 0.008;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static final class backLeft{

        public static final int DrivePort = 6; //6
        public static final int TurnPort = 9; //9
        public static final int EncPort = 2; //2
        public static final double offset = 340; // 250.50 o 340 o 430
 
        public static final boolean DrivemotorReversed = false;
        public static final boolean TurnmotorReversed = false; //false
        public static final double PIDSTATUS = -1.0; //-1
 
        public static final double kP = 0.008;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static final class backRight{

        public static final int DrivePort = 7; //7
        public static final int TurnPort = 5; //5
        public static final int EncPort = 3; //3
        public static final double offset = 336; //246 o 336 o 426
 
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = false;
        public static final double PIDSTATUS = -1.0;
 
        public static final double kP = 0.008;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    
    

    
}


// :v