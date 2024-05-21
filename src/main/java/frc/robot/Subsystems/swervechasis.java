package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.backLeft;
import frc.robot.Constants.backRight;
import frc.robot.Constants.frontLeft;
import frc.robot.Constants.frontRight;


public class swervechasis extends SubsystemBase {

    swervemodule FrontLeft, FrontRight, BackLeft, BackRight;

    Translation2d FLTranslation = new Translation2d(0, 0);
    Translation2d FRTranslation = new Translation2d(0, 0);
    Translation2d BLTranslation = new Translation2d(0, 0);
    Translation2d BRTranslation = new Translation2d(0, 0);

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(FLTranslation, FRTranslation, BLTranslation, BRTranslation);

    public swervechasis(){
       
        FrontLeft = new swervemodule(1,
            frontLeft.DrivePort, 
            frontLeft.TurnPort, 
            frontLeft.EncPort, 
            frontLeft.kP, 
            frontLeft.kI, 
            frontLeft.kD, 
            frontLeft.DrivemotorReversed, 
            frontLeft.TurnmotorReversed);
        
        
        FrontRight = new swervemodule(2,
            frontRight.DrivePort, 
            frontRight.TurnPort, 
            frontRight.EncPort, 
            frontRight.kP, 
            frontRight.kI, 
            frontRight.kD, 
            frontRight.DrivemotorReversed, 
            frontRight.TurnmotorReversed);
        
        BackLeft = new swervemodule(4, // Creo que es 4 
            backLeft.DrivePort, 
            backLeft.TurnPort, 
            backLeft.EncPort, 
            backLeft.kP, 
            backLeft.kI, 
            backLeft.kD, 
            backLeft.DrivemotorReversed, 
            backLeft.TurnmotorReversed);
        
        BackRight = new swervemodule(3,
            backRight.DrivePort, 
            backRight.TurnPort, 
            backRight.EncPort, 
            backRight.kP, 
            backRight.kI, 
            backRight.kD, 
            backRight.DrivemotorReversed, 
            backRight.TurnmotorReversed);
        
        
    }   

}
