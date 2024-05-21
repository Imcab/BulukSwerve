package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.backLeft;
import frc.robot.Constants.backRight;
import frc.robot.Constants.frontLeft;
import frc.robot.Constants.frontRight;


public class swervechasis extends SubsystemBase {

    AHRS navX;
    double navXoffset = 0;

    swervemodule FrontLeft, FrontRight, BackLeft, BackRight;

    Translation2d FLTranslation = new Translation2d(0, 0);
    Translation2d FRTranslation = new Translation2d(0, 0);
    Translation2d BLTranslation = new Translation2d(0, 0);
    Translation2d BRTranslation = new Translation2d(0, 0);

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(FLTranslation, FRTranslation, BLTranslation, BRTranslation);

    SwerveModulePosition[] positions = {FrontLeft.getPosition(), FrontRight.getPosition(), BackLeft.getPosition(), BackRight.getPosition()};

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), positions, new Pose2d(0, 0, getRotation2d()));

    public swervechasis(){

        navX = new AHRS(SPI.Port.kMXP);
        navX.setAngleAdjustment(navXoffset);
       
        FrontLeft = new swervemodule(1,
            frontLeft.DrivePort, 
            frontLeft.TurnPort, 
            frontLeft.EncPort,
            frontLeft.offset, 
            frontLeft.kP, 
            frontLeft.kI, 
            frontLeft.kD, 
            frontLeft.DrivemotorReversed, 
            frontLeft.TurnmotorReversed);
        
        
        FrontRight = new swervemodule(2,
            frontRight.DrivePort, 
            frontRight.TurnPort, 
            frontRight.EncPort,
            frontRight.offset,
            frontRight.kP, 
            frontRight.kI, 
            frontRight.kD, 
            frontRight.DrivemotorReversed, 
            frontRight.TurnmotorReversed);
        
        BackLeft = new swervemodule(3,
            backLeft.DrivePort, 
            backLeft.TurnPort, 
            backLeft.EncPort, 
            backLeft.offset,
            backLeft.kP, 
            backLeft.kI, 
            backLeft.kD, 
            backLeft.DrivemotorReversed, 
            backLeft.TurnmotorReversed);
        
        BackRight = new swervemodule(3,
            backRight.DrivePort, 
            backRight.TurnPort, 
            backRight.EncPort, 
            backRight.offset,
            backRight.kP, 
            backRight.kI, 
            backRight.kD, 
            backRight.DrivemotorReversed, 
            backRight.TurnmotorReversed);
    }   

    public void setChassisSpeed(double speedX, double speedY, double speedZ){

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(speedX, speedY, speedZ));

        setModuleStates(states);

    }

    public void setFieldOrientedSpeed(double speedX, double speedY, double speedZ) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedZ, getRotation2d());
        
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, 0.99);
        setModuleStates(states);
    }

    public void setModuleStates (SwerveModuleState[] state){
        FrontLeft.setDesiredState(state[0]);
        FrontRight.setDesiredState(state[1]);
        BackLeft.setDesiredState(state[2]);
        BackRight.setDesiredState(state[3]);

    }

    public double getAngle(){
        return navX.getAngle();
    }

    public Rotation2d getRotation2d(){
        return new Rotation2d(getAngle());
    }

    public Pose2d getPose2d(){
        return odometry.getPoseMeters();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("NavX", getAngle());
        SmartDashboard.putNumber("robot X ", getPose2d().getX());
        SmartDashboard.putNumber("robot Y ", getPose2d().getY());

        positions[0] = FrontLeft.getPosition();
        positions[1] = FrontRight.getPosition();
        positions[2] = BackLeft.getPosition();
        positions[3] = BackRight.getPosition();

        odometry.update(getRotation2d(), positions);

    }
}




