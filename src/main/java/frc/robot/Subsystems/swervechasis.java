package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
 
// Librerias 
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

    AHRS navX =  new AHRS(SPI.Port.kMXP);
    double navXoffset = 170;

    double distancia = 39;
    Translation2d FLTranslation = new Translation2d(-distancia, -distancia);
    Translation2d FRTranslation = new Translation2d(-distancia, distancia);
    Translation2d BLTranslation = new Translation2d(distancia, -distancia);
    Translation2d BRTranslation = new Translation2d(distancia, distancia);

    // Creacion del metodo de la kinematica 
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(FLTranslation, FRTranslation, BLTranslation, BRTranslation);
    //Creacion del metodo para lo odometria 
    
    
    

    private swervemodule FrontLeft = new swervemodule(1,
            frontLeft.DrivePort, 
            frontLeft.TurnPort, 
            frontLeft.EncPort,
            frontLeft.offset, 
            frontLeft.kP, 
            frontLeft.kI, 
            frontLeft.kD, 
            frontLeft.DrivemotorReversed, 
            frontLeft.TurnmotorReversed);
        
        
    private swervemodule FrontRight = new swervemodule(2,
            frontRight.DrivePort, 
            frontRight.TurnPort, 
            frontRight.EncPort,
            frontRight.offset,
            frontRight.kP, 
            frontRight.kI, 
            frontRight.kD, 
            frontRight.DrivemotorReversed, 
            frontRight.TurnmotorReversed);
        
    private swervemodule BackLeft = new swervemodule(3,
            backLeft.DrivePort, 
            backLeft.TurnPort, 
            backLeft.EncPort, 
            backLeft.offset,
            backLeft.kP, 
            backLeft.kI, 
            backLeft.kD, 
            backLeft.DrivemotorReversed, 
            backLeft.TurnmotorReversed);
        
    private  swervemodule BackRight = new swervemodule(4,
            backRight.DrivePort, 
            backRight.TurnPort, 
            backRight.EncPort, 
            backRight.offset,
            backRight.kP, 
            backRight.kI, 
            backRight.kD, 
            backRight.DrivemotorReversed, 
            backRight.TurnmotorReversed);

    SwerveModulePosition positions[] = {FrontLeft.getPosition(), FrontRight.getPosition(), BackLeft.getPosition(), BackRight.getPosition()};

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), positions, new Pose2d(0, 0, getRotation2d()));
  

    
    
    public swervechasis(){
        navX.setAngleAdjustment(navXoffset);
    }   

    // Creacion del metodo para los tipos de velocidades
    public void setChassisSpeed(double speedX, double speedY, double speedZ){

        // Asignacion de las velocidades ya creadas a los modulos 
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(speedX, speedY, speedZ*0.2));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 0.7);

        setModuleStates(states);

    }

    // Creacion del metodo para las velocidades orientadas al campo 
    public void setFieldOrientedSpeed(double speedX, double speedY, double speedZ) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedZ * 0.2, getRotation2d());
        
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        // Kinematicas para asingar la velocidad y cantidad de energia necesaria para el swerve 
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 0.7);
        setModuleStates(states);
    }

    // Metodo para asignar a cada modulo su estado 
    public void setModuleStates (SwerveModuleState[] state){
        FrontLeft.setDesiredState(state[0]);
        FrontRight.setDesiredState(state[1]);
        BackLeft.setDesiredState(state[2]);
        BackRight.setDesiredState(state[3]);

    }

    // Metodo para adquirir el angulo con la navx 
    public double getAngle(){
        return navX.getAngle();
    }

    // Metodo para que los modulos sigan el angulo mas cercano 
    public Rotation2d getRotation2d(){
        return new Rotation2d(getAngle());
    }

    //Metodo para obtener la posicion en metros con la odometria 
    public Pose2d getPose2d(){
        return odometry.getPoseMeters();
    }


    @Override
    public void periodic(){
        //Mostrar en la smartdashboard los valores que quieras 
        SmartDashboard.putNumber("NavX", getAngle());
        SmartDashboard.putNumber("robot X ", getPose2d().getX());
        SmartDashboard.putNumber("robot Y ", getPose2d().getY());
        
        //Asignar un numero de posicion a cada modulo 
        positions[0] = FrontLeft.getPosition();
        positions[1] = FrontRight.getPosition();
        positions[2] = BackLeft.getPosition();
        positions[3] = BackRight.getPosition();
        
        //Actualizacion de la posicion segun la odometria 
        odometry.update(getRotation2d(), positions); 

    }
}




