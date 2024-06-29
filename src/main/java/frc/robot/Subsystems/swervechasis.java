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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.backLeft;
import frc.robot.Constants.backRight;
import frc.robot.Constants.frontLeft;
import frc.robot.Constants.frontRight;



public class swervechasis extends SubsystemBase {

    AHRS navX =  new AHRS(SPI.Port.kMXP);

    double distancia = 39;
    Translation2d FLTranslation = new Translation2d(distancia, -distancia);
    Translation2d FRTranslation = new Translation2d(-distancia, -distancia); 
    Translation2d BLTranslation = new Translation2d(distancia, distancia);  
    Translation2d BRTranslation = new Translation2d(-distancia, distancia);  
    // Creacion del metodo de la kinematica t
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(FLTranslation, FRTranslation, BLTranslation, BRTranslation);
    //Creacion del metodo para lo odometria 
    
    private final StructArrayPublisher<SwerveModuleState> publisherModuleStates;

    private final StructArrayPublisher<SwerveModuleState> publisherModuleStates2;

    private final StructArrayPublisher<Rotation2d> publisherRobotHeading;

    private final StructArrayPublisher<Pose2d> publisherOdometry;
    
    

    private swervemodule FrontLeft = new swervemodule(1,
            frontLeft.DrivePort, 
            frontLeft.TurnPort, 
            frontLeft.EncPort,
            frontLeft.offset, 
            frontLeft.kP, 
            frontLeft.kI, 
            frontLeft.kD, 
            frontLeft.DrivemotorReversed, 
            frontLeft.TurnmotorReversed,
            frontLeft.PIDSTATUS);
        
        
    private swervemodule FrontRight = new swervemodule(2,
            frontRight.DrivePort, 
            frontRight.TurnPort, 
            frontRight.EncPort,
            frontRight.offset,
            frontRight.kP, 
            frontRight.kI, 
            frontRight.kD, 
            frontRight.DrivemotorReversed, 
            frontRight.TurnmotorReversed,
            frontRight.PIDSTATUS);
        
    private swervemodule BackLeft = new swervemodule(3,
            backLeft.DrivePort, 
            backLeft.TurnPort, 
            backLeft.EncPort, 
            backLeft.offset,
            backLeft.kP, 
            backLeft.kI, 
            backLeft.kD, 
            backLeft.DrivemotorReversed, 
            backLeft.TurnmotorReversed,
            backLeft.PIDSTATUS);
        
    private  swervemodule BackRight = new swervemodule(4,
            backRight.DrivePort, 
            backRight.TurnPort, 
            backRight.EncPort, 
            backRight.offset,
            backRight.kP, 
            backRight.kI, 
            backRight.kD, 
            backRight.DrivemotorReversed, 
            backRight.TurnmotorReversed,
            backRight.PIDSTATUS);

    SwerveModulePosition positions[] = {FrontLeft.getPosition(), FrontRight.getPosition(), BackLeft.getPosition(), BackRight.getPosition()};

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), positions, new Pose2d(0, 0, getRotation2d()));
  

    
    
    public swervechasis(){


        publisherModuleStates = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

      publisherModuleStates2 = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveStates2", SwerveModuleState.struct).publish();
    
        publisherRobotHeading = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/RobotRotation", Rotation2d.struct).publish();

       publisherOdometry = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/Odometry", Pose2d.struct).publish();

      new Thread(() -> {
        try{
            Thread.sleep(1000);
            zeroHeading();
        } catch (Exception e){

        }
      }).start();
        
    }

    public void zeroHeading(){
        navX.reset();
    }

    // Creacion del metodo para los tipos de velocidades
    public void setChassisSpeed(double speedX, double speedY, double speedZ){

        // Asignacion de las velocidades ya creadas a los modulos 
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(speedX, speedY, speedZ));
        //SwerveDriveKinematics.desaturateWheelSpeeds(states, 0.5);

        setModuleStates(states);

        
    }


    // Creacion del metodo para las velocidades orientadas al campo 
    public void setFieldOrientedSpeed(double speedX, double speedY, double speedZ) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedZ , getRotation2d());
        
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        // Kinematicas para asingar la velocidad y cantidad de energia necesaria para el swerve 
        setModuleStates(states);
        
    }   
    // Metodo para asignar a cada modulo su estado 
    public void setModuleStates (SwerveModuleState[] state){
        //SwerveDriveKinematics.desaturateWheelSpeeds(state, Constants.metXsec);
        FrontLeft.setDesiredState(state[0]);
        FrontRight.setDesiredState(state[1]);
        BackLeft.setDesiredState(state[2]);
        BackRight.setDesiredState(state[3]);

    }
    // Metodo para adquirir el angulo con la navx 
    public double getAngle(){
        return Math.IEEEremainder(navX.getAngle(), 360);
    }

    // Metodo para que los modulos sigan el angulo mas cercano 
    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(-getAngle());
    }

    //Metodo para obtener la posicion en metros con la odometria 
    public Pose2d getPose2d(){
        return odometry.getPoseMeters();
    }

    public void PoseReset(){
        odometry.resetPosition(getRotation2d(), positions, new Pose2d(0, 0, getRotation2d()));
    }


    @Override
    public void periodic(){
        
        
        //Asignar un numero de posicion a cada modulo 
        positions[0] = FrontLeft.getPosition();
        positions[1] = FrontRight.getPosition();
        positions[2] = BackLeft.getPosition();
        positions[3] = BackRight.getPosition();

        odometry.update(getRotation2d(), positions); 

        publisherModuleStates.set(new SwerveModuleState[] {FrontLeft.gModuleState(), FrontRight.gModuleState(), BackLeft.gModuleState(), BackRight.gModuleState()});

        publisherModuleStates2.set(new SwerveModuleState[] {FrontLeft.gModuleState2(), FrontRight.gModuleState2(), BackLeft.gModuleState2(), BackRight.gModuleState2()});
        
        publisherRobotHeading.set(new Rotation2d[] {odometry.getPoseMeters().getRotation()});

        publisherOdometry.set(new Pose2d[] {odometry.getPoseMeters()});

        


        //Mostrar en la smartdashboard los valores que quieras 
        SmartDashboard.putNumber("NavX", getAngle());
        SmartDashboard.putNumber("NavXRot", getRotation2d().getDegrees());
        SmartDashboard.putNumber("robot X ", getPose2d().getX());
        SmartDashboard.putNumber("robot Y ", getPose2d().getY());

    }
}