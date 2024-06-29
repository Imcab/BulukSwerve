
package frc.robot.Subsystems;

//Librerias 
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class swervemodule extends SubsystemBase{
    
    //Variables 
    CANSparkMax m_drive;
    CANSparkMax m_turn;

    RelativeEncoder enc_drive, enc_turn;

    AnalogInput absoluteEncoder;
    

    PIDController PID;

    int number;
    double offsetEnc;

    double PIDCONTROLLERSTATUS;


    //Creacion del metodo principal 
    public swervemodule(int moduleNumber, int ID_drive, int ID_turn, int portencoder, double offset, double kP, double kI, double kD, boolean DReversed, boolean TReversed, double PIDSTATUS){
     
        // Asignacion de valroes a cada variable
        m_drive = new CANSparkMax(ID_drive, MotorType.kBrushless);
        m_turn = new CANSparkMax(ID_turn, MotorType.kBrushless);

        m_drive.restoreFactoryDefaults();
        m_turn.restoreFactoryDefaults();

        enc_drive = m_drive.getEncoder();
        enc_turn = m_turn.getEncoder();

        enc_drive.setVelocityConversionFactor(Constants.meterspersecond);
        enc_drive.setPositionConversionFactor(Constants.encDrot2met);

        //enc_turn.setVelocityConversionFactor(Constants.degXsec);
        enc_turn.setPositionConversionFactor(Constants.encTrot2deg);

        absoluteEncoder = new AnalogInput(portencoder);

        PID = new PIDController(kP, kI, kD);
        PID.enableContinuousInput(-180, 180);

        m_drive.setInverted(DReversed);
        m_turn.setInverted(TReversed);

        number = moduleNumber;
        offsetEnc = offset;
        PIDCONTROLLERSTATUS = PIDSTATUS;
    }

    // Metodo para obtener la posicion de cada modulo 

    //Metodo para obtener la posicion de al manejar 
    public double getDrivePosition(){
        double position;

        position = enc_drive.getPosition();

        return position;
    }

    //Metodo para definir como van a funcionar los encoders
    public Rotation2d AngleEncoder(){

        double encoderBits = absoluteEncoder.getValue();
        double angleEncoder = (encoderBits * 360) / 4096;

        return Rotation2d.fromDegrees(angleEncoder-offsetEnc);

    }
    public Rotation2d AngleEncoderODOMETRY(){

        double angle = AngleEncoder().getDegrees() + 90;

        return Rotation2d.fromDegrees(angle);

    }

    // Metodo para definir la velocidad y los angulos de los modulos 
    public void setDesiredState(SwerveModuleState desiredState){

        desiredState = SwerveModuleState.optimize(desiredState, AngleEncoder());

        setSpeed(desiredState);
        setAngle(desiredState);

    }

     // Metodo para definir la velociad de los modulos 
    public void setSpeed(SwerveModuleState desiredState){

        m_drive.set(desiredState.speedMetersPerSecond );

    }

    //Metodo para definir el PID con los encoders 
    public void setAngle(SwerveModuleState desiredState){

        double PIDvalue = PID.calculate(AngleEncoder().getDegrees(), desiredState.angle.getDegrees()); 

        //m_turn.set(-PIDvalue);
        m_turn.set(PIDCONTROLLERSTATUS * PIDvalue);

    }

    public double getTurnPosition(){
        double position = enc_turn.getPosition();
        return position;
    }

    public double getDriveVelocity(){
        double vel = enc_drive.getVelocity();
        return vel;
    }

    public SwerveModulePosition getPosition(){

        return new SwerveModulePosition(getDrivePosition(), AngleEncoderODOMETRY());

    }

    public SwerveModuleState gModuleState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(-getTurnPosition()));
    }
    public SwerveModuleState gModuleState2(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(AngleEncoderODOMETRY().getDegrees()));
    }

    @Override
    public void periodic (){

        // Mostrar valores del angulo y la forma de manejo 
        SmartDashboard.putNumber("Angle Encoder "+ number, AngleEncoder().getDegrees());
        SmartDashboard.putNumber("Drive position"+ number, getDrivePosition());
        SmartDashboard.putNumber("Turn pos " + number, getTurnPosition());

    }
 
}

// :v
