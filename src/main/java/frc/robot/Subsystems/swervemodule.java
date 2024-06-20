
package frc.robot.Subsystems;

//Librerias 
import com.revrobotics.CANSparkMax;
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

    AnalogInput absoluteEncoder;
    PIDController PID;

    int number;
    double offsetEnc;

    //Creacion del metodo principal 
    public swervemodule(int moduleNumber, int ID_drive, int ID_turn, int portencoder, double offset, double kP, double kI, double kD, boolean DReversed, boolean TReversed){
     
        // Asignacion de valroes a cada variable
        m_drive = new CANSparkMax(ID_drive, MotorType.kBrushless);
        m_turn = new CANSparkMax(ID_turn, MotorType.kBrushless);

        absoluteEncoder = new AnalogInput(portencoder);

        PID = new PIDController(kP, kI, kD);

        
        m_drive.setInverted(DReversed);
        m_turn.setInverted(TReversed);

        number = moduleNumber;
        offsetEnc = offset;


    }

    // Metodo para obtener la posicion de cada modulo 
    

    public SwerveModuleState gModuleState(){
        return new SwerveModuleState(getDriveVel(), new Rotation2d(getTurningPosition()));
    }

    //Metodo para obtener la posicion de al manejar 
    public double getDrivePosition(){
        double position;

        position = m_drive.getEncoder().getPosition();

        return position*Constants.meterspersecond;
    }
    public double getTurningPosition(){
        double turn = m_turn.getEncoder().getPosition();
        return turn;
    }

    public double getDriveVel(){
        double vel = m_drive.getEncoder().getVelocity();
        

        return vel;
    }

    //Metodo para definir como van a funcionar los encoders
    public Rotation2d AngleEncoder(){

        double encoderBits = absoluteEncoder.getValue();
        double angleEncoder = (encoderBits * 360) / 4096;

        return Rotation2d.fromDegrees(angleEncoder-offsetEnc);

    }

    // Metodo para definir la velociad de los modulos 
    public void setSpeed(SwerveModuleState desiredState){

        m_drive.set(desiredState.speedMetersPerSecond);

    }

    //Metodo para definir el PID con los encoders 
    public void setAngle(SwerveModuleState desiredState){

        double PIDvalue = PID.calculate(AngleEncoder().getDegrees(), desiredState.angle.getDegrees());

        m_turn.set(-PIDvalue);

    }

    // Metodo para definir la velocidad y los angulos de los modulos 
    public void setDesiredState(SwerveModuleState desiredState){

        desiredState = SwerveModuleState.optimize(desiredState, AngleEncoder());

        setSpeed(desiredState);
        setAngle(desiredState);

    }

    public SwerveModulePosition getPosition(){

        return new SwerveModulePosition(getDrivePosition(), AngleEncoder());

    }

    @Override
    public void periodic (){

        // Mostrar valores del angulo y la forma de manejo 
        SmartDashboard.putNumber("Angle Encoder "+ number, AngleEncoder().getDegrees());
        SmartDashboard.putNumber("Drive position"+ number, getDrivePosition());
    }
 
}

// :v
