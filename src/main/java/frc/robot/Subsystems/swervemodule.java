package frc.robot.Subsystems;

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
    
    CANSparkMax m_drive;
    CANSparkMax m_turn;

    AnalogInput absoluteEncoder;
    PIDController PID;

    int number;
    double offsetEnc;

    public swervemodule(int moduleNumber, int ID_drive, int ID_turn, int portencoder, double offset, double kP, double kI, double kD, boolean DReversed, boolean TReversed){
     
        m_drive = new CANSparkMax(ID_drive, MotorType.kBrushless);
        m_turn = new CANSparkMax(ID_turn, MotorType.kBrushless);

        absoluteEncoder = new AnalogInput(portencoder);

        PID = new PIDController(kP, kI, kD);

        
        m_drive.setInverted(DReversed);
        m_turn.setInverted(TReversed);

        number = moduleNumber;
        offsetEnc = offset;


    }

    public SwerveModulePosition getPosition(){

        return new SwerveModulePosition(getDrivePosition(), AngleEncoder());

    }

    public double getDrivePosition(){
        double position;

        position = m_drive.getEncoder().getPosition();

        return position*Constants.meterspersecond;
    }

    public Rotation2d AngleEncoder(){

        double encoderBits = absoluteEncoder.getValue();
        double angleEncoder = (encoderBits * 360) / 4096;

        return Rotation2d.fromDegrees(angleEncoder-offsetEnc);

    }

    public void setSpeed(SwerveModuleState desiredState){

        m_drive.set(desiredState.speedMetersPerSecond);

    }

    public void setAngle(SwerveModuleState desiredState){

        double PIDvalue = PID.calculate(AngleEncoder().getDegrees(), desiredState.angle.getDegrees());

        m_turn.set(PIDvalue);

    }

    public void setDesiredState(SwerveModuleState desiredState){

        desiredState = SwerveModuleState.optimize(desiredState, AngleEncoder());

        setSpeed(desiredState);
        setAngle(desiredState);

    }

    @Override
    public void periodic (){

        SmartDashboard.putNumber("Angle Encoder "+ number, AngleEncoder().getDegrees());
        SmartDashboard.putNumber("Drive position"+ number, getDrivePosition());
    }
 
}

// :v
