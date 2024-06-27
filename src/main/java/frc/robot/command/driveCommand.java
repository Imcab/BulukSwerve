package frc.robot.command;

// Librerias 
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.swervechasis;

// Creacion e iniciacion del comando
public class driveCommand extends Command { 
    swervechasis chasis;
    Supplier<Double> speedX, speedY, speedZ;
    SlewRateLimiter xLimiter, yLimiter, turnLimiter;

    // Creacion del metodo y asignacion de variables a cada tipo de velociddad 
    public driveCommand (swervechasis chasis, Supplier<Double> speedX, Supplier<Double> speedY, Supplier<Double> speedZ) {
        addRequirements(chasis);

        this.chasis = chasis;
        this.speedX = speedX;
        this.speedY = speedY;
        this.speedZ = speedZ;

        this.xLimiter = new SlewRateLimiter(Constants.meterspersecond);
        this.yLimiter = new SlewRateLimiter(Constants.meterspersecond);
        this.turnLimiter = new SlewRateLimiter(Constants.angvel);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        
        // Asignacion de como cada variable va a adquirir su velocidad segun el joystick 
        double Xvel = -speedX.get();
        double Yvel = speedY.get();
        double Zvel = speedZ.get();



        if (Math.abs(Xvel) < 0.05) {
            Xvel = 0;
        }
        if (Math.abs(Yvel) < 0.05) {
            Yvel = 0;
        }
        if (Math.abs(Zvel) < 0.05) {
            Zvel = 0;
        }

       /* Xvel = xLimiter.calculate(Xvel) * Constants.meterspersecond;
        Yvel = xLimiter.calculate(Yvel) * Constants.meterspersecond;
        Zvel = xLimiter.calculate(Zvel) * Constants.meterspersecond; */

        chasis.setFieldOrientedSpeed(Xvel, Yvel, Zvel * 0.01);
        //chasis.setChassisSpeed(X, Y, Z);
    }
    @Override
    public boolean isFinished(){
        return false;
    }

}
