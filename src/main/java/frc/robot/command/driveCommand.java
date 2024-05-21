package frc.robot.command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.swervechasis;

public class driveCommand extends Command { 
    swervechasis chasis;
    Supplier<Double> speedX, speedY, speedZ;

    public driveCommand (swervechasis chasis, Supplier<Double> speedX, Supplier<Double> speedY, Supplier<Double> speedZ) {
        this.chasis = chasis;
        this.speedX = speedX;
        this.speedY = speedY;
        this.speedZ = speedZ;

        addRequirements(chasis);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){

        double X = speedX.get();
        double Y = speedY.get();
        double Z = speedZ.get();

        if (Math.abs(X) < 0.05) {
            X = 0;
        }
        if (Math.abs(Y) < 0.05) {
            Y = 0;
        }
        if (Math.abs(Z) < 0.05) {
            Z = 0;
        }

        chasis.setFieldOrientedSpeed(X, Y, Z);
    }
    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}