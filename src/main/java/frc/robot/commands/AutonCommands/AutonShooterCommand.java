package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutonShooterCommand extends Command {
    
    NewShooterSubsystem shooterSubsystem;
    Timer timer;
    
    public AutonShooterCommand(NewShooterSubsystem shooterSubsystem)  {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute()   {
        shooterSubsystem.newshooterOn();
    }

    @Override
    public boolean isFinished()    {
        if (timer.get() >=0.5) {
            shooterSubsystem.indexOnMax();
            return false;
        }
        if (timer.get() >=1) {
            shooterSubsystem.newstopshooter();
            shooterSubsystem.indexOff();
            return true;
        }
        return false;
    }

}
