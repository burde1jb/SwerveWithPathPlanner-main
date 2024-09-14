package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewShooterSubsystem;

public class AutonIndexCommand extends Command {
    
    NewShooterSubsystem shooterSubsystem; 
    
    public AutonIndexCommand(NewShooterSubsystem shooterSubsystem)  {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute()   {
        shooterSubsystem.indexAuton(true);
    }

    @Override
    public boolean isFinished()    {
        return true;
    }

}
