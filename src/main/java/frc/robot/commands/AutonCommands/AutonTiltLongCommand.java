package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.NewShooterSubsystem;

public class AutonTiltLongCommand extends Command    {
    
    NewShooterSubsystem tiltSubsystem;
    boolean isItFinished;
    boolean tiltFinished;

    public AutonTiltLongCommand(NewShooterSubsystem tiltSubsystem) {
        this.tiltSubsystem = tiltSubsystem;
        addRequirements(tiltSubsystem);
    }

    @Override
    public void initialize()    {
        isItFinished = false;
        tiltFinished = false;
    }

    @Override
    public void execute()   {
        if (!tiltFinished && tiltSubsystem.wentTo(RobotConstants.longshot)) {
            tiltSubsystem.tiltstop();
            tiltFinished = true;
            isItFinished = true;
        }
    }


    @Override
    public boolean isFinished() {
        return isItFinished;
    }

}