package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutonIntakeOnCommand extends Command   {
    
    IntakeSubsystem intakeSubsystem;

    public AutonIntakeOnCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }
    
    @Override
    public void execute()   {
        intakeSubsystem.intakeOn(true);
    }

}
