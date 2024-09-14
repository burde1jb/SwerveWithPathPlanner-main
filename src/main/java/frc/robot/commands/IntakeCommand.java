package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class IntakeCommand extends Command {
    
    IntakeSubsystem intakeSubsystem;
    Joystick controller1;
    XboxController controller2;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, XboxController controller2, Joystick controller1)  {
        this.intakeSubsystem = intakeSubsystem;
        this.controller1 = controller1;
        this.controller2 = controller2;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute()   {
        if (controller2.getRightBumper() && !controller2.getStartButton())   {
            intakeSubsystem.intakeOn(true);
        }
        else if (controller2.getLeftY() > .2 || controller2.getLeftY() < -.2)   {
            intakeSubsystem.intakeSlow(false);
        }
        else if (controller2.getBackButton())   {
            intakeSubsystem.intakeOn(false);
        }
        else if (controller2.getStartButton() && controller2.getRightBumper() && intakeSubsystem.sensor.get())  {
            intakeSubsystem.intakeOn(true);
        }
        else    {
          intakeSubsystem.intakeOff();
        }
    }

}