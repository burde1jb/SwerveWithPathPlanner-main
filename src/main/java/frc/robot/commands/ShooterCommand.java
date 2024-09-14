package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

// public class ShooterCommand extends Command {
//     ShooterSubsystem shooterSubsystem;

//     XboxController controller2;
//     public ShooterCommand(ShooterSubsystem shooterSubsytem, XboxController controller2){

//         this.shooterSubsystem = shooterSubsytem;
//         this.controller2 = controller2;
//         addRequirements(shooterSubsytem);
//     }

    // @Override
//     public void execute()   {
//         if (controller2.getLeftTriggerAxis() > 0.2) {
//             shooterSubsystem.shooterOn();
//         }
//         else    {
//             shooterSubsystem.stop();
//         }

//         if (controller2.getRightTriggerAxis() > 0.2)    {
//             shooterSubsystem.shoot();
//         }
//         else if (controller2.getBackButton())   {
//             shooterSubsystem.unstuck();
//         }
//         else    {
//             shooterSubsystem.resetServo();
//         }
//     }
// }
