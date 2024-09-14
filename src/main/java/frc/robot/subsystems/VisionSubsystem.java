package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase{
    private final double limelightMountAngle = 0;   // In radians
    private final double limelightMountHeight = 0;  // In 
    public double x;
    public double y;
    public NetworkTableEntry tx;
    public NetworkTableEntry ty;

    public VisionSubsystem() {
    }

    public void publishToDashboard() {
        // read values periodically
        SmartDashboard.putNumber("tx", x);
        SmartDashboard.putNumber("ty", y);
    }

    public void update() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        publishToDashboard();
    }

    public double distanceFromTarget(double targetHeight) {
        return (targetHeight - limelightMountHeight) / Math.tan(y + limelightMountAngle);
    }

    
}
   