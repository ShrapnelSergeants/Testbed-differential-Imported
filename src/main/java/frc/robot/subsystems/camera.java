package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class camera extends SubsystemBase {
    public camera() {
        CameraServer.startAutomaticCapture();
    }
}
