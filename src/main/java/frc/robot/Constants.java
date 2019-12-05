package frc.robot;

// import frc.robot.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;
// import com.team254.frc2019.subsystems.ServoMotorSubsystem.TalonSRXConstants;
// import com.team254.frc2019.subsystems.Limelight.LimelightConstants;
// import com.team254.lib.geometry.Pose2d;
// import com.team254.lib.geometry.Rotation2d;
// import com.team254.lib.geometry.Translation2d;

// import java.net.NetworkInterface;
// import java.net.SocketException;
// import java.util.Enumeration;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 */
public class Constants {
    public static final double kLooperDt = 0.01;

    public static final int kRightMasterID = 0;
    public static final int kRightSlaveID = 1;
    public static final int kLeftMasterID = 2;
    public static final int kLeftSlaveID = 3;

    public static final int kXboxControllerPort = 0;

    // wheels
    // Tuned 3/26/19
    public static final double kDriveWheelTrackWidthInches = 25.42;
    public static final double kTrackScrubFactor = 1.0469745223;

}
