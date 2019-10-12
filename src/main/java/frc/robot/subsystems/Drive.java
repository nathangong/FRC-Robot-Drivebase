package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Drive {
    private final int rightMasterID = 0;
    private final int leftMasterID = 1;
    private final int rightSlaveID = 2;
    private final int leftSlaveID = 3;

    private TalonSRX rightMaster, leftMaster, rightSlave, leftSlave;

    public Drive() {
        rightMaster = new TalonSRX(rightMasterID);
        leftMaster = new TalonSRX(leftMasterID);
        rightSlave = new TalonSRX(rightSlaveID);
        leftSlave = new TalonSRX(leftSlaveID);

        rightSlave.set(ControlMode.Follower, rightMasterID);
        leftSlave.set(ControlMode.Follower, leftMasterID);

        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 1000);
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 1000);
    }
    
    public void setOpenLoop(double throttle, double turn) {
        rightMaster.set(ControlMode.PercentOutput, throttle + turn);
        leftMaster.set(ControlMode.PercentOutput, throttle - turn);
    }

    public void stop() {
        rightMaster.setNeutralMode(NeutralMode.Brake);
        leftMaster.setNeutralMode(NeutralMode.Brake);
    }
}