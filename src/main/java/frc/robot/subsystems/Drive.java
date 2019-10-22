package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Drive extends Subsystem {
    private final int rightMasterID = 0;
    private final int leftMasterID = 1;
    private final int rightSlaveID = 2;
    private final int leftSlaveID = 3;

    private TalonSRX mRightMaster, mLeftMaster, mRightSlave, mLeftSlave;

    private static Drive mInstance = null;

    private PeriodicIO mPeriodicIO;

    public Drive() {
        mPeriodicIO = new PeriodicIO();

        mRightMaster = new TalonSRX(rightMasterID);
        mLeftMaster = new TalonSRX(leftMasterID);
        mRightSlave = new TalonSRX(rightSlaveID);
        mLeftSlave = new TalonSRX(leftSlaveID);

        mRightSlave.set(ControlMode.Follower, rightMasterID);
        mLeftSlave.set(ControlMode.Follower, leftMasterID);

        mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 1000);
        mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 1000);
    }

    public static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }
        return mInstance;
    }

    public static class PeriodicIO {
        double right_demand;
        double left_demand;
    }

    public void writePeriodicOutputs() {
        mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
        mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
    }

    public boolean checkSystem() {
        return true;
    }
    
    public void setOpenLoop(double throttle, double turn) {
        mPeriodicIO.right_demand = throttle + turn;
        mPeriodicIO.left_demand = throttle - turn;
    }
    
    public void outputTelemetry() {

    }

    public void stop() {
        mRightMaster.setNeutralMode(NeutralMode.Brake);
        mLeftMaster.setNeutralMode(NeutralMode.Brake);
    }
}