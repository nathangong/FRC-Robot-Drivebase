package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class Drive extends Subsystem {

    private TalonSRX mRightMaster, mLeftMaster, mRightSlave, mLeftSlave;

    private static Drive mInstance = null;

    private PeriodicIO mPeriodicIO;

    public Drive() {
        mPeriodicIO = new PeriodicIO();

        mRightMaster = new TalonSRX(Constants.kRightMasterID);
        mLeftMaster = new TalonSRX(Constants.kLeftMasterID);
        mRightSlave = new TalonSRX(Constants.kRightSlaveID);
        mLeftSlave = new TalonSRX(Constants.kLeftSlaveID);

        mRightSlave.set(ControlMode.Follower, Constants.kRightMasterID);
        mLeftSlave.set(ControlMode.Follower, Constants.kLeftMasterID);

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

        if (throttle == 0) {
            mPeriodicIO.right_demand = 0;
            mPeriodicIO.left_demand = 0;
        }

        if (Math.abs(mPeriodicIO.right_demand) > 1.0 || Math.abs(mPeriodicIO.left_demand) > 1.0) {
            mPeriodicIO.right_demand /= Math.max(Math.abs(mPeriodicIO.right_demand), Math.abs(mPeriodicIO.left_demand));
            mPeriodicIO.left_demand /= Math.max(Math.abs(mPeriodicIO.right_demand), Math.abs(mPeriodicIO.left_demand));
        }
    }
    
    public void outputTelemetry() {

    }

    public void stop() {
        mRightMaster.setNeutralMode(NeutralMode.Brake);
        mLeftMaster.setNeutralMode(NeutralMode.Brake);
    }
}