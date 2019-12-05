package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Constants;

public class Drive extends Subsystem {

    private TalonSRX mRightMaster, mLeftMaster, mRightSlave, mLeftSlave;

    private static Drive mInstance;

    private PeriodicIO mPeriodicIO;

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        mRightMaster = new TalonSRX(Constants.kRightMasterID);
        mLeftMaster = new TalonSRX(Constants.kLeftMasterID);
        mRightSlave = new TalonSRX(Constants.kRightSlaveID);
        mLeftSlave = new TalonSRX(Constants.kLeftSlaveID);

        mRightMaster.setNeutralMode(NeutralMode.Coast);
        mLeftMaster.setNeutralMode(NeutralMode.Coast);

        mRightSlave.set(ControlMode.Follower, Constants.kRightMasterID);
        mLeftSlave.set(ControlMode.Follower, Constants.kLeftMasterID);
    }

    public synchronized static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }
        return mInstance;
    }

    public static class PeriodicIO {
        double right_demand;
        double left_demand;
    }

    @Override
    public void readPeriodicInputs() {
        
    }

    @Override
    public void writePeriodicOutputs() {
        mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
        mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
    }

    public boolean checkSystem() {
        return true;
    }

    
    public synchronized void setOpenLoop(double throttle, double turn, boolean isQuickTurn) {
        mPeriodicIO.right_demand = throttle + turn;
        mPeriodicIO.left_demand = throttle - turn;

        if (throttle == 0 && !isQuickTurn) {
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
        mLeftMaster.set(ControlMode.PercentOutput, 0);
        mRightMaster.set(ControlMode.PercentOutput, 0);
    }
}