package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Constants;

public class Drive extends Subsystem {

    private TalonSRX mRightMaster, mLeftMaster, mRightSlave, mLeftSlave;

    private static Drive mInstance;

    private PeriodicIO mPeriodicIO;
    
    private boolean mIsHighGear;
    private boolean mIsQuickTurn;

    private XboxController mController;

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        mRightMaster = new TalonSRX(Constants.kRightMasterID);
        mLeftMaster = new TalonSRX(Constants.kLeftMasterID);
        mRightSlave = new TalonSRX(Constants.kRightSlaveID);
        mLeftSlave = new TalonSRX(Constants.kLeftSlaveID);

        mRightSlave.set(ControlMode.Follower, Constants.kRightMasterID);
        mLeftSlave.set(ControlMode.Follower, Constants.kLeftMasterID);

        mController = new XboxController(Constants.kXboxControllerPort);

        mIsHighGear = false;
        mIsQuickTurn = false;
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

    @Override
    public void readPeriodicInputs() {
        double throttle = mController.getY(Hand.kLeft);
        double turn = mController.getX(Hand.kRight);
        mIsQuickTurn = mController.getTriggerAxis(Hand.kRight) > 0.5;
        setOpenLoop(throttle, turn);
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

        if (throttle == 0 && !mIsQuickTurn) {
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

    public boolean isHighGear() {
        return mIsHighGear;
    }

    public boolean isQuickTurn() {
        return mIsQuickTurn;
    }
}