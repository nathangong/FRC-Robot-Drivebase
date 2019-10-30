package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;
import frc.lib.util.CheesyDriveHelper;
import frc.lib.util.DriveSignal;

public class Drive extends Subsystem {

    private TalonSRX mRightMaster, mLeftMaster, mRightSlave, mLeftSlave;

    private static Drive mInstance;

    private PeriodicIO mPeriodicIO;
    
    private CheesyDriveHelper mCheesyDriveHelper;
    private boolean mIsHighGear;
    private boolean mIsQuickTurn;

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        mRightMaster = new TalonSRX(Constants.kRightMasterID);
        mLeftMaster = new TalonSRX(Constants.kLeftMasterID);
        mRightSlave = new TalonSRX(Constants.kRightSlaveID);
        mLeftSlave = new TalonSRX(Constants.kLeftSlaveID);

        mRightSlave.set(ControlMode.Follower, Constants.kRightMasterID);
        mLeftSlave.set(ControlMode.Follower, Constants.kLeftMasterID);

        mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 1000);
        mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 1000);

        mCheesyDriveHelper = new CheesyDriveHelper();
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

    public void writePeriodicOutputs() {
        mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
        mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
    }

    public boolean checkSystem() {
        return true;
    }
    
    public void setOpenLoop(double throttle, double turn) {
        DriveSignal driveSignal = mCheesyDriveHelper.cheesyDrive(throttle, turn, mIsQuickTurn, mIsHighGear);
        mPeriodicIO.right_demand = driveSignal.getRight();
        mPeriodicIO.left_demand = driveSignal.getLeft();
    }
    
    public void outputTelemetry() {

    }

    public void stop() {
        mRightMaster.setNeutralMode(NeutralMode.Brake);
        mLeftMaster.setNeutralMode(NeutralMode.Brake);
    }

    public boolean isHighGear() {
        return mIsHighGear;
    }

    public boolean isQuickTurn() {
        return mIsQuickTurn;
    }
}