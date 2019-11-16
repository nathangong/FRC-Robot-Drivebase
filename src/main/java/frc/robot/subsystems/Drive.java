package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import frc.robot.Constants;
import frc.robot.Kinematics;
import frc.lib.geometry.Twist2d;
import frc.lib.util.DriveSignal;
import frc.lib.util.Util;

public class Drive extends Subsystem {

    private CANSparkMax mRightMaster, mLeftMaster, mRightSlave, mLeftSlave;

    private static Drive mInstance;

    private PeriodicIO mPeriodicIO;
    
    private boolean mIsHighGear;
    private boolean mIsQuickTurn;

    private XboxController mController;

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        mRightMaster = new CANSparkMax(Constants.kRightMasterID, CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeftMaster = new CANSparkMax(Constants.kLeftMasterID, CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightSlave = new CANSparkMax(Constants.kRightSlaveID, CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeftSlave = new CANSparkMax(Constants.kLeftSlaveID, CANSparkMaxLowLevel.MotorType.kBrushless);

        mRightSlave.follow(mRightMaster);
        mLeftSlave.follow(mLeftMaster);

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
        setCheesyishDrive(throttle, turn, false);
    }

    public void writePeriodicOutputs() {
        mLeftMaster.set(mPeriodicIO.left_demand);
        mRightMaster.set(mPeriodicIO.right_demand);
    }

    public boolean checkSystem() {
        return true;
    }
    
    public void setOpenLoop(DriveSignal signal) {
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_demand = signal.getLeft();
    }

    public synchronized void setCheesyishDrive(double throttle, double wheel, boolean quickTurn) {
        if (Util.epsilonEquals(throttle, 0.0, 0.04)) {
            throttle = 0.0;
        }

        if (Util.epsilonEquals(wheel, 0.0, 0.035)) {
            wheel = 0.0;
        }

        final double kWheelGain = 0.05;
        final double kWheelNonlinearity = 0.05;
        final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
        // Apply a sin function that's scaled to make it feel better.
        if (!quickTurn) {
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = wheel / (denominator * denominator) * Math.abs(throttle);
        }

        wheel *= kWheelGain;
        DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
        double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
        setOpenLoop(new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
    }
    
    public void outputTelemetry() {

    }

    public void stop() {
        mRightMaster.set(0);
        mLeftMaster.set(0);
    }

    public boolean isHighGear() {
        return mIsHighGear;
    }

    public boolean isQuickTurn() {
        return mIsQuickTurn;
    }
}