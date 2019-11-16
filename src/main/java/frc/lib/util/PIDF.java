package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;

public class PIDF {
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double mSetpoint;
    private double mError;
    private double mPreviousTime = 0;
    private double mIntegral = 0;
    private double mDerivative;
    private double mPreviousError = 0;
    private int kIZone;

    PIDF(double p, double i, double d, double f, int iZone) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
        kIZone = iZone;
    }
    
    public void setPIDF(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public double getF() {
        return kF;
    }

    public double getError() {
        return mError;
    }

    public double getSetpoint() {
        return mSetpoint;
    }
    
    public void setSetpoint(double setpoint) {
        mSetpoint = setpoint;
    }

    public double update(double position) {
        double error = mSetpoint - position;
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - mPreviousTime;

        mIntegral = mIntegral + error * dt;
        mDerivative = (error - mPreviousError) / dt;

        double output;

        mPreviousTime = currentTime;
        mPreviousError = error;

        if (Math.abs(error) < kIZone) output = kP * error + kI * mIntegral + kD * mDerivative + kF * mSetpoint;
        else output = kP * error + kD * mDerivative + kF * mSetpoint;
        return output;
    }
}