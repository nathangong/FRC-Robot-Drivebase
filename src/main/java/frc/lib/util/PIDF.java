package frc.lib.util;

public class PIDF {
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double mSetpoint;
    private double mError;
    
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
}