package org.firstinspires.ftc.teamcode.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class LCHSMotor implements DcMotorEx {
    private final DcMotorEx delegate;
    private double cachedPower;
    private double cachedVelocity;

    public LCHSMotor(HardwareMap hardwareMap, String name) {
        this(hardwareMap.dcMotor.get(name));
    }

    public LCHSMotor(DcMotor delegate) {
        this.delegate = (DcMotorEx) delegate;
    }

    public void checkAndSetMode(RunMode mode) {
        if (getMode() != mode) {
            setMode(mode);
        }
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return delegate.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        delegate.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return delegate.getController();
    }

    @Override
    public int getPortNumber() {
        return delegate.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        delegate.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return delegate.getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {
        delegate.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return delegate.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        delegate.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return delegate.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return delegate.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return delegate.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        delegate.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return delegate.getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        delegate.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return delegate.getDirection();
    }

    @Override
    public synchronized void setPower(double power) {
        if (power != cachedPower) {
            delegate.setPower(power);
            cachedPower = power;
        }
    }

    @Override
    public double getPower() {
        return delegate.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return delegate.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return delegate.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return delegate.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return delegate.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        delegate.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        delegate.close();
    }

    @Override
    public void setMotorEnable() {
        delegate.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        delegate.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return delegate.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate) {
        if (angularRate != cachedVelocity) {
            delegate.setVelocity(angularRate);
            cachedVelocity = angularRate;
        }

    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        if (angularRate != cachedVelocity) {
            delegate.setVelocity(angularRate, unit);
            cachedVelocity = angularRate;
        }
    }

    @Override
    public double getVelocity() {
        return delegate.getVelocity();
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return delegate.getVelocity(unit);
    }

    @Override
    @Deprecated
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        delegate.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    @Deprecated
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return delegate.getPIDCoefficients(mode);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        delegate.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        delegate.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        delegate.setPositionPIDFCoefficients(p);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return delegate.getPIDFCoefficients(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        delegate.setTargetPositionTolerance(tolerance);
    }

    @Override
    public int getTargetPositionTolerance() {
        return delegate.getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return delegate.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return delegate.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        delegate.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return delegate.isOverCurrent();
    }


}
