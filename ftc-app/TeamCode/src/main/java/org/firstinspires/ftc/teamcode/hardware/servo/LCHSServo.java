package org.firstinspires.ftc.teamcode.hardware.servo;

import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.firstinspires.ftc.teamcode.auto.xml.RobotConfigXML;
import org.firstinspires.ftc.teamcode.robot.RingShooter;

import java.util.HashMap;
import java.util.Map;

import javax.xml.xpath.XPathExpressionException;

public class LCHSServo implements Servo {
    private final Servo delegate;
    private final String name;
    private double cachedPosition;
    private String state = "";
    private XPathAccess servoConfig;

    public LCHSServo(HardwareMap hardwareMap, String name, RobotConfigXML config) {
        this.name = name;
        this.delegate = hardwareMap.servo.get(name);
        servoConfig = config.getPath(name.toUpperCase());
    }

    public LCHSServo(HardwareMap hardwareMap,  String name) {
        this.name = name;
        this.delegate = hardwareMap.servo.get(name);
    }

    public LCHSServo(Servo delegate) {
        this.delegate = delegate;
        name = delegate.getDeviceName();
    }


    public String getState() {
        return state;
    }

    public void setState(String state) {
        this.state = state;
        if (servoConfig == null) {
            RobotLogCommon.e("LCHSServo:"+name, "Tried to set servo state but config was not set");
            return;
        }
        try {
            double position = servoConfig.getDouble(state);
            setPosition(position);
        } catch (XPathExpressionException e) {
            RobotLogCommon.e("LCHSServo:"+name, "Could not find servo state {"+state+"} for servo "+name);
        }
    }


    public String getName() {
        return name;
    }

    @Override
    public ServoController getController() {
        return delegate.getController();
    }

    @Override
    public int getPortNumber() {
        return delegate.getPortNumber();
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
    public void setPosition(double position) {
        if (position != cachedPosition) {
            delegate.setPosition(position);
            cachedPosition = position;
        }
    }

    @Override
    public double getPosition() {
        return delegate.getPosition();
    }

    @Override
    public void scaleRange(double min, double max) {
        delegate.scaleRange(min, max);
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
}
