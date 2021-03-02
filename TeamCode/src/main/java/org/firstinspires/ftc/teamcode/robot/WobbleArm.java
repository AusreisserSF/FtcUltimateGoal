package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.motor.CachingMotorEx;
import org.firstinspires.ftc.teamcode.hardware.servo.CachingServo;

public class WobbleArm {

    public CachingMotorEx flipMotor;
    public CachingServo servo;

    WobbleArm(HardwareMap hardwareMap) {
        flipMotor = new CachingMotorEx(hardwareMap, "wobble motor");
        servo = new CachingServo(hardwareMap, "wobble servo");
    }
}
