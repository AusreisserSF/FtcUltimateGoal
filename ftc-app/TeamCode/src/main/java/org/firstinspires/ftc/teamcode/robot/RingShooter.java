package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.motor.CachingMotorEx;
import org.firstinspires.ftc.teamcode.hardware.servo.CachingServo;

public class RingShooter {

    public CachingMotorEx intakeMotor;
    public CachingMotorEx liftMotor;
    public CachingMotorEx shootMotor;

    RingShooter(HardwareMap hardwareMap) {
        intakeMotor = new CachingMotorEx(hardwareMap, "intake");
        liftMotor = new CachingMotorEx(hardwareMap, "lift");
        shootMotor = new CachingMotorEx(hardwareMap, "shoot");
    }

    public void shootPowerShot() {
        // TODO: implement shoot target
    }

}
