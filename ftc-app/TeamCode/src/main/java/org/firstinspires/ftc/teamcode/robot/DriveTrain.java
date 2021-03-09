package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.motor.CachingMotorEx;
import org.firstinspires.ftc.teamcode.math.LCHSMath;
import org.firstinspires.ftc.teamcode.math.Pose;

public class DriveTrain {

    public static final int CLICKS_PER_INCH = 45;

    public CachingMotorEx lf;
    public CachingMotorEx rf;
    public CachingMotorEx lb;
    public CachingMotorEx rb;

    private DcMotor.RunMode mode;
    private final double targetVelocity = 2900;

    DriveTrain(HardwareMap hardwareMap) {
        lf = new CachingMotorEx(hardwareMap, "lf");
        rf = new CachingMotorEx(hardwareMap, "rf");
        lb = new CachingMotorEx(hardwareMap, "lb");
        rb = new CachingMotorEx(hardwareMap, "rb");

        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setMode(DcMotor.RunMode mode) {
        this.mode = mode;
        lf.setMode(mode);
        lb.setMode(mode);
        rf.setMode(mode);
        rb.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        lf.setZeroPowerBehavior(behavior);
        rf.setZeroPowerBehavior(behavior);
        lb.setZeroPowerBehavior(behavior);
        rb.setZeroPowerBehavior(behavior);
    }

    public void drive(Pose pose) {
        drive(pose, 1.0);
    }
    public void drive(Pose pose, double powerFactor) {
        // Calculate power for mecanum drive
        double lfp = LCHSMath.clipPower(pose.y + pose.r + pose.x);
        double rfp = LCHSMath.clipPower(pose.y - pose.r - pose.x);
        double lbp = LCHSMath.clipPower(pose.y + pose.r - pose.x);
        double rbp = LCHSMath.clipPower(pose.y - pose.r + pose.x);

        if (mode == DcMotor.RunMode.RUN_USING_ENCODER) {
            lf.setVelocity(lfp * targetVelocity * powerFactor);
            rf.setVelocity(rfp * targetVelocity * powerFactor);
            lb.setVelocity(lbp * targetVelocity * powerFactor);
            rb.setVelocity(rbp * targetVelocity * powerFactor);
        } else {
            lf.setPower(lfp * powerFactor);
            rf.setPower(rfp * powerFactor);
            lb.setPower(lbp * powerFactor);
            rb.setPower(rbp * powerFactor);
        }
    }

    public DcMotor.RunMode getMode() {
        return mode;
    }

    public void stop() {
        drive(new Pose(0, 0, 0));
    }

}
