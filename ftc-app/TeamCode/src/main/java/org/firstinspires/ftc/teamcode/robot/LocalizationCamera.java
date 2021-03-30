package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.auto.xml.RobotConfigXML;
import org.firstinspires.ftc.teamcode.hardware.motor.CachingMotorEx;
import org.firstinspires.ftc.teamcode.hardware.servo.LCHSServo;
import org.firstinspires.ftc.teamcode.math.Pose;

public class LocalizationCamera {

    private T265Camera slamra;

    public LocalizationCamera(HardwareMap hardwareMap) {

        Transform2d cameraToRobot = new Transform2d();
        double encoderMeasurementCovariance = 1; // Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
        Pose2d startingPose = new Pose2d(0, 0, new Rotation2d());

        slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
        slamra.setPose(startingPose);
    }

    public void start() {
        slamra.start();
    }

    public Pose getPose() {
        Pose2d slamPose = slamra.getLastReceivedCameraUpdate().pose;
        return new Pose(
            slamPose.getTranslation().getX(),
            slamPose.getTranslation().getY(),
            slamPose.getRotation().getDegrees()
        );
    }

    public T265Camera.PoseConfidence getConfidence() {
        return slamra.getLastReceivedCameraUpdate().confidence;
    }

    public Pose getVelocity() {
        return new Pose(
            slamra.getLastReceivedCameraUpdate().velocity.vxMetersPerSecond,
            slamra.getLastReceivedCameraUpdate().velocity.vyMetersPerSecond,
            slamra.getLastReceivedCameraUpdate().velocity.omegaRadiansPerSecond
        );
    }

}
