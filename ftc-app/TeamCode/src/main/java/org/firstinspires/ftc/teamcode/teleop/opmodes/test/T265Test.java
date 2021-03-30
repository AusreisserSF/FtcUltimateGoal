package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

public class T265Test extends TeleOpBase {

    T265Camera slamra;
    @Override
    protected void initialize() {
        Transform2d cameraToRobot = new Transform2d();
// Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
        double encoderMeasurementCovariance = 0.8;
// Set to the starting pose of the robot
        Pose2d startingPose = new Pose2d(1, 1, new Rotation2d());

        slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
        slamra.setPose(startingPose);
        slamra.start();
    }

    @Override
    protected void update() {
        T265Camera.CameraUpdate cameraUpdate = slamra.getLastReceivedCameraUpdate();
        telemetry.addData("pose", cameraUpdate.pose);
        telemetry.addData("confidence", cameraUpdate.confidence.name());
        telemetry.addData("velocity", cameraUpdate.velocity);
        telemetry.update();
    }
}
