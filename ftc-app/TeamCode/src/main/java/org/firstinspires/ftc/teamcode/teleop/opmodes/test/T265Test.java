package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Test")
//@Disabled
public class T265Test extends BaseDrive {

    T265Camera slamra;
    @Override
    protected void initialize() {
        Transform2d cameraToRobot = new Transform2d();
// Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
        double encoderMeasurementCovariance = 0.8;
// Set to the starting pose of the robot
        Pose2d startingPose = new Pose2d(0, 0, new Rotation2d());

        slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
        slamra.setPose(startingPose);
        slamra.start();

    }

    @Override
    protected void update() {
        updateDrive();

        T265Camera.CameraUpdate cameraUpdate = slamra.getLastReceivedCameraUpdate();
        if (cameraUpdate != null) {
            telemetry.addData("pose", cameraUpdate.pose);
            telemetry.addData("confidence", cameraUpdate.confidence.name());
            telemetry.addData("velocity", cameraUpdate.velocity);
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {
        super.onStop();
        slamra.stop();
    }
}
