package org.firstinspires.ftc.teamcode.robot.utils;

import com.qualcomm.robotcore.util.RobotLog;

public class SomethingBadHappened extends InterruptedException {

    public SomethingBadHappened(String message) {
        RobotLog.dd("SomethingBadHappened", message);
        RobotLog.ee("SomethingBadHappened", message);
    }

}
