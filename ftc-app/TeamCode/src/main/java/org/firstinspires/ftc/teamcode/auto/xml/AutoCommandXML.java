package org.firstinspires.ftc.teamcode.auto.xml;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;

import javax.xml.xpath.XPathExpressionException;

// XPathAccess supports access to primitive types (int, double, String)
// as they appear in XML elements and attributes.
// The static methods in this class use XPathAccess but return a higher-level
// structure.
public class AutoCommandXML {

    public static Pose getPose(XPathAccess command) throws XPathExpressionException {
        return getPose(command, "");
    }

    public static Pose getPose(XPathAccess command, String prefix) throws XPathExpressionException {
        String path = prefix.length() > 0 ? prefix + "_pose" : "pose";
        return new Pose(
                command.getDouble(path+"/x", 0.0),
                command.getDouble(path+"/y", 0.0),
                command.getDouble(path+"/r", 0.0) );
    }

    public static PIDController getPIDController(XPathAccess command, double target) throws XPathExpressionException {
        return getPIDController(command, target, "");
    }

    public static PIDController getPIDController(XPathAccess command, double target, String prefix) throws XPathExpressionException {
        String path = prefix.length() > 0 ? prefix + "_pid" : "pid";
        PIDCoefficients pidCoefficients = new PIDCoefficients(
                command.getDouble(path+"/p", 0.0),
                command.getDouble(path+"/i", 0.0),
                command.getDouble(path+"/d", 0.0) );
        return new PIDController(pidCoefficients, target);
    }

    public static Angle getAngle(XPathAccess pCommandXPath, String pElement) throws XPathExpressionException {
        double degrees = pCommandXPath.getDouble(pElement);
        return new Angle(degrees, AngleUnit.DEGREES);
    }

    public static Angle getAngle(XPathAccess pCommandXPath, String pElement, Angle defaultAngle) throws XPathExpressionException {
        double degrees = pCommandXPath.getDouble(pElement, defaultAngle.getDegrees());
        return new Angle(degrees, AngleUnit.DEGREES);
    }
}
