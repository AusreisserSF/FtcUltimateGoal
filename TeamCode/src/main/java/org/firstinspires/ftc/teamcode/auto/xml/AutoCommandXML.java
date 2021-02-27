package org.firstinspires.ftc.teamcode.auto.xml;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.PIDController;

import javax.xml.xpath.XPathExpressionException;

// XPathAccess supports access to primitive types (int, double, String)
// as they appear in XML elements and attributes.
// The static methods in this class use XPathAccess but return a higher-level
// structure.
public class AutoCommandXML {

    public static PIDController getPIDController(XPathAccess pCommandXPath, double pDegrees) throws XPathExpressionException {
        PIDCoefficients pidCoefficients = new PIDCoefficients(
                pCommandXPath.getDouble("pid/p", 0.0),
                pCommandXPath.getDouble("pid/i", 0.0),
                pCommandXPath.getDouble("pid/d", 0.0));
        return new PIDController(pidCoefficients, pDegrees);
    }

    public static Angle getAngle(XPathAccess pCommandXPath, String pElement) throws XPathExpressionException {
        double degrees = pCommandXPath.getDouble(pElement);
        return new Angle(degrees, AngleUnit.DEGREES);
    }
}
