package org.firstinspires.ftc.teamcode.auto.xml;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.math.Angle;

import java.util.List;

import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathExpressionException;
import javax.xml.xpath.XPathFactory;

//!! This is essentially a wrapper around XPathAccess.
// I would have edited XPathAccess directly but it is a .class file
// since it is from ftcdevcommon.
public class AutoCommandXML {

    public static final String TAG = "AutoCommandXML";

    // these declarations are in FTCAuto but used only once
    private final XPathFactory xPathFactory = XPathFactory.newInstance();
    private final XPath xPath = xPathFactory.newXPath();
    private final XPathAccess xPathAccess;

    private final String name;

    // only one parameter
    public AutoCommandXML(RobotActionXML.CommandXML commandXML) {
        this.name = commandXML.getCommandId().toUpperCase();
        xPathAccess = new XPathAccess(xPath, commandXML.getCommandElement(), commandXML.getCommandId());
    }

    // accessor functions
    public String getName() {
        return name;
    }

    public double getDouble(String key) throws XPathExpressionException {
        return xPathAccess.getDouble(key);
    }

    public int getInt(String key) throws XPathExpressionException {
        return xPathAccess.getInt(key);
    }

    //!! i wish to add getAngle and getPID to XPathAccess
    public Angle getAngle(String key) throws XPathExpressionException {
        double degrees = xPathAccess.getDouble(key);
        return new Angle(degrees, AngleUnit.DEGREES);
    }

    public PIDCoefficients getPID(String key) throws XPathExpressionException {
        if (key.length() < 1) key += " ";
        double kp = xPathAccess.getDouble(key + "kp");
        double ki = xPathAccess.getDouble(key + "ki");
        double kd = xPathAccess.getDouble(key + "kd");
        return new PIDCoefficients(kp, ki, kd);
    }

    public PIDCoefficients getPID() throws XPathExpressionException {
        return getPID("");
    }

    public String getStringInRange(String path, List<String> rangeList) throws XPathExpressionException {
        return xPathAccess.getStringInRange(path, rangeList);
    }

    public List<String> validRange(String... rangeValues) {
        return xPathAccess.validRange(rangeValues);
    }



    // and so on..


}
