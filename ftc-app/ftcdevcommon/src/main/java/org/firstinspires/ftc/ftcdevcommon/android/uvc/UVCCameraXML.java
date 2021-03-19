package org.firstinspires.ftc.ftcdevcommon.android.uvc;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.w3c.dom.Element;

import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathExpressionException;

// Encapsulate a single command from the robot configuration file.
public class UVCCameraXML {

    private final static String TAG = "UVCCameraXML";
    private final XPathAccess xPathAccess;

    public UVCCameraXML(XPath pXpath, Element pCommandElement, String pCommandId) {
        xPathAccess = new XPathAccess(pXpath, pCommandElement, pCommandId);
    }

    // Collect information about a single camera from the children of each camera's element node.
    public RobotCameraUVC getOneCameraElement() {

        try {
            String product_name = xPathAccess.getString("product_name", true);
            int vendor_id = xPathAccess.getInt("vendor_id");
            int product_id = xPathAccess.getInt("product_id");
            String serial_number = xPathAccess.getString("serial_number", true);
            int width = xPathAccess.getInt("width");
            int height = xPathAccess.getInt("height");

            return new RobotCameraUVC(product_name, vendor_id, product_id, serial_number, width, height);
        } catch (XPathExpressionException xex) {
            throw new AutonomousRobotException(TAG, "Xpath exception in accessing UVC camera data");
        }
    }

}
