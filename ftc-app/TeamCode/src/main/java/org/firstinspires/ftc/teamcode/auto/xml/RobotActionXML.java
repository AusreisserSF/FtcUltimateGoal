package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.RobotXMLElement;
import org.firstinspires.ftc.teamcode.auto.vision.VumarkReader;
import org.opencv.core.Rect;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathConstants;
import javax.xml.xpath.XPathExpressionException;
import javax.xml.xpath.XPathFactory;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;

public class RobotActionXML {

    public static final String TAG = "RobotActionXML";
    private static final String FILE_NAME = "RobotAction.xml";

    private final Document document;
    private final XPath xpath;

    /*
    // IntelliJ only
    private static final String JAXP_SCHEMA_LANGUAGE =
            "http://java.sun.com/xml/jaxp/properties/schemaLanguage";
    private static final String W3C_XML_SCHEMA =
            "http://www.w3.org/2001/XMLSchema";
     */

    public RobotActionXML(String pWorkingDirectory) throws ParserConfigurationException, SAXException, IOException {

/*
// IntelliJ only
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        dbFactory.setNamespaceAware(true);
        dbFactory.setValidating(true);
        dbFactory.setAttribute(JAXP_SCHEMA_LANGUAGE, W3C_XML_SCHEMA);

        //## ONLY works with a validating parser.
        dbFactory.setIgnoringElementContentWhitespace(true);
// End IntelliJ only
*/

// Android only
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        // ONLY works with a validating parser (DTD or schema) dbFactory.setIgnoringElementContentWhitespace(true);
        //PY 8/17/2019 Android throws UnsupportedOperationException dbFactory.setXIncludeAware(true);
// End Android only

        DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
        String actionFilename = pWorkingDirectory + FILE_NAME;
        document = dBuilder.parse(new File(actionFilename));

        XPathFactory xpathFactory = XPathFactory.newInstance();
        xpath = xpathFactory.newXPath();
    }

    // Find the requested opMode in the RobotAction.xml file.
    // Package and return all data associated with the OpMode.
    public RobotActionData getOpModeData(String pOpMode) throws XPathExpressionException {

        Level lowestLoggingLevel = null; // null means use the default lowest logging level
        Rect imageROI = new Rect(0, 0, 0, 0); // default is an empty ROI; use the values from RingParameters.xml.
        List<VumarkReader.SupportedVumark> vumarksOfInterest = new ArrayList<>();
        List<RobotXMLElement> actions = new ArrayList<>();

        // Use XPath to locate the desired OpMode.
        String opModePath = "/RobotAction/OpMode[@id=" + "'" + pOpMode + "']";
        Node opModeNode = (Node) xpath.evaluate(opModePath, document, XPathConstants.NODE);
        if (opModeNode == null)
            throw new AutonomousRobotException(TAG, "Missing OpMode " + pOpMode);

        RobotLogCommon.d(TAG, "Extracting data from RobotAction.xml for OpMode " + pOpMode);

        // The next element in the XML is required: <parameters>
        Node parametersNode = getNextElement(opModeNode.getFirstChild());
        if ((parametersNode == null) || !parametersNode.getNodeName().equals("parameters"))
            throw new AutonomousRobotException(TAG, "Missing required <parameters> element");

        // The three possible elements under <parameters> are:
        //   <lowest_logging_level>
        //   <image_roi>
        //   <vumarks>
        // All are optional.

        // A missing or empty optional lowest_logging_level will eventually return null, which
        // means to use the logger's default.
        Node nextParameterNode = getNextElement(parametersNode.getFirstChild());
        if ((nextParameterNode != null) && (nextParameterNode.getNodeName().equals("lowest_logging_level"))) {
            String lowestLoggingLevelString = nextParameterNode.getTextContent().trim();
            if (!lowestLoggingLevelString.isEmpty()) {
                switch (lowestLoggingLevelString) {
                    case "d": {
                        lowestLoggingLevel = Level.FINE;
                        break;
                    }
                    case "v": {
                        lowestLoggingLevel = Level.FINER;
                        break;
                    }
                    case "vv": {
                        lowestLoggingLevel = Level.FINEST;
                        break;
                    }
                    default: {
                        throw new AutonomousRobotException(TAG, "Invalid lowest logging level");
                    }
                }
            }
            nextParameterNode = getNextElement(nextParameterNode.getNextSibling());
        }

        // The next optional element in the XML is <image_roi>.
        if ((nextParameterNode != null) && nextParameterNode.getNodeName().equals("image_roi")) {
            imageROI = ImageXMLCommon.parseROI(nextParameterNode);
            nextParameterNode = getNextElement(nextParameterNode.getNextSibling());
        }

        // The next optional element in the XML is <vumarks>.
        if ((nextParameterNode != null) && nextParameterNode.getNodeName().equals("vumarks")) {
            NodeList vumarkChildren = nextParameterNode.getChildNodes();
            Node oneVumarkNode;
            for (int i = 0; i < vumarkChildren.getLength(); i++) {
                oneVumarkNode = vumarkChildren.item(i);

                if (oneVumarkNode.getNodeType() != Node.ELEMENT_NODE)
                    continue;

                VumarkReader.SupportedVumark oneVumark = VumarkReader.SupportedVumark.valueOf(oneVumarkNode.getNodeName());
                vumarksOfInterest.add(oneVumark);
            }
        }

        // Make sure there are no extraneous elements.
        if (nextParameterNode != null) {
            String nodeName = nextParameterNode.getNodeName();
            if (!(nodeName.equals("lowest_logging_level") ||
                    nodeName.equals("image_roi") ||
                    nodeName.equals("vumarks")))
                throw new AutonomousRobotException(TAG, "Unrecognized element under <parameters> " + nodeName);
        }

        // Now proceed to the <actions> element of the selected OpMode.
        String actionsPath = opModePath + "/actions";
        Node actionsNode = (Node) xpath.evaluate(actionsPath, document, XPathConstants.NODE);
        if (actionsNode == null)
            throw new AutonomousRobotException(TAG, "Missing <actions> element");

        // Now iterate through the children of the <actions> element of the selected OpMode.
        NodeList actionChildren = actionsNode.getChildNodes();
        Node actionNode;

        RobotXMLElement actionXMLElement;
        for (int i = 0; i < actionChildren.getLength(); i++) {
            actionNode = actionChildren.item(i);

            if (actionNode.getNodeType() != Node.ELEMENT_NODE)
                continue;

            actionXMLElement = new RobotXMLElement((Element) actionNode);
            actions.add(actionXMLElement);
        }

        return new RobotActionData(lowestLoggingLevel, imageROI, vumarksOfInterest, actions);
    }

    private Node getNextElement(Node pNode) {
        Node nd = pNode;
        while (nd != null) {
            if (nd.getNodeType() == Node.ELEMENT_NODE) {
                return nd;
            }
            nd = nd.getNextSibling();
        }
        return null;
    }

    public static class RobotActionData {
        public final Level lowestLoggingLevel;
        public final Rect imageROI;
        public final List<VumarkReader.SupportedVumark> vumarksOfInterest;
        public final List<RobotXMLElement> actions;

        public RobotActionData(Level pLevel, Rect pROI,
                               List<VumarkReader.SupportedVumark> pVumarks,
                               List<RobotXMLElement> pActions) {
            lowestLoggingLevel = pLevel;
            imageROI = pROI;
            vumarksOfInterest = pVumarks;
            actions = pActions;
        }

    }
}
