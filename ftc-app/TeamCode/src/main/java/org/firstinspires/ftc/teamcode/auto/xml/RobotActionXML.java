package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;

public class RobotActionXML {

    public static final String TAG = "RobotActionXML";
    private static final String FILE_NAME = "RobotAction.xml";

    /*
    // IntelliJ only
    private static final String JAXP_SCHEMA_LANGUAGE =
            "http://java.sun.com/xml/jaxp/properties/schemaLanguage";
    private static final String W3C_XML_SCHEMA =
            "http://www.w3.org/2001/XMLSchema";
     */

    private final Level minimumLoggingLevel;
    private final boolean initVuforia;
    private final NodeList opModeNodes;
    private final int opModeNodeCount;

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
 */
        // Android only
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        // ONLY works with a validating parser (DTD or schema) dbFactory.setIgnoringElementContentWhitespace(true);
        //PY 8/17/2019 Android throws UnsupportedOperationException dbFactory.setXIncludeAware(true);
        // End Android only

        DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
        String actionFilename = pWorkingDirectory + FILE_NAME;
        Document document = dBuilder.parse(new File(actionFilename));

        Element robotActionRoot = document.getDocumentElement();
        String debugLoggingLevel = robotActionRoot.getAttribute("debugLevel");
        // Using Level.parse allows all integer values.
        if (debugLoggingLevel.trim().isEmpty())
            minimumLoggingLevel = null;
        else {
            switch (debugLoggingLevel) {
                case "d": {
                    minimumLoggingLevel = Level.FINE;
                    break;
                }
                case "v": {
                    minimumLoggingLevel = Level.FINER;
                    break;
                }
                case "vv": {
                    minimumLoggingLevel = Level.FINEST;
                    break;
                }
                default: {
                    throw new AutonomousRobotException(TAG, "Invalid debug log option");
                }
            }
        }

        // For testing without the camera.
        String initVuforiaAttr = robotActionRoot.getAttribute("init_vuforia").trim();
        if (initVuforiaAttr.equals("yes"))
            initVuforia = true;
        else
            initVuforia = false;

        opModeNodes = document.getElementsByTagName("OpMode");
        opModeNodeCount = opModeNodes.getLength();
        RobotLogCommon.i(TAG, "In RobotActionXML; opened and parsed the XML file");
        RobotLogCommon.i(TAG, "Found " + opModeNodeCount + " OpModes");
    }

    // Return the value of the attribute from the root element or null
    // if the attribute is missing.
    public Level getMinimumLoggingLevel() {
        return minimumLoggingLevel;
    }

    // Return the Vurforia initialization setting.
    public boolean initializeVuforia() {
        return initVuforia;
    }

    // Iterate through the top-level elements of the RobotAction.xml file, find the requested opMode,
    // create a CommandXML class for each of the commands in the OpMode, and return them in a List.
    // If the requested OpMode has an "alliance" attribute, return it also (or a default).
    public List<CommandXML> getOpModeCommands(String pOpMode) {

        List<CommandXML> commands = new ArrayList<>();

        // Find the XML element that matches the requested OpMode.
        String opModeIdAttr;
        Node matchingOpMode = null;
        Node oneOpModeNode;
        for (int i = 0; i < opModeNodeCount; i++) {
            oneOpModeNode = opModeNodes.item(i);

            if (oneOpModeNode.getNodeType() != Node.ELEMENT_NODE)
                continue;

            opModeIdAttr = ((Element) oneOpModeNode).getAttribute("id");
            if (opModeIdAttr.equals(pOpMode)) {
                matchingOpMode = oneOpModeNode;
                RobotLogCommon.d(TAG, "Found a matching opmode " + opModeIdAttr);
               break;
            }
        }

        if (matchingOpMode == null)
            throw new AutonomousRobotException(TAG, "Did not find match for opmode " + pOpMode);

        // Now iterate through the command children of the selected OpMode.
        NodeList opModeChildren = matchingOpMode.getChildNodes();
        Node oneCommandNode;

        CommandXML commandXML;
        for (int i = 0; i < opModeChildren.getLength(); i++) {
            oneCommandNode = opModeChildren.item(i);

            if (oneCommandNode.getNodeType() != Node.ELEMENT_NODE)
                continue;

            commandXML = new CommandXML((Element) oneCommandNode);
            commands.add(commandXML);
        }

        return commands;
    }

	/*
	 public static class Utils {
       public static Element getNextElement(Element el) {
         Node nd = el.getNextSibling();
         while (nd != null) {
           if (nd.getNodeType() == Node.ELEMENT_NODE) {
            return (Element)nd;
         }
         nd = nd.getNextSibling();
        }
       return null;
       }
      }
	 */

    // Holds the XML element for a command in an xml file.
    //   <command id="MOVE_FINGERS">
    //     <position>down</position>
    //   </command>
    // Use the companion class XPathAccess to get attributes,
    // child nodes, and text relative to this node.
    public static class CommandXML {

        // --------- CLASS VARIABLES ----------
        private final Element commandElement;
        private final String commandId;

        // --------- CONSTRUCTORS ----------
        public CommandXML(Element pCommandElement) {
            commandElement = pCommandElement;

            // The command id is the tag name of the element
            commandId = pCommandElement.getTagName();
        }

        // --------- FUNCTIONS ----------
        public Element getCommandElement() {
            return commandElement;
        }

        public String getCommandId() {
            return commandId;
        }

    }
}
