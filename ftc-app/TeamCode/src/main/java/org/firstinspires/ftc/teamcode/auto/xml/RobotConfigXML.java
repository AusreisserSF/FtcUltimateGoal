package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.RobotXMLElement;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathFactory;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;


// Usage
/*
    // Construction
    RobotConfigXML configXML = new RobotConfigXML(xmlDirectory);

    // local variable
    XPathAccess configXPath;

    // As needed
    configXPath = robotConfigXML.getPath("WOBBLE_SERVO"); 

    String upDown = configXPath.getString("position");
*/

public class RobotConfigXML {

    public static final String TAG = "RobotConfigXML";
    private static final String FILE_NAME = "RobotConfig.xml";

    // IntelliJ only
    /*
    private static final String JAXP_SCHEMA_LANGUAGE =
            "http://java.sun.com/xml/jaxp/properties/schemaLanguage";
    private static final String W3C_XML_SCHEMA =
            "http://www.w3.org/2001/XMLSchema";
     */
    // End IntelliJ only

    private final XPathFactory xpathFactory = XPathFactory.newInstance();
    private final XPath xpath = xpathFactory.newXPath();
    private final Map<String, RobotXMLElement> robotElementCollection = new HashMap<>();

    public RobotConfigXML(String pWorkingDirectory) throws ParserConfigurationException, SAXException, IOException {

        // IntelliJ only
        /*
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        dbFactory.setNamespaceAware(true);
        dbFactory.setValidating(true);
        dbFactory.setAttribute(JAXP_SCHEMA_LANGUAGE, W3C_XML_SCHEMA);

        //## ONLY works with a validating parser.
        dbFactory.setIgnoringElementContentWhitespace(true);
         */
        // End IntelliJ only

        // Android only
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        // ONLY works with a validating parser (DTD or schema) dbFactory.setIgnoringElementContentWhitespace(true);
        //PY 8/17/2019 Android throws UnsupportedOperationException dbFactory.setXIncludeAware(true);
        // End Android only

        DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
        String configFilename = pWorkingDirectory + FILE_NAME;
        Document document = dBuilder.parse(new File(configFilename));

        // Collect all of the elements from the XML file.
        Element robotConfigRoot = document.getDocumentElement(); 
        NodeList configChildren = robotConfigRoot.getChildNodes();
        Node oneConfigNode;

        // Iterate through the XML elements in the configuration
        // file and store each one into a Map whose key is the
        // tag name of the element.
        RobotXMLElement robotXMLElement;
        RobotXMLElement mappedRobotXMLElement;
        for (int i = 0; i < configChildren.getLength(); i++) {
            oneConfigNode = configChildren.item(i);

            if (oneConfigNode.getNodeType() != Node.ELEMENT_NODE)
                continue;

            robotXMLElement = new RobotXMLElement((Element) oneConfigNode);
            mappedRobotXMLElement = robotElementCollection.put(robotXMLElement.getRobotXMLElementName(), robotXMLElement);
            if (mappedRobotXMLElement != null)
               RobotLogCommon.d(TAG, "Duplicate element " + robotXMLElement.getRobotXMLElementName()); 
        }

        RobotLogCommon.i(TAG, "In RobotConfigXML; opened and parsed the XML file"); 
    }
   
   public XPathAccess getPath(String pElementName) {
        if (pElementName == null)
            throw new AutonomousRobotException(TAG, "Null element name not allowed");

        RobotXMLElement mappedRobotXMLElement = robotElementCollection.get(pElementName);
        if (mappedRobotXMLElement == null)
          throw new AutonomousRobotException(TAG, "No such element in RobotConfig.xml: " + pElementName);

        return new XPathAccess(xpath, mappedRobotXMLElement.getRobotXMLElement(), mappedRobotXMLElement.getRobotXMLElementName());
    }

}
