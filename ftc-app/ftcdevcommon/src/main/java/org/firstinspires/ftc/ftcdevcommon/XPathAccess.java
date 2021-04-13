package org.firstinspires.ftc.ftcdevcommon;

import javax.xml.xpath.*;
import org.w3c.dom.Element;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

// Generic access to XML elements via XPath.
public class XPathAccess {

	// --------- CLASS VARIABLES ----------
	private static final String TAG = "XPathAccess";

	private final XPath xpath;
	private final Element xmlElement;

	// --------- CONSTRUCTORS ----------
	//## Compromise: pass an XML element in to the constructor.
	// It would be possible to pass the element in to every method
	// but the syntax is simpler this way.
	public XPathAccess(RobotXMLElement pRobotXMLElement) {
		XPathFactory xpathFactory = XPathFactory.newInstance();
		xpath = xpathFactory.newXPath();
		xmlElement = pRobotXMLElement.getRobotXMLElement();
	}

	// --------- FUNCTIONS ----------

 	// Gets a text string from an element or attribute and checks it against a list
	// of valid values. Always returns lower case.
	public String getStringInRange(String pPath, List<String>pRangeList) throws XPathExpressionException {
		String text = getString(pPath);
		if (!pRangeList.contains(text))
			throw new AutonomousRobotException(TAG, "Text value " + text + " is not in a valid value");

		return text;
	}

	// Works for both elements and attributes.
	// Enforces lower case throughout.
	public String getString(String pPath) throws XPathExpressionException {
	    return getString(pPath, false);
	}

	public String getString(String pPath, boolean pIgnoreCase) throws XPathExpressionException {
		// Crude but effective in our environment: if the path includes a '@'
		// consider it to be an XML attribute.
		String text;
		if (pPath.contains("@"))
			text = getAttributeText(pPath);
		else
			text = getElementText(pPath);

		if (text.isEmpty())
			throw new AutonomousRobotException(TAG, "Requested item " + pPath + " does not exist in " + xmlElement.getTagName());

		// Make sure that the text is lower case if case is significant.
		if (!pIgnoreCase && !text.equals(text.toLowerCase()))
			throw new AutonomousRobotException(TAG, "Expected lower case in text " + text);

		return text;
	}

	// Gets a text string from an element or attribute and checks it against a list
	// of valid values.
	public String getStringInRange(String pPath, String pDefaultIfMissing, List<String>pRangeList) throws XPathExpressionException {

		String text = getString(pPath, pDefaultIfMissing);
		if (text.equals(pDefaultIfMissing))
			return text;

		if (!pRangeList.contains(text))
			throw new AutonomousRobotException(TAG, "Text value " + text + " is not in a valid value");

		return text;
	}

	// If the requested text is not present, this method returns the default value.
	// Otherwise it returns the text of the element or attribute. Enforces lower
	// case throughout.
	public String getString(String pPath, String pDefaultIfMissing) throws XPathExpressionException {

		// Make sure that the default value is lower case.
		if (!pDefaultIfMissing.equals(pDefaultIfMissing.toLowerCase()))
			throw new AutonomousRobotException(TAG, "Expected lower case in default " + pDefaultIfMissing);

		// Crude but effective in our environment: if the path includes a '@'
		// consider it to be an XML attribute.
		String text;
		if (pPath.contains("@"))
			text = getAttributeText(pPath);
		else
			text = getElementText(pPath);

		if (text.isEmpty())
			return pDefaultIfMissing;

		// Make sure that the text is lower case.
		if (!text.equals(text.toLowerCase()))
			throw new AutonomousRobotException(TAG, "Expected lower case in text " + text);

		return text;
	}

	public List<String> validRange(String... pRangeValues) {

		List<String> finalRangeList = new ArrayList<>();

		if (pRangeValues == null)
			return finalRangeList;

		// We're not checking the values in the range list for lower case
		// because lower case is enforced everywhere else. So the range
		// check will automatically fail on upper or mixed case strings.
		finalRangeList.addAll(Arrays.asList(pRangeValues));
		return finalRangeList;
	}

	public int getInt(String pPath) throws XPathExpressionException {
		String text = getElementText(pPath);
		if (text.isEmpty())
			throw new AutonomousRobotException(TAG, "Requested item " + pPath + " does not exist in " + xmlElement.getTagName());

		return getIntFromText(text, pPath);
	}

	public int getInt(String pPath, int defaultValue) throws XPathExpressionException {
		String text = getElementText(pPath);
		if (text.isEmpty())
			return defaultValue;

		return getIntFromText(text, pPath);
	}

	public double getDouble(String pPath) throws XPathExpressionException {
		String text = getElementText(pPath);
		if (text.isEmpty())
			throw new AutonomousRobotException(TAG, "Requested item " + pPath + " does not exist in " + xmlElement.getTagName());

		return getDoubleFromText(text, pPath);
	}

	public double getDouble(String pPath, double defaultValue) throws XPathExpressionException {
		String text = getElementText(pPath);
		if (text.isEmpty())
			return defaultValue;

		return getDoubleFromText(text, pPath);
	}
	
	public boolean getBoolean(String pPath) throws XPathExpressionException {
		String text = getElementText(pPath);
		if (text.isEmpty())
			throw new AutonomousRobotException(TAG, "Requested item " + pPath + " does not exist in " + xmlElement.getTagName());

		return getBooleanFromText(text, pPath);
	}

	public boolean getBoolean(String pPath, boolean defaultValue) throws XPathExpressionException {
		String text = getElementText(pPath);
		if (text.isEmpty())
			return defaultValue;

		return getBooleanFromText(text, pPath);
	}

	// Assumes that the path contains a valid XPath attribute expression.
	private String getAttributeText(String pPath) throws XPathExpressionException {
		XPathExpression expr = xpath.compile(pPath);
		// Trim needed because only validating parsers will strip white space.
		return ((String) expr.evaluate(xmlElement, XPathConstants.STRING)).trim();
	}

	public String getElementText(String pPath) throws XPathExpressionException {
        XPathExpression expr = xpath.compile(pPath + "/text()");
        // Trim needed because only validating parsers will strip white space.
		return ((String) expr.evaluate(xmlElement, XPathConstants.STRING)).trim();
	}
	
	public int getIntFromText(String pIntText, String pNodeName) {
		if (pIntText.isEmpty())
			throw new AutonomousRobotException(TAG, "Requested item " + pIntText + " does not exist in " + xmlElement.getTagName());

		int itemInt;
		try {
			itemInt = Integer.parseInt(pIntText);
		} catch (NumberFormatException ex) {
			throw new AutonomousRobotException(TAG, "Value in " + pNodeName + ": " + pIntText + " is not an int");
		}

		return itemInt;
	}

	public double getDoubleFromText(String pDoubleText, String pNodeName) {
		if (pDoubleText.isEmpty())
			throw new AutonomousRobotException(TAG, "Requested item " + pNodeName + " does not exist in " + xmlElement.getTagName());

		double itemDouble;
		try {
			itemDouble = Double.parseDouble(pDoubleText);
		} catch (NumberFormatException ex) {
			throw new AutonomousRobotException(TAG, "Value in " + pNodeName + ": " + pDoubleText + " is not a double");
		}

		return itemDouble;
	}
	
	public boolean getBooleanFromText(String pBoolText, String pNodeName) {
		if (pBoolText.isEmpty())
			throw new AutonomousRobotException(TAG, "Requested item " + pNodeName + " does not exist in " + xmlElement.getTagName());

		if (pBoolText.equals("true"))
			return true;
		if (pBoolText.equals("false"))
			return false;
		throw new AutonomousRobotException(TAG, "Value in " + pNodeName + ": " + pBoolText + " is not a boolean");
	}
}
