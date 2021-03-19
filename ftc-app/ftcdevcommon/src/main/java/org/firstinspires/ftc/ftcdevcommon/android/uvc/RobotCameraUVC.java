package org.firstinspires.ftc.ftcdevcommon.android.uvc;

// Final immutable class that holds all of the necessary information about one camera.
public final class RobotCameraUVC {
	private final String product_name;
	private final int vendor_id;
	private final int product_id;
	private final String serial_number;
	private final int width;
	private final int height;

	// Note: the USB device name can change based on the configuration of the USB hub
	// so we can't set it in advance.
	//**PY 8/21/2019 I don't like setting a crucial field after construction.
	// Beter to make this a final class. Check usages of the device name. You
	// may have to create a map where key = USB device name and value = this class.
	public RobotCameraUVC(String pProductName, int pVendorId, int pProductId, String pSerialNumber,
                          int pWidth, int pHeight) {
		product_name = pProductName;
		vendor_id = pVendorId;
		product_id = pProductId;
		serial_number = pSerialNumber;
		width = pWidth;
		height = pHeight;
	}

	public String getProductName() {
		return product_name;
	}

    public int getVendorId() {
        return vendor_id;
    }
    
    public int getProductId() {
        return product_id;
    }

    public String getSerialNumber() { return serial_number; }

    public int getWidth() { return width; }

    public int getHeight() { return height; }
}