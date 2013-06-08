package org.team3309.vision;

import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.Highgui;
import org.opencv.imgproc.Imgproc;

public class FrisbeeTracker {

	private boolean debug = false;

	private static final int MIN_HUE = 225;
	private static final int MIN_SAT = 225;
	private static final int MIN_VAL = 225;
	private static final double kMinRatio = .75;
	private static final double kMaxRatio = 1.25;
	private static final int kMinWidth = 20;
	private static final int kMaxWidth = 20;

	private ArrayList<Polygon> polygons;
	private MatOfPoint2f[] contours;
	private Size size;
	private Mat hsv, hue, sat, val, bin;

	public FrisbeeTracker(boolean debug) {
		this.debug = debug;
	}

	public Frisbee processImage(Mat rawImage, VisionFrame resultFrame) {
		if (size == null || size.width != rawImage.width()
				|| size.height != rawImage.height()) {
			size = new Size(rawImage.width(), rawImage.height());
			bin = new Mat(size, CvType.CV_8UC1);
			hsv = new Mat(size, CvType.CV_8UC3);
			hue = new Mat(size, CvType.CV_8UC1);
			sat = new Mat(size, CvType.CV_8UC1);
			val = new Mat(size, CvType.CV_8UC1);
		}

		Imgproc.cvtColor(rawImage, hsv, Imgproc.COLOR_BGR2HSV);
		Mat[] channels = Utils.split(rawImage, hue, sat, val);
		hue = channels[0];
		sat = channels[1];
		val = channels[2];

		Highgui.imwrite("hue.png", hue);
		Highgui.imwrite("sat.png", sat);
		Highgui.imwrite("val.png", val);

		Imgproc.threshold(hue, bin, MIN_HUE, 255, Imgproc.THRESH_BINARY);
		Imgproc.threshold(sat, sat, MIN_SAT, 255, Imgproc.THRESH_BINARY);
		Imgproc.threshold(val, val, MIN_VAL, 255, Imgproc.THRESH_BINARY);

		Core.bitwise_and(hue, bin, bin);
		Core.bitwise_and(bin, sat, bin);
		Core.bitwise_and(bin, val, bin);

		contours = Utils.findConvexContours(bin);
		polygons = new ArrayList<Polygon>();
		for (MatOfPoint2f c : contours) {
			MatOfPoint2f approxCurve = new MatOfPoint2f();
			Imgproc.approxPolyDP(c, approxCurve, 20, true);
			Polygon p = new Polygon(approxCurve);
			polygons.add(p);
			Core.polylines(rawImage, p.getMatOfPointsList(), true, new Scalar(255,0,0));
		}
		ArrayList<Polygon> possibleFrisbees = new ArrayList<Polygon>();
		for(Polygon p : polygons){
			if(p.getNumVertices() > 5)
				possibleFrisbees.add(p);
		}
		System.out.println("found " + possibleFrisbees.size() + " polygons");

		resultFrame.show(rawImage);
		return null;
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// load the opencv native library
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

		Mat rawImage = Highgui
				.imread("/home/vmagro/workspace/Vision/frisbee_sample_1538.png");
		VisionFrame frame = new VisionFrame("Frisbee");

		FrisbeeTracker tracker = new FrisbeeTracker(true);
		tracker.processImage(rawImage, frame);
	}

}
