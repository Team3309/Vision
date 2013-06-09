package org.team3309.vision;

import java.awt.Color;
import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.Highgui;
import org.opencv.imgproc.Imgproc;

public class FrisbeeTracker {

	private boolean debug = false;
	
	private static final Scalar possibleColor = Colors.fromColor(Color.red);
	private static final Scalar frisbeeColor = Colors.fromColor(Color.blue);
	private static final Scalar closestColor = Colors.fromColor(Color.green);
	
	private static final double kHorizontalFOVDeg = 47.0;

	private static int MIN_HUE = 220;
	private static int MIN_SAT = 170;
	private static int MIN_VAL = 180;
	private static final double kMinRatio = .75;
	private static final double kMaxRatio = 1.25;
	private static final int kMinWidth = 20;
	private static final int kMaxWidth = 20;
	private static final int kHoleClosingIterations = 9;

	private ArrayList<Polygon> polygons;
	private Size size;
	private Mat morphKernel;
	private Mat hsv, hue, sat, val, bin, result;

	public FrisbeeTracker(boolean debug) {
		this.debug = debug;

		morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
				new Size(3, 3));
	}

	public Frisbee processImage(Mat rawImage, VisionFrame frame) {
		Imgproc.GaussianBlur(rawImage, rawImage, new Size(9,9), 2,2);
		if (size == null || size.width != rawImage.width()
				|| size.height != rawImage.height()) {
			size = new Size(rawImage.width(), rawImage.height());
			bin = new Mat(size, CvType.CV_8UC1);
			hsv = new Mat(size, CvType.CV_8UC3);
			hue = new Mat(size, CvType.CV_8UC1);
			sat = new Mat(size, CvType.CV_8UC1);
			val = new Mat(size, CvType.CV_8UC1);
			result = new Mat(size, rawImage.type());
		}
		result = rawImage.clone();

		Imgproc.cvtColor(rawImage, hsv, Imgproc.COLOR_BGR2HSV);
		Mat[] channels = Utils.split(rawImage, hue, sat, val);
		hue = channels[0];
		sat = channels[1];
		val = channels[2];

		Highgui.imwrite("hue.png", hue);
		Highgui.imwrite("sat.png", sat);
		Highgui.imwrite("val.png", val);

		Imgproc.threshold(hue, bin, MIN_HUE, 255, Imgproc.THRESH_BINARY);
		//frame.show(bin);
		Imgproc.threshold(sat, sat, MIN_SAT, 255, Imgproc.THRESH_BINARY);
		//frame.show(sat);
		Imgproc.threshold(val, val, MIN_VAL, 255, Imgproc.THRESH_BINARY);
		//frame.show(val);

		Core.bitwise_and(hue, bin, bin);
		Core.bitwise_and(bin, sat, bin);
		Core.bitwise_and(bin, val, bin);
		
		// Fill in any gaps using binary morphology
		//Imgproc.morphologyEx(bin, bin, Imgproc.MORPH_CLOSE, morphKernel, new Point(-1, -1), kHoleClosingIterations);
		//frame.show(bin);
		
		MatOfPoint2f[] contours = Utils.findConvexContours(bin);
		polygons = new ArrayList<Polygon>();
		for(MatOfPoint2f mat : contours){
			MatOfPoint2f approxCurve = new MatOfPoint2f();
			Imgproc.approxPolyDP(mat, approxCurve, 5, true);
			polygons.add(new Polygon(approxCurve));
		}
		ArrayList<Polygon> possible = new ArrayList<Polygon>();
		for(Polygon p : polygons){
			Core.polylines(result, p.getMatOfPointsList(), true, possibleColor);
			if(p.getNumVertices() > 6)
				possible.add(p);
		}
		System.out.println("found "+possible.size()+" possible circles");
		Polygon closest = null;
		double closestY = 0;
		for(Polygon p : possible){
			RotatedRect ellipse = Imgproc.fitEllipse(p.getMatOfPoint2f());
			MatOfPoint pts = new MatOfPoint();
			ellipse.size.height /= 2;
			ellipse.size.width /= 2;
			Core.ellipse2Poly(ellipse.center, ellipse.size, (int) ellipse.angle, 0, 360, 5, pts);
			Core.fillConvexPoly(result, pts, frisbeeColor);
			
			double fovWidth = 11.5*size.width/ellipse.size.width;
			double range = (fovWidth/2)/Math.atan(Math.toRadians(kHorizontalFOVDeg));
			Core.putText(result, String.valueOf(Math.round(range)), ellipse.center, Core.FONT_HERSHEY_COMPLEX, 1, Colors.RED);
			
			if(ellipse.center.y > closestY){
				closestY = ellipse.center.y;
				closest = p;
			}
		}
		if(closest != null){
			System.out.println("Found a target");
			RotatedRect ellipse = Imgproc.fitEllipse(closest.getMatOfPoint2f());
			MatOfPoint pts = new MatOfPoint();
			ellipse.size.height /= 2;
			ellipse.size.width /= 2;
			Core.ellipse2Poly(ellipse.center, ellipse.size, (int) ellipse.angle, 0, 360, 5, pts);
			Core.fillConvexPoly(result, pts, closestColor);
			
			double fovWidth = 11.5*size.width/ellipse.size.width;
			double range = (fovWidth/2)/Math.atan(Math.toRadians(kHorizontalFOVDeg));
			Core.putText(result, String.valueOf(Math.round(range)), ellipse.center, Core.FONT_HERSHEY_COMPLEX, 1, Colors.RED);
		}
		
		frame.show(result);
		return null;
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// load the opencv native library
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

		//Mat rawImage = Highgui.imread("/home/vmagro/workspace/Vision/frisbee_sample_1538.png");
		Mat rawImage = Highgui.imread("/home/vmagro/workspace/Vision/frisbee_sample_27.png");
		//Mat rawImage = Highgui.imread("/home/vmagro/workspace/Vision/perfect_circle.png");
		
		FrisbeeTracker tracker = new FrisbeeTracker(true);
		
		VisionFrame frame = new VisionFrame("Frisbee");

		tracker.processImage(rawImage, frame);
	}

}
