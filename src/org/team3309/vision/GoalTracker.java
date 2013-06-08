package org.team3309.vision;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.TreeMap;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.Highgui;
import org.opencv.imgproc.Imgproc;

/**
 * 
 * @author jrussell
 * @author vmagro
 */
public class GoalTracker {
	private Scalar targetColor = new Scalar(255, 0, 0);

	// Constants that need to be tuned
	private static final double kNearlyHorizontalSlope = Math.tan(Math
			.toRadians(20));
	private static final double kNearlyVerticalSlope = Math.tan(Math
			.toRadians(90 - 20));
	private static final int kMinWidth = 125;
	private static final int kMaxWidth = 400;
	private static final double kRangeOffset = 0.0;
	private static final int kHoleClosingIterations = 9;
	private static final int kHueThresh = 50 - 25;// 60-15 for daisy 2012
	private static final int kSatThresh = 75; // 200 for daisy 2012
	private static final int kValThresh = 200; // 55 for daisy 2012
	private static final double kMinRatio = .15; // .5 for daisy 2012
	private static final double kMaxRatio = .75; // 1 for daisy 2012

	private static final double kShooterOffsetDeg = 0; // the shooter may not be
														// perfectly aligned
														// with the base
	private static final double kShooterHeight = 20; // the height to the pivot
														// point of the arm

	private static final double kHorizontalFOVDeg = 47.0;
	private static final double kVerticalFOVDeg = 480.0 / 640.0 * kHorizontalFOVDeg;
	private static final double kCameraHeightIn = 54.0;
	private static final double kCameraPitchDeg = 21.0;
	private static final double kTopTargetHeightIn = 104.125 + 6; // 104+1/8 to
																	// the
																	// bottom, 6
																	// more
																	// inches to
																	// center

	private TreeMap<Double, Double> rangeTable;

	private boolean m_debugMode = false;

	// Store JavaCV temporaries as members to reduce memory management during
	// processing
	private Size size = null;
	private MatOfPoint2f[] contours;
	private ArrayList<Polygon> polygons;
	private Mat morphKernel;
	private Mat bin;
	private Mat hsv;
	private Mat hue;
	private Mat sat;
	private Mat val;
	private Point linePt1;
	private Point linePt2;

	public GoalTracker() {
		this(false);
	}

	public GoalTracker(boolean debug) {
		m_debugMode = debug;
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
				new Size(3, 3));
		// morphKernel = IplConvKernel.create(3, 3, 1, 1, Imgproc.CV_SHAPE_RECT,
		// null);

		rangeTable = new TreeMap<Double, Double>();
		// rangeTable.put(110.0, 3800.0+kRangeOffset);
		// rangeTable.put(120.0, 3900.0+kRangeOffset);
		// rangeTable.put(130.0, 4000.0+kRangeOffset);
		rangeTable.put(140.0, 3434.0 + kRangeOffset);
		rangeTable.put(150.0, 3499.0 + kRangeOffset);
		rangeTable.put(160.0, 3544.0 + kRangeOffset);
		rangeTable.put(170.0, 3574.0 + kRangeOffset);
		rangeTable.put(180.0, 3609.0 + kRangeOffset);
		rangeTable.put(190.0, 3664.0 + kRangeOffset);
		rangeTable.put(200.0, 3854.0 + kRangeOffset);
		rangeTable.put(210.0, 4034.0 + kRangeOffset);
		rangeTable.put(220.0, 4284.0 + kRangeOffset);
		rangeTable.put(230.0, 4434.0 + kRangeOffset);
		rangeTable.put(240.0, 4584.0 + kRangeOffset);
		rangeTable.put(250.0, 4794.0 + kRangeOffset);
		rangeTable.put(260.0, 5034.0 + kRangeOffset);
		rangeTable.put(270.0, 5234.0 + kRangeOffset);

		// DaisyExtensions.init();
	}

	public double getRPMsForRange(double range) {
		double lowKey = -1.0;
		double lowVal = -1.0;
		for (double key : rangeTable.keySet()) {
			if (range < key) {
				double highVal = rangeTable.get(key);
				if (lowKey > 0.0) {
					double m = (range - lowKey) / (key - lowKey);
					return lowVal + m * (highVal - lowVal);
				} else
					return highVal;
			}
			lowKey = key;
			lowVal = rangeTable.get(key);
		}

		return 5234.0 + kRangeOffset;
	}

	public Goal processImage(Mat rawImage, VisionFrame resultFrame) {
		double heading = 0.0;

		/*
		 * // Get the current heading of the robot first if( !m_debugMode ) {
		 * try { heading = Robot.getTable().getDouble("Heading"); } catch(
		 * NoSuchElementException e) { } catch( IllegalArgumentException e ) { }
		 * }
		 */

		if (size == null || size.width != rawImage.width()
				|| size.height != rawImage.height()) {
			size = new Size(rawImage.width(), rawImage.height());
			bin = new Mat(size, CvType.CV_8UC1);
			hsv = new Mat(size, CvType.CV_8UC3);
			hue = new Mat(size, CvType.CV_8UC1);
			sat = new Mat(size, CvType.CV_8UC1);
			val = new Mat(size, CvType.CV_8UC1);
			linePt1 = new Point(size.width / 2, 0);
			linePt2 = new Point(size.width / 2, size.height);
		}
		// Get the raw Mats for OpenCV
		Mat input = rawImage;

		// Convert to HSV color space
		Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
		ArrayList<Mat> channels = new ArrayList<Mat>();
		channels.add(hue);
		channels.add(sat);
		channels.add(val);
		Core.split(hsv, channels);
		hue = channels.get(0);
		sat = channels.get(1);
		val = channels.get(2);

		// Threshold each component separately
		// Hue
		// NOTE: Red is at the end of the color space, so you need to OR
		// together
		// a thresh and inverted thresh in order to get points that are red
		Imgproc.threshold(hue, bin, kHueThresh, 255, Imgproc.THRESH_BINARY);
		// Imgproc.threshold(hue, hue, 60+15, 255, Imgproc.THRESH_BINARY_INV);

		// Saturation
		Imgproc.threshold(sat, sat, kSatThresh, 255, Imgproc.THRESH_BINARY);

		// Value
		Imgproc.threshold(val, val, kValThresh, 255, Imgproc.THRESH_BINARY);

		// Combine the results to obtain our binary image which should for the
		// most
		// part only contain pixels that we care about
		Core.bitwise_and(hue, bin, bin);
		Core.bitwise_and(bin, sat, bin);
		Core.bitwise_and(bin, val, bin);

		// Fill in any gaps using binary morphology
		Imgproc.morphologyEx(bin, bin, Imgproc.MORPH_CLOSE, morphKernel,
				new Point(-1, -1), kHoleClosingIterations);

		// Find contours
		contours = Utils.findConvexContours(bin);
		polygons = new ArrayList<Polygon>();
		for (MatOfPoint2f c : contours) {
			RotatedRect bounding = Imgproc.minAreaRect(c);
			double ratio = ((double) bounding.size.width)
					/ ((double) bounding.size.height);
			// height and width are switched so that the height of the bounding
			// box is the width of the goal
			if (ratio > kMinRatio && ratio < kMaxRatio
					&& bounding.size.height > kMinWidth
					&& bounding.size.height < kMaxWidth) {
				MatOfPoint2f approxCurve = new MatOfPoint2f();
				Imgproc.approxPolyDP(c, approxCurve, 20, true);
				polygons.add(new Polygon(approxCurve));
			}
		}

		Polygon square = null;
		int highest = Integer.MAX_VALUE;

		for (Polygon p : polygons) {
			if (p.isConvex() && p.getNumVertices() == 4) {
				// We passed the first test...we fit a rectangle to the polygon
				// Now do some more tests

				Point[] points = p.getPoints();
				// We expect to see a top line that is nearly horizontal, and
				// two side lines that are nearly vertical
				int numNearlyHorizontal = 0;
				int numNearlyVertical = 0;
				for (int i = 0; i < 4; i++) {
					double dy = points[i].y - points[(i + 1) % 4].y;
					double dx = points[i].x - points[(i + 1) % 4].x;
					double slope = Double.MAX_VALUE;
					if (dx != 0)
						slope = Math.abs(dy / dx);

					if (slope < kNearlyHorizontalSlope)
						++numNearlyHorizontal;
					else if (slope > kNearlyVerticalSlope)
						++numNearlyVertical;
				}

				if (numNearlyHorizontal >= 1 && numNearlyVertical == 2) {
					Core.polylines(rawImage, p.getMatOfPointsList(), true,
							new Scalar(255, 0, 0), 2);

					if (p.getCenter().y < highest) { // y coordinates use 0 at
														// the top
						square = p;
						highest = (int) p.getCenter().y;
					}
				}
			} else {
				Core.polylines(rawImage, p.getMatOfPointsList(), true,
						new Scalar(0, 255, 255), 2);
			}
		}

		// Draw a crosshair
		if (m_debugMode)
			Core.line(rawImage, linePt1, linePt2, targetColor, 2);

		if (square != null) {
			double x = square.getCenter().x;
			x = (2 * (x / size.width)) - 1;
			double y = square.getCenter().y;
			y = -((2 * (y / size.height)) - 1);

			double azimuth = this.boundAngle0to360Degrees(x * kHorizontalFOVDeg
					/ 2.0 + heading - kShooterOffsetDeg);
			double range = (kTopTargetHeightIn - kCameraHeightIn)
					/ Math.tan((y * kVerticalFOVDeg / 2.0 + kCameraPitchDeg)
							* Math.PI / 180.0);
			double angle = Math.toDegrees(Math.atan2(kTopTargetHeightIn
					- kShooterHeight, range));
			double rpms = getRPMsForRange(range);
			
			Goal target = new Goal(square, range, azimuth, angle);

			System.out.println("Target found");
			System.out.println("x: " + x);
			System.out.println("y: " + y);
			System.out.println("azimuth: " + azimuth);
			System.out.println("range: " + range);
			System.out.println("angle deg: " + angle);
			System.out.println("rpms: " + rpms);
			Core.polylines(rawImage, square.getMatOfPointsList(), true,
					targetColor, 7);
			Core.putText(rawImage, String.valueOf(Math.round(angle)),
					square.getCenter(), Core.FONT_HERSHEY_COMPLEX, 1,
					new Scalar(255, 0, 0));
			if (resultFrame != null)
				resultFrame.show(rawImage);
			
			return target;
		} else {
			if(resultFrame != null)
				resultFrame.show(rawImage);
			return null;
		}
	}

	private double boundAngle0to360Degrees(double angle) {
		// Naive algorithm
		while (angle >= 360.0) {
			angle -= 360.0;
		}
		while (angle < 0.0) {
			angle += 360.0;
		}
		return angle;
	}

}