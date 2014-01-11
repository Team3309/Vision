package org.team3309.vision;

import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/**
 * This is the Friarbots vision code for 2013. Code loosely based on Miss Daisy
 * 341's code for 2012.
 * 
 * @author vmagro
 */
public class GoalTracker {
	private Scalar targetColor = Colors.BLUE;

	// Constants that need to be tuned
	private static final double kNearlyHorizontalSlope = Math.tan(Math
			.toRadians(20));
	private static final double kNearlyVerticalSlope = Math.tan(Math
			.toRadians(90 - 20));
	private static final int kMinWidth = 50;//125;
	private static final int kMaxWidth = 1000;//400;
	private static final int kHoleClosingIterations = 9;
	
	/*private static final int kHueThresh = 50 - 25;// 60-15 for daisy 2012
	private static final int kSatThresh = 75; // 200 for daisy 2012
	private static final int kValThresh = 200; // 55 for daisy 2012 */
	public static int kHueMin = 78;// 60-15 for daisy 2012
	public static int kHueMax = 98;
	public static int kSatMin = 172; // 200 for daisy 2012
	public static int kSatMax = 255;
	public static int kValMin = 212; // 55 for daisy 2012
	public static int kValMax = 255;

    public static int kCanMin = 0;
    public static int kCanMax = 255;

    public static int kShapeMin = 254;
    public static int kShapeMax = 255;
    
    public static int kFrameMillis = 0;
	
	
	private static final double kMinRatio = 0.5;//.15; // .5 for daisy 2012
	private static final double kMaxRatio = 10;//.75; // 1 for daisy 2012

	private static final double kShooterOffsetDeg = 0; // the shooter may not be
														// perfectly aligned
														// with the base
	private static final double kShooterHeight = 20; // the height to the pivot
														// point of the arm

	private static final double kHorizontalFOVDeg = 51.3;
	private static final double kVerticalFOVDeg = 480.0 / 640.0 * kHorizontalFOVDeg;
	private static final double kVerticalFovRad = Math.toRadians(kVerticalFOVDeg);
	
	private static final double kImgHeight = 480;
	private static final double kCameraHeightIn = 24;//35.0;
	private static final double kCameraPitchDeg = 0.0;
	private static final double kTopTargetHeightIn = 47;//104.125 + 6; // 104+1/8 to
																	// the
																	// bottom, 6
																	// more
																	// inches to
																	// center

	private static final double kTargetWidthIn = 24.125; //rebound rumble target is 24+1/8

    private static final double sobel_kernel_size = 3;

	private static final double EPSILON = 20;

	private static final int DESIRED_LINES = 100;
	
	private boolean m_debugMode = false;

	// Store OpenCV temporaries as members to reduce memory management during
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
    private Mat can;

    private Mat detected_edges;
    private Mat detected_lines;
    private Mat lines;
	private Point linePt1;
	private Point linePt2;
	
	private int minimumHoughLineLength = 20;

	private boolean hasHitDesiredLines;

	private long lastDraw = System.currentTimeMillis();


	public static boolean kDidDraw;

	public GoalTracker() {
		this(false);
	}

	public GoalTracker(boolean debug) {
		m_debugMode = debug;
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
				new Size(3, 3));
	}

    public Goal processImage(Mat rawImage, JFrame calibrationWindow)
    {
    	long startTime = System.currentTimeMillis();
        double heading = 0.0;

		/*
		 * // Get the current heading of the robot first if( !m_debugMode ) {
		 * try { heading = Robot.getTable().getDouble("Heading"); } catch(
		 * NoSuchElementException e) { } catch( IllegalArgumentException e ) { }
		 * }
		 */

        if (size == null || size.width != rawImage.width()
                || size.height != rawImage.height())
        {
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
        //Imgproc.threshold(hue, bin, kHueMin, 255, Imgproc.THRESH_BINARY);
        Core.inRange(hue, new Scalar(kHueMin), new Scalar(kHueMax), hue);
//        Imgproc.threshold(hue, hue, kHueMax, 255, Imgproc.THRESH_TOZERO_INV);
//        Imgproc.threshold(hue, hue, kHueMin, 255, Imgproc.THRESH_TOZERO);

        // Saturation
        //Imgproc.threshold(sat, sat, kSatMin, 255, Imgproc.THRESH_BINARY);
        Core.inRange(sat, new Scalar(kSatMin), new Scalar(kSatMax), sat);
//        Imgproc.threshold(sat, sat, kSatMax, 255, Imgproc.THRESH_TOZERO_INV);
//        Imgproc.threshold(sat, sat, kSatMin, 255, Imgproc.THRESH_TOZERO);

        // Value
        //Imgproc.threshold(val, val, kValMin, 255, Imgproc.THRESH_BINARY);
        Core.inRange(val, new Scalar(kValMin), new Scalar(kValMax), val);
//        Imgproc.threshold(val, val, kValMax, 255, Imgproc.THRESH_TOZERO_INV);
//        Imgproc.threshold(val, val, kValMin, 255, Imgproc.THRESH_TOZERO);

        bin.setTo(Scalar.all(0));
        // Combine the results to obtain our binary image which should for the
        // most
        // part only contain pixels that we care about
        Core.bitwise_or(hue, bin, bin);
        Core.bitwise_and(bin, sat, bin);
        Core.bitwise_and(bin, val, bin);

        can = new Mat(bin.size(), bin.type());
       /* detected_edges = new Mat(bin.size(),  bin.type());*/

        can.setTo(Scalar.all(0));

        //detected_edges.copyTo(can);                                             `
        detected_lines = new Mat(input.size(), input.type());

        int erosionShape, dilationShape;
        boolean wut = kShapeMax*3 < (255-kShapeMin);

        if(kShapeMax*3 < (255-kShapeMin))
        {
/*            erosionShape = Imgproc.MORPH_CROSS;*/
            erosionShape = Imgproc.MORPH_ELLIPSE;
        }
        else if (kShapeMax*3 < ((255 - kShapeMin) * 2))
        {
            erosionShape = Imgproc.MORPH_ELLIPSE;
        }
        else
        {
/*            erosionShape = Imgproc.MORPH_RECT;*/
            erosionShape = Imgproc.MORPH_ELLIPSE;
        }

        if(kShapeMin*3 < (kShapeMax))
        {
/*            dilationShape = Imgproc.MORPH_CROSS;*/
            dilationShape = Imgproc.MORPH_ELLIPSE;
        }
        else if (kShapeMin*3 < ((kShapeMax) * 2))
        {
            dilationShape = Imgproc.MORPH_ELLIPSE;
        }
        else
        {
/*            dilationShape = Imgproc.MORPH_RECT;*/
            dilationShape = Imgproc.MORPH_ELLIPSE;
        }


/*        Mat elementDilation =  Imgproc.getStructuringElement(dilationShape,
                new Size(2 * kCanMin + 1, 2 * kCanMin + 1),
                new Point(kCanMin, kCanMin));
        /// Apply the erosion operation
        Imgproc.erode(bin, can, elementDilation);

        Mat elementErosion = Imgproc.getStructuringElement(erosionShape,
                new Size(2 * (255-kCanMax) + 1, 2* (255 - kCanMax) + 1),
                new Point((255-kCanMax), (255-kCanMax)));
        /// Apply the erosion operation
        Imgproc.dilate(can, can, elementErosion );*/




        Mat elementErosion = Imgproc.getStructuringElement(erosionShape,
                new Size(2 * (255-kCanMax) + 1, 2* (255 - kCanMax) + 1),
                new Point((255-kCanMax), (255-kCanMax)));
        /// Apply the erosion operation
        Imgproc.dilate(bin, can, elementErosion );

        Mat elementDilation =  Imgproc.getStructuringElement(dilationShape,
                new Size(2 * kCanMin + 1, 2 * kCanMin + 1),
                new Point(kCanMin, kCanMin));
        /// Apply the erosion operation
        Imgproc.erode(can, can, elementDilation);


        lines = new Mat();
        Imgproc.HoughLinesP(can, lines, 1, Math.PI/180, 80, kShapeMin, 30);
        
        /*
         * Sort our lines by horizontal and vertical
         */
        List<Line> listOfHorizontalLines = new ArrayList<Line>();
        List<Line> listOfVerticalLines = new ArrayList<Line>();
        List<Line> listOfLines = new ArrayList<Line>();
        //check for lines intersecting this line within the frame
        int numLines = 0;
        for (int x = 0; x < lines.cols(); x++)
        {
            numLines = lines.cols();
            double[] vec = lines.get(0, x);
            double x1 = vec[0],
                   y1 = vec[1],
                   x2 = vec[2],
                   y2 = vec[3];
            Point start = new Point(x1, y1);
            Point end = new Point(x2, y2);

            double slope = (y2 - y1) / (x2 - x1);
            // The slope of the line determines the if it is horizontal or vertical
            //
            // vertical (More than 1)
            //  |   / slope of 1
            //  |  /
            //  | /
            //  |/
            //  /--------- horizontal (less than 1)
            if (Math.abs(slope) > 1) {
            	listOfVerticalLines.add(new Line(start,end));
            } else {
            	listOfHorizontalLines.add(new Line(start,end));
            }
            //add to a secondary master hough line list
        	listOfLines.add(new Line(start, end));
        	//Draw our hough lines
            Core.line(input, start, end, new Scalar(255,0,0), 3);
        }
        
        //At start, it starts with a very small number of lines
        //It increases the number of lines until it has enough lines
        if (listOfLines.size() < DESIRED_LINES && !hasHitDesiredLines) {
        	minimumHoughLineLength--;
        } else {
        	hasHitDesiredLines = true;
        }
        
        /////////////////////METHOD 1////////////////////////////////
        //Find the average 'y' value of the horizontal lines. Use this value to sort them into
        //lines belonging to the top and bottom line of the box
        double horizontalLinesAverageY = 0;
        for (Line l: listOfHorizontalLines) {
        	horizontalLinesAverageY += l.start.y + l.end.y;
        }
        horizontalLinesAverageY /= listOfHorizontalLines.size() * 2;

        //Find the average 'x' value of the vertical lines. Use this value to sort them into
        //lines belonging to the left and rigth line of the box
        //This is unused.
        double verticalLinesAverageX = 0;
        for (Line l: listOfVerticalLines) {
        	verticalLinesAverageX += l.start.y + l.end.y;
        }
        verticalLinesAverageX /= listOfVerticalLines.size() * 2;
        
        
        //Find the averages of the lower and upper lines
        double horizontalUpperX1 = 0;
        double horizontalUpperY1 = 0; 
        double horizontalUpperX2 = 0;
        double horizontalUpperY2 = 0;
        double numHorizontalUpper = 0;
        
        double horizontalLowerX1 = 0;
        double horizontalLowerY1 = 0; 
        double horizontalLowerX2 = 0;
        double horizontalLowerY2 = 0;
        double numHorizontalLower = 0;
        
        boolean flag0 = false;
        boolean flag1 = false;
        for (Line l : listOfHorizontalLines) {
        	if (l.start.y > horizontalLinesAverageY) {
        		horizontalUpperX1 += l.start.x;
        		horizontalUpperY1 += l.start.y;
        		horizontalUpperX2 += l.end.x;
        		horizontalUpperY2 += l.end.y;
        		
        		numHorizontalUpper++;
        		flag0 = true;
        	} else {
        		horizontalLowerX1 += l.start.x;
        		horizontalLowerY1 += l.start.y;
        		horizontalLowerX2 += l.end.x;
        		horizontalLowerY2 += l.end.y;
        		
        		numHorizontalLower++;
        		flag1 = true;
        	}
        	//if (flag0 && flag1) break;
        }
        
        horizontalUpperX1 /= numHorizontalUpper;
        horizontalUpperY1 /= numHorizontalUpper;
        horizontalUpperX2 /= numHorizontalUpper;
        horizontalUpperY2 /= numHorizontalUpper;
        
        horizontalLowerX1 /= numHorizontalLower;
        horizontalLowerY1 /= numHorizontalLower;
        horizontalLowerX2 /= numHorizontalLower;
        horizontalLowerY2 /= numHorizontalLower;
        
        //construct these into points
        Point horizontalUpperStart = new Point(horizontalUpperX1, horizontalUpperY1);
        Point horizontalUpperEnd = new Point(horizontalUpperX2, horizontalUpperY2);
        

        Point horizontalLowerStart = new Point(horizontalLowerX1, horizontalLowerY1);
        Point horizontalLowerEnd = new Point(horizontalLowerX2, horizontalLowerY2);
        
        Line horizontalUpperLine = new Line(horizontalUpperStart,horizontalUpperEnd);
        Line horizontalLowerLine = new Line(horizontalLowerStart,horizontalLowerEnd);

        //draw the lines to the image
        Core.line(input, horizontalUpperStart, horizontalUpperEnd, new Scalar(0,0,255), 3);
        Core.line(input, horizontalLowerStart, horizontalLowerEnd, new Scalar(0,0,255), 3);
        
        //Unused
        double VerticalUpperX1 = 0;
        double VerticalUpperY1 = 0; 
        double VerticalUpperX2 = 0;
        double VerticalUpperY2 = 0;
        double numVerticalUpper = 0;
        
        double VerticalLowerX1 = 0;
        double VerticalLowerY1 = 0; 
        double VerticalLowerX2 = 0;
        double VerticalLowerY2 = 0;
        double numVerticalLower = 0;
        
        
        for (Line l : listOfVerticalLines) {
        	if (l.start.y > verticalLinesAverageX) {
        		VerticalUpperX1 += l.start.x;
        		VerticalUpperY1 += l.start.y;
        		VerticalUpperX2 += l.end.x;
        		VerticalUpperY2 += l.end.y;
        		
        		numVerticalUpper++;
        		
        	} else {
        		VerticalLowerX1 += l.start.x;
        		VerticalLowerY1 += l.start.y;
        		VerticalLowerX2 += l.end.x;
        		VerticalLowerY2 += l.end.y;
        		
        		numVerticalLower++;
        	}
        }
        VerticalUpperX1 /= numVerticalUpper;
        VerticalUpperY1 /= numVerticalUpper;
        VerticalUpperX2 /= numVerticalUpper;
        VerticalUpperY2 /= numVerticalUpper;
        
        VerticalLowerX1 /= numVerticalLower;
        VerticalLowerY1 /= numVerticalLower;
        VerticalLowerX2 /= numVerticalLower;
        VerticalLowerY2 /= numVerticalLower;
        
        Point VerticalUpperStart = new Point(VerticalUpperX1, VerticalUpperY1);
        Point VerticalUpperEnd = new Point(VerticalUpperX2, VerticalUpperY2);

        Point VerticalLowerStart = new Point(VerticalLowerX1, VerticalLowerY1);
        Point VerticalLowerEnd = new Point(VerticalLowerX2, VerticalLowerY2);
        
        Line verticalUpperLine = new Line(VerticalUpperStart,VerticalUpperEnd);
        Line verticalLowerLine = new Line(VerticalLowerStart,VerticalLowerEnd);

        Core.line(input, horizontalUpperStart, horizontalLowerStart, new Scalar(0,255,255), 3);
        Core.line(input, horizontalUpperEnd, horizontalLowerEnd, new Scalar(0,255,255), 3);
        
        
        
        ////////////////////////////////METHOD 2//////////////////////////
        
        //List of corners. Corners occur where a vertical and horizontal line intersect.
        List<PointAndConnectingLines> corners = new ArrayList<PointAndConnectingLines>();
        for (Line hl: listOfHorizontalLines) {
        	for (Line vl: listOfVerticalLines) {
            	double distance = Math.sqrt(
            			(hl.start.x - vl.start.x) *
            			(hl.start.x - vl.start.x) + 
            			(hl.start.y - vl.start.y) *
            			(hl.start.y - vl.start.y));
            	if (distance < EPSILON) {
            		corners.add(new PointAndConnectingLines(
            				new Point((hl.start.x + vl.start.x) / 2, (hl.start.y + vl.start.y) / 2),
            				hl,
            				vl));
            		hl.addLinkedLine(vl);
            		vl.addLinkedLine(hl);
            	}
            	
            	distance = Math.sqrt(
            			(hl.start.x - vl.end.x) *
            			(hl.start.x - vl.end.x) + 
            			(hl.start.y - vl.end.y) *
            			(hl.start.y - vl.end.y));
            	if (distance < EPSILON) {
            		corners.add(new PointAndConnectingLines(
            				new Point((hl.start.x + vl.start.x) / 2, (hl.end.y + vl.end.y) / 2),
            				hl,
            				vl));
            		hl.addLinkedLine(vl);
            		vl.addLinkedLine(hl);
            	}
            	
            	distance = Math.sqrt(
            			(hl.end.x - vl.end.x) *
            			(hl.end.x - vl.end.x) + 
            			(hl.end.y - vl.end.y) *
            			(hl.end.y - vl.end.y));
            	if (distance < EPSILON) {
            		corners.add(new PointAndConnectingLines(
            				new Point((hl.end.x + vl.end.x) / 2, (hl.end.y + vl.end.y) / 2),
            				hl,
            				vl));
            		hl.addLinkedLine(vl);
            		vl.addLinkedLine(hl);
            	}
            	
            	distance = Math.sqrt(
            			(hl.end.x - vl.start.x) *
            			(hl.end.x - vl.start.x) + 
            			(hl.end.y - vl.start.y) *
            			(hl.end.y - vl.start.y));
            	if (distance < EPSILON) {
            		corners.add(new PointAndConnectingLines(
            				new Point((hl.end.x + vl.end.x) / 2, (hl.start.y + vl.start.y) / 2),
            				hl,
            				vl));
            		hl.addLinkedLine(vl);
            		vl.addLinkedLine(hl);
            	}
            }
        	
        }
        
        //draw all corners.
        for (PointAndConnectingLines p11: corners) Core.line(input, p11.p, p11.p, new Scalar(255,0,255), 7);

        //Filter corners. Find point per corner.
        List<PointAndConnectingLines> filteredCorners = new ArrayList<PointAndConnectingLines>(); //filtered bottom right corner points
        int i = 0;
        while (i < corners.size()) {
        	PointAndConnectingLines pacl = corners.get(i);
        	Point p = pacl.p;
        	boolean isNewPoint = true;
        	for (int j = 0; j < filteredCorners.size(); j++) {
        		PointAndConnectingLines p1 = filteredCorners.get(j);
        		Point p11 = p1.p;
        		double distance = Math.sqrt((p.x - p11.x) * (p.x - p11.x) + (p.y - p11.y) * (p.y - p11.y));
        		if (distance < EPSILON * 4) { //distance here is arbitrary at the moment
        			isNewPoint = false;
        		}
        	}
        	if (isNewPoint) {
        		filteredCorners.add(pacl);
        	}
        	
        	i++;
        }
        
        for (PointAndConnectingLines p: filteredCorners) {
            //Core.line(input, p.p, p.p, new Scalar(255,255,0), 7);
    	}
        
        
        if (filteredCorners.size() == 4) {
        	double averageX = 0;
        	double averageY = 0;
        	for (PointAndConnectingLines pacl: filteredCorners) {
        		Point p = pacl.p;
        		averageX += p.x;
        		averageY += p.y;
        	}
        	averageX /= 4;
        	averageY /= 4;
        	
        	Point p1 = null;
            Point p2 = null;
            Point p3 = null;
            Point p4 = null;
            
            
            
            for (PointAndConnectingLines pacl: filteredCorners) {
            	Point p = pacl.p;
        		if (averageX > p.x && averageY > p.y) p1 = p;
        		if (averageX > p.x && averageY < p.y) p2 = p;
        		if (averageX < p.x && averageY < p.y) p3 = p;
        		if (averageX < p.x && averageY > p.y) p4 = p;
        	}
            if (p1 == null || p2 == null || p3 == null || p4 == null) {
            	
            } else {

            	//Display area of rectangle in pixels squared.
            	//Uses irregular polygon area formula
                System.out.print("Area: ");
                System.out.println(Math.abs(
                		.5 * 
                		(p1.x * p2.y + p2.x + p3.y + p3.x * p4.y + p4.x * p1.y - 
                		 p2.x * p1.y - p3.x * p2.y - p4.x * p3.y - p1.x * p4.y)));
            }
        }
        
        List<Line> trulyVerticalLines = new ArrayList<Line>();
        Line longestTrulyVerticalLine = null;
        for (Line l: listOfVerticalLines) {
        	//decided to go with Donny's approach over vector math because of pixels being an integer unit.
        	double slope = (l.start.y - l.end.y) / (l.start.x - l.end.x);
        	if (slope > 10) {
        		trulyVerticalLines.add(l);
        		if (longestTrulyVerticalLine == null) longestTrulyVerticalLine = l;
        		double d1 = Math.sqrt((l.end.x - l.start.x) * (l.end.x - l.start.x) + (l.end.y - l.start.y) + (l.end.y - l.start.y));
        		double d2 = Math.sqrt((longestTrulyVerticalLine.end.x - longestTrulyVerticalLine.start.x) * (longestTrulyVerticalLine.end.x - longestTrulyVerticalLine.start.x) + (longestTrulyVerticalLine.end.y - longestTrulyVerticalLine.start.y) + (longestTrulyVerticalLine.end.y - longestTrulyVerticalLine.start.y));
        		if (d1 > d2) longestTrulyVerticalLine = l;
        		Core.line(rawImage, l.start, l.end, new Scalar(255,255,255), 10);
        	}
        }
        
        if (longestTrulyVerticalLine != null) {
        	Line l = longestTrulyVerticalLine;
        	System.out.println(Math.sqrt((l.end.x - l.start.x) * (l.end.x - l.start.x) + (l.end.y - l.start.y) + (l.end.y - l.start.y)));
        	//add calculation code here

        }
        
        System.out.println("Truly Vertical Lines:" + trulyVerticalLines.size());
        System.out.println("Time (ms): " + (System.currentTimeMillis() - startTime));
        kFrameMillis = (int) (System.currentTimeMillis() - startTime);
        long thisDraw = System.currentTimeMillis();
        
        
        if (thisDraw - lastDraw  > 200) {
        	kDidDraw = true;
        	lastDraw = thisDraw;
        	drawBin(calibrationWindow); //TODO MARKER
            draw(rawImage, calibrationWindow); //TODO MARKER
        } else {
        	kDidDraw = false;
        }


    //    Imgproc.blur(bin, detected_edges, new Size(3, 3));
    //    Imgproc.Canny( detected_edges, detected_edges, kCanMin, kCanMax);
/*        Imgproc.morphologyEx(detected_edges, detected_edges, Imgproc.MORPH_CLOSE, morphKernel,
            new Point(-1, -1), kHoleClosingIterations);*/
        // Find contours
            /*contours = Utils.findConvexContours(can);
            polygons = new ArrayList<Polygon>();
            for (MatOfPoint2f c : contours)
            {
                RotatedRect bounding = Imgproc.minAreaRect(c);
                double ratio = ((double) bounding.size.width)
                    / ((double) bounding.size.height);
            // height and width are switched so that the height of the bounding
            // box is the width of the goal
*//*            if (ratio > kMinRatio && ratio < kMaxRatio
                    && bounding.size.height > kMinWidth
                    && bounding.size.height < kMaxWidth) {*//*
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                Imgproc.approxPolyDP(c, approxCurve, 20, true);
                polygons.add(new Polygon(approxCurve));*/
/*            }*/
     /*   }*/

        /*Polygon square = null;
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
*//*                for (int i = 0; i < 4; i++) {
                    double dy = points[i].y - points[(i + 1) % 4].y;
                    double dx = points[i].x - points[(i + 1) % 4].x;
                    double slope = Double.MAX_VALUE;
                    if (dx != 0)
                        slope = Math.abs(dy / dx);

                    if (slope < kNearlyHorizontalSlope)
                        ++numNearlyHorizontal;
                    else if (slope > kNearlyVerticalSlope)
                        ++numNearlyVertical;
                }*//*

*//*                if (numNearlyHorizontal >= 1 && numNearlyVertical == 2) {
                    Core.polylines(rawImage, p.getMatOfPointsList(), true,
                            Colors.BLUE, 2);

                    if (p.getCenter().y < highest) { // y coordinates use 0 at
                        // the top
                        square = p;
                        highest = (int) p.getCenter().y;
                    }
                }*//*

                Core.polylines(rawImage, p.getMatOfPointsList(), true,
                        Colors.fromColor(Color.blue), 2);
            }
            else
            {
            Core.polylines(rawImage, p.getMatOfPointsList(), true,
                    Colors.fromColor(Color.yellow), 2);
            }
        }*/
/*
        can.setTo(Scalar.all(0));

        //detected_edges.copyTo(can);*/

        // Fill in any gaps using binary morphology


/*        lines = new Mat();
        Imgproc.HoughLinesP(can, lines, 1, Math.PI/180, 15, 40, 15 );

        detected_lines.setTo(Scalar.all(0));*/




        //check for lines intersecting this line within the frame
        /*for(int i = 0; i < lines.cols(); i++)
        {
            double[] line = lines.get(0,i);
            //int  lineslope = line.
        }
        int numLines = 0;
        for (int x = 0; x < lines.cols(); x++)
        {
            numLines = lines.cols();
            double[] vec = lines.get(0, x);
            double x1 = vec[0],
                   y1 = vec[1],
                   x2 = vec[2],
                   y2 = vec[3];
            Point start = new Point(x1, y1);
            Point end = new Point(x2, y2);

            double slope;
            double slopeOther;
            if((x2-x1)!=0)
            {
                slope = (y2-y1)/(x2-x1);
            }
            else
            {
                slope = Double.MAX_VALUE;
            }
            for(int j = x + 1; j < lines.cols(); j++)
            {
                double x1Other = vec[0],
                        y1Other = vec[1],
                        x2Other = vec[2],
                        y2Other = vec[3];
                Point startOther = new Point(x1, y1);
                Point endOther = new Point(x2, y2);
                if((x2-x1)!=0)
                {
                    slopeOther = (y2-y1)/(x2-x1);
                }
                else
                {
                    slopeOther = Double.MAX_VALUE;
                }

                if(slope!=0 && slopeOther)
                {

                }
            }

            Core.line(input, start, end, new Scalar(255,0,0), 3);
        }



/*

  hueFrame.show(hue);
        satFrame.show(sat);
        valFrame.show(val);
        oriFrame.show(input);*/

        //drawBin(calibrationWindow); //TODO MARKER
        //draw(rawImage, calibrationWindow);



/*		bin = hue;*/

/*        if (resultFrame != null)
            resultFrame.show(bin);*/

        /*// Fill in any gaps using binary morphology
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
*/
        /*Polygon square = null;
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
                            Colors.BLUE, 2);

                    if (p.getCenter().y < highest) { // y coordinates use 0 at
                        // the top
                        square = p;
                        highest = (int) p.getCenter().y;
                    }
                }

            }
            Core.polylines(rawImage, p.getMatOfPointsList(), true,
                    Colors.fromColor(Color.yellow), 2);
        }*/

        // Draw a crosshair
        /*if (m_debugMode)

            Core.line(rawImage, linePt1, linePt2, targetColor, 2);

        if (square != null) {
            double x = square.getCenter().x;
            x = (2 * (x / size.width)) - 1;
            double y = square.getCenter().y;
            y = -((2 * (y / size.height)) - 1);

            double azimuth = this.boundAngle0to360Degrees(x * kHorizontalFOVDeg
                    / 2.0 + heading - kShooterOffsetDeg);
//            double range = (kTopTargetHeightIn - kCameraHeightIn)
//                    / Math.tan((y * kVerticalFOVDeg / 2.0 + kCameraPitchDeg)
//                    * Math.PI / 180.0);
            int diffHeightPx = (int) Math.abs((kImgHeight / 2) - square.getCenter().y);
            double range = ((kImgHeight*(kTopTargetHeightIn - kCameraHeightIn)) / (2 * diffHeightPx)) / (Math.tan(kVerticalFovRad / 2));

            double angle = Math.toDegrees(Math.atan2(kTopTargetHeightIn
                    - kShooterHeight, range));

            Goal target = new Goal(square, range, azimuth, angle);

			System.out.println("Target found");
			System.out.println("x: " + x);
			System.out.println("y: " + y);
			System.out.println("azimuth: " + azimuth);
			System.out.println("range: " + range);
			System.out.println("angle deg: " + angle);
            Core.polylines(rawImage, square.getMatOfPointsList(), true,
                    targetColor, 7);
            Core.putText(rawImage, String.valueOf(Math.round(range)), //Math.round(angle)),
                    square.getCenter(), Core.FONT_HERSHEY_COMPLEX, 1,
                    Colors.BLUE);
*//*			if (resultFrame != null)
				resultFrame.show(rawImage);*//*

            draw(rawImage, calibrationWindow);
            return target;
        } else {
        	draw(rawImage, calibrationWindow);
            return null;
        }*/
    //draw(rawImage, calibrationWindow); //TODO MARKER
    return null;
    }

	private double boundAngle0to360Degrees(double angle) {
		return angle % 360;
	}

    private void draw (Mat input, JFrame calibrationWindow)
    {
        VisionLabel hueLabel = (VisionLabel)calibrationWindow.getContentPane().getComponent(6);
        VisionLabel satLabel = (VisionLabel)calibrationWindow.getContentPane().getComponent(7);
        VisionLabel valLabel = (VisionLabel)calibrationWindow.getContentPane().getComponent(8);
        VisionLabel cannyLabel = (VisionLabel)calibrationWindow.getContentPane().getComponent(16);
        VisionLabel inputLabel = (VisionLabel)calibrationWindow.getContentPane().getComponent(17);

        Size hueSizeScaled = new Size(hueLabel.getMaximumSize().getWidth(),
                hueLabel.getMaximumSize().getHeight());
        Size satSizeScaled = new Size(satLabel.getMaximumSize().getWidth(),
                satLabel.getMaximumSize().getHeight());
        Size valSizeScaled = new Size(valLabel.getMaximumSize().getWidth(),
                valLabel.getMaximumSize().getHeight());
        Size cannySizeScaled = new Size(cannyLabel.getMaximumSize().getWidth(),
                cannyLabel.getMaximumSize().getHeight());
        Size inputSizeScaled = new Size(inputLabel.getMaximumSize().getWidth(),
                inputLabel.getMaximumSize().getHeight());

        if ((inputSizeScaled.height < hue.size().height) || (inputSizeScaled.width < hue.size().width))
        {
            Mat hueScaled = new Mat(hueSizeScaled, CvType.CV_8UC1);
            Mat satScaled = new Mat(satSizeScaled, CvType.CV_8UC1);
            Mat valScaled = new Mat(valSizeScaled, CvType.CV_8UC1);
            Mat cannyScaled = new Mat(cannySizeScaled, CvType.CV_8UC3);
            Mat inputScaled = new Mat(inputSizeScaled, CvType.CV_8UC3);

            Imgproc.resize(hue, hueScaled, hueScaled.size(), 0, 0, Imgproc.INTER_AREA);
            Imgproc.resize(sat, satScaled, satScaled.size(), 0, 0, Imgproc.INTER_AREA);
            Imgproc.resize(val, valScaled, valScaled.size(), 0, 0, Imgproc.INTER_AREA);
            Imgproc.resize(can, cannyScaled, cannyScaled.size(), 0, 0, Imgproc.INTER_AREA);
            Imgproc.resize(input, inputScaled, inputScaled.size(), 0, 0, Imgproc.INTER_AREA);

            hueLabel.show(hueScaled);
            satLabel.show(satScaled);
            valLabel.show(valScaled);
            cannyLabel.show(cannyScaled);
            inputLabel.show(inputScaled);
        }
        else
        {
            hueLabel.show(hue);
            satLabel.show(sat);
            valLabel.show(val);

            inputLabel.show(can);
            inputLabel.show(input);
        }
    }
    
    private void drawBin(JFrame calibrationWindow){
        VisionLabel resultLabel = (VisionLabel)calibrationWindow.getContentPane().getComponent(15);
        Size resultSizeScaled = new Size(resultLabel.getMaximumSize().getWidth(),
                resultLabel.getMaximumSize().getHeight());
        if ((resultSizeScaled.height < bin.size().height) || (resultSizeScaled.width < bin.size().width))
        {
            Mat resultScaled = new Mat(resultSizeScaled, CvType.CV_8UC1);

            Imgproc.resize(bin, resultScaled, resultScaled.size(), 0, 0, Imgproc.INTER_AREA);
            resultLabel.show(resultScaled);
        }
        else
        {
            resultLabel.show(bin);
        }
    }
}