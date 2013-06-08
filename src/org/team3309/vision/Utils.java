package org.team3309.vision;

import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

public class Utils {
	
	public static Mat[] split(Mat src, Mat... dst){
		ArrayList<Mat> dstList = new ArrayList<Mat>();
		for(Mat mat : dst)
			dstList.add(mat);
		Core.split(src, dstList);
		return dstList.toArray(new Mat[0]);
	}
	
	public static MatOfPoint2f[] findConvexContours(Mat image) {
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		Imgproc.findContours(image, contours, new Mat(), Imgproc.RETR_EXTERNAL,
				Imgproc.CHAIN_APPROX_TC89_KCOS);
		ArrayList<MatOfPoint2f> results = new ArrayList<MatOfPoint2f>();
		for (MatOfPoint contour : contours) {
			MatOfInt hull = new MatOfInt();
			Imgproc.convexHull(contour, hull);
			ArrayList<Point> hullPoints = new ArrayList<Point>();
			int rows = hull.rows();
			for (int i = 0; i < rows; i++) {
				int index = (int) hull.get(i, 0)[0];
				double[] coords = contour.get(index, 0);
				Point p = new Point(coords[0], coords[1]);
				hullPoints.add(p);
			}
			MatOfPoint2f mat = new MatOfPoint2f();
			mat.alloc(hullPoints.size());
			mat.fromList(hullPoints);
			results.add(mat);
		}

		return results.toArray(new MatOfPoint2f[0]);
	}

}
