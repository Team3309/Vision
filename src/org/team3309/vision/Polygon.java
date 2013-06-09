package org.team3309.vision;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

public class Polygon {
	
	private RotatedRect boundingBox;
	private Point[] points;
	
	public Polygon(MatOfPoint2f points){
		this.points = points.toArray();
		boundingBox = Imgproc.minAreaRect(points);
	}
	
	public int getNumVertices(){
		return points.length;
	}
	
	public boolean isConvex(){
		return true; //its obviously convex, we got it from a convex hull operation
	}
	
	public Point[] getPoints(){
		return points;
	}
	
	public List<MatOfPoint> getMatOfPointsList(){
		ArrayList<MatOfPoint> list = new ArrayList<MatOfPoint>();
		MatOfPoint mat = new MatOfPoint();
		mat.alloc(points.length);
		mat.fromArray(points);
		list.add(mat);
		return list;
	}
	
	public MatOfPoint2f getMatOfPoint2f(){
		MatOfPoint2f mat = new MatOfPoint2f();
		mat.alloc(points.length);
		mat.fromArray(points);
		return mat;
	}
	
	public Point getCenter(){
		return boundingBox.center;
	}
	
}
