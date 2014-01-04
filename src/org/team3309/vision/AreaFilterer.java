package org.team3309.vision;

import java.util.List;

import org.opencv.core.Point;

public class AreaFilterer {
	
	public static void reportArea(List<Point> allCorners, List<Line> allLines) {
		
		
		
	}
	
	public static void printArea(Point p1, Point p2, Point p3, Point p4) {
		System.out.print("Area: ");
        System.out.println(Math.abs(
        		.5 * 
        		(p1.x * p2.y + p2.x + p3.y + p3.x * p4.y + p4.x * p1.y - 
        		 p2.x * p1.y - p3.x * p2.y - p4.x * p3.y - p1.x * p4.y)));
	}

}
