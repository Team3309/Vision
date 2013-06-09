package org.team3309.vision;

import org.opencv.core.Point;

public class Circle {
	
	private Point center;
	private int radius;
	
	public Circle(Point center, int radius){
		this.center = center;
		this.radius = radius;
	}
	
	public Point getCenter(){
		return center;
	}
	
	public int getRadius(){
		return radius;
	}

}
