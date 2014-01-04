package org.team3309.vision;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Point;

public class Line {
	
	public Point start;
	public Point end;
	
	List<Line> linkedLines;
	
	public Line(Point p, Point p1) {
		this.start = p;
		this.end = p1;
		
		linkedLines = new ArrayList<Line>();
	}
	
	public void addLinkedLine(Line vl) {
		linkedLines.add(vl);
	}

	
	public void doAttemptRectangleMake(List<Point> masterList, int i) {
		boolean shouldAddStart = true;
		boolean shouldAddEnd = true;
		for (Point p: masterList) {
			double distance = distance(p, start);
			double distance1 = distance(p, end);
			if (distance < 20) shouldAddStart = false;
			if (distance1 < 20) shouldAddEnd = false;
			
		}
		if (shouldAddStart) masterList.add(start);
		if (shouldAddEnd) masterList.add(end);
		if (i < 1) {
			for (Line l: linkedLines) {
				l.doAttemptRectangleMake(masterList, i + 1);
			}
		}
	}

	private double distance(Point p, Point p1) {
		return Math.sqrt((p1.x - p.x) * (p1.x - p.x) + (p1.y + p.y) * (p1.y + p.y));
	}

}
