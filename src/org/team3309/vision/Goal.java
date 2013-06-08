package org.team3309.vision;

import org.opencv.core.Point;

public class Goal {

	private Polygon p;
	private double range, azimuth, angle;

	public Goal(Polygon p, double range, double azimuth, double angle) {
		this.p = p;
		this.range = range;
		this.azimuth = azimuth;
		this.angle = angle;
	}

	/**
	 * Get the center of the target in pixels
	 * 
	 * @return
	 */
	public Point getCenter() {
		return p.getCenter();
	}

	/**
	 * Get the horizontal distance to the target in inches
	 * 
	 * @return
	 */
	public double getRange() {
		return range;
	}

	/**
	 * Get the azimuth to the target in degrees. This is the number of degrees
	 * the robot needs to turn in order to be pointed at the target
	 * 
	 * @return
	 */
	public double getAzimuth() {
		return azimuth;
	}

	/**
	 * Get the angle of elevation to the target. If the arm matches this angle
	 * exactly and runs at a sufficient speed to make the frisbee fly straight,
	 * it should go in.
	 * 
	 * @return
	 */
	public double getAngle() {
		return angle;
	}

}
