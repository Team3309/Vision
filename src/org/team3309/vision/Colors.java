package org.team3309.vision;

import java.awt.Color;

import org.opencv.core.Scalar;

public class Colors {
	
	public static final Scalar RED = fromColor(Color.red);
	public static final Scalar GREEN = fromColor(Color.green);
	public static final Scalar BLUE = fromColor(Color.blue);
	
	public static Scalar fromColor(Color c){
		return new Scalar(c.getBlue(), c.getGreen(), c.getRed());
	}

}
