package org.team3309.vision;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;

public class Vision {

	private static final boolean demoMode = true;
	private static final boolean demoVideo = false;

	static {
		// load the opencv native library
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		GoalTracker tracker = new GoalTracker();
		if (demoMode) {
			demoMain(tracker);
		}
	}

	public static void demoMain(GoalTracker tracker) {
		if (!demoVideo) {
			// Load the image
			Mat rawImage = null;
			rawImage = Highgui
					.imread("/home/vmagro/workspace/JavaTest/sample1.png");
			
			VisionFrame frame = new VisionFrame("result");
			tracker.processImage(rawImage, frame);
		} else {
			File imagesFolder = new File("/home/vmagro/Downloads/987vid/images");
			ArrayList<String> files = new ArrayList<String>(
					Arrays.asList(imagesFolder.list()));
			Collections.sort(files);
			Mat rawImage;
			VisionFrame frame = new VisionFrame("Video");
			for (String file : files) {
				rawImage = Highgui
						.imread("/home/vmagro/Downloads/987vid/images/" + file);
				tracker.processImage(rawImage, frame);
				try {
					Thread.sleep(50); // 10fps
				} catch (InterruptedException e) {
					e.printStackTrace();
				}

			}
		}
	}

}
