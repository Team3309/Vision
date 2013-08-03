package org.team3309.vision;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import javax.swing.JFrame;
import javax.swing.JSlider;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;

public class Vision implements ChangeListener {

	private static final boolean demoMode = false;
	private static final boolean demoVideo = false;
	private static final boolean useMin = true;

	static {
		// load the opencv native library
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

/*	JSlider hue = new JSlider();
	JSlider sat = new JSlider();
	JSlider val = new JSlider();*/
    JSlider huemin = new JSlider();
    JSlider satmin = new JSlider();
    JSlider valmin = new JSlider();
    JSlider huemax = new JSlider();
    JSlider satmax = new JSlider();
    JSlider valmax = new JSlider();

	public Vision() {
/*		JFrame h = new JFrame("h");
        JFrame s = new JFrame("s");
        JFrame v = new JFrame("v");*/
        JFrame hmin = new JFrame("hmin");
        JFrame smin = new JFrame("smin");
        JFrame vmin = new JFrame("vmin");
        JFrame hmax = new JFrame("hmax");
        JFrame smax = new JFrame("smax");
        JFrame vmax = new JFrame("vmax");
/*		hue.setMaximum(255);
		sat.setMaximum(255);
		val.setMaximum(255);*/
        huemin.setMaximum(255);
        satmin.setMaximum(255);
        valmin.setMaximum(255);
        huemax.setMaximum(255);
        satmax.setMaximum(255);
        valmax.setMaximum(255);
/*		h.setSize(255, 50);
		s.setSize(255, 50);
		v.setSize(255, 50);*/
        hmin.setSize(255, 50);
        smin.setSize(255, 50);
        vmin.setSize(255, 50);
        hmax.setSize(255, 50);
        smax.setSize(255, 50);
        vmax.setSize(255, 50);
/*		h.add(hue);
        s.add(sat);
        v.add(val);*/
        hmin.add(huemin);
        smin.add(satmin);
        vmin.add(valmin);
        hmax.add(huemax);
        smax.add(satmax);
        vmax.add(valmax);
/*		h.setVisible(true);
		s.setVisible(true);
		v.setVisible(true);*/
        hmin.setVisible(true);
        smin.setVisible(true);
        vmin.setVisible(true);
        hmax.setVisible(true);
        smax.setVisible(true);
        vmax.setVisible(true);

/*		hue.addChangeListener(this);
        sat.addChangeListener(this);
        val.addChangeListener(this);*/
        huemin.addChangeListener(this);
        satmin.addChangeListener(this);
        valmin.addChangeListener(this);
        huemax.addChangeListener(this);
        satmax.addChangeListener(this);
        valmax.addChangeListener(this);

		GoalTracker tracker = new GoalTracker();

		VisionFrame frame = new VisionFrame("result");

		VideoCapture cam = new VideoCapture(0);

		int width = (int) cam.get(Highgui.CV_CAP_PROP_FRAME_WIDTH);
		int height = (int) cam.get(Highgui.CV_CAP_PROP_FRAME_HEIGHT);

		long lastTime = 0;
		Mat rawImage = new Mat(width, height, CvType.CV_32FC3);
		while (cam.grab()) {
			cam.retrieve(rawImage);
			tracker.processImage(rawImage, frame);
			if(System.currentTimeMillis() - lastTime > 1000){
				if(useMin){
					System.out.println("hue:"+GoalTracker.kHueMin);
					System.out.println("sat:"+GoalTracker.kSatMin);
					System.out.println("val:"+GoalTracker.kValMin);
				}
				else{
					System.out.println("hue:"+GoalTracker.kHueMax);
					System.out.println("sat:"+GoalTracker.kSatMax);
					System.out.println("val:"+GoalTracker.kValMax);
				}
				System.out.println("\n\n");


				lastTime = System.currentTimeMillis();
			}
		}
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		if (demoMode) {
			GoalTracker tracker = new GoalTracker();
			demoMain(tracker);
		} else {
			new Vision();
		}
	}

	public static void demoMain(GoalTracker tracker) {
		if (!demoVideo) {
			// Load the image
			Mat rawImage = null;
			rawImage = Highgui.imread("goal_sample_987.png");

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

	@Override
	public void stateChanged(ChangeEvent e) {
/*		if (e.getSource().equals(hue)) {
			if(useMin)
				GoalTracker.kHueMin = hue.getValue();
			else
				GoalTracker.kHueMax = hue.getValue();
		}
		if (e.getSource().equals(sat)) {
			if(useMin)
				GoalTracker.kSatMin = sat.getValue();
			else
				GoalTracker.kSatMax = sat.getValue();
		}
		if (e.getSource().equals(val)) {
			if(useMin)
				GoalTracker.kValMin = val.getValue();
			else
				GoalTracker.kValMax = val.getValue();
		}*/
        if (e.getSource().equals(huemin)) {
            GoalTracker.kHueMin = huemin.getValue();
        }
        if (e.getSource().equals(satmin)) {
            GoalTracker.kSatMin = satmin.getValue();
        }
        if (e.getSource().equals(valmin)) {
            GoalTracker.kValMin = valmin.getValue();
        }
        if (e.getSource().equals(huemax)) {
            GoalTracker.kHueMax = huemax.getValue();
        }
        if (e.getSource().equals(satmax)) {
            GoalTracker.kSatMax = satmax.getValue();
        }
        if (e.getSource().equals(valmax)) {
            GoalTracker.kValMax = valmax.getValue();
        }
	}

}
