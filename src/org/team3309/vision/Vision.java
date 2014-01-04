package org.team3309.vision;

import java.awt.*;
import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;

import slider.RangeSlider;

public class Vision implements ChangeListener {

	private static final boolean demoMode = false;
	private static final boolean demoVideo = false;
	private static final boolean useMin = true;

	static {
		// load the opencv native library
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

    RangeSlider hue = new RangeSlider();
    RangeSlider sat = new RangeSlider();
    RangeSlider val = new RangeSlider();
    RangeSlider can = new RangeSlider();
    RangeSlider shape = new RangeSlider();

	public Vision() {
/*      JFrame hmin = new JFrame("hmin");
        JFrame smin = new JFrame("smin");
        JFrame vmin = new JFrame("vmin");
        JFrame hmax = new JFrame("hmax");
        JFrame smax = new JFrame("smax");
        JFrame vmax = new JFrame("vmax");*/
        JFrame hsvCalibration = new JFrame("HSV Calibration");
        hsvCalibration.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();

        GraphicsConfiguration screenConfig = GraphicsEnvironment.getLocalGraphicsEnvironment()
                                                .getScreenDevices()[0].getDefaultConfiguration();
        Insets screenInsets = Toolkit.getDefaultToolkit().getScreenInsets(screenConfig);

        hsvCalibration.setSize((int)screenSize.getWidth() - screenInsets.left - screenInsets.right,
                               (int)screenSize.getHeight() - screenInsets.bottom - screenInsets.top);

        hsvCalibration.setLocation(screenInsets.top, screenInsets.left);
        //hsvCalibration.setResizable(false);
        hsvCalibration.setExtendedState(JFrame.MAXIMIZED_BOTH);

        int windowWidth = hsvCalibration.getWidth();
        int windowHeight = hsvCalibration.getHeight();

        hsvCalibration.setLayout(new FlowLayout(FlowLayout.CENTER, (int)(windowWidth*0.02), (int)(windowHeight*0.02)));
        
        VisionLabel hueFrame = new VisionLabel();
        hueFrame.setMaximumSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.35)));
        VisionLabel satFrame = new VisionLabel();
        satFrame.setMaximumSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.35)));
        VisionLabel valFrame = new VisionLabel();
        valFrame.setMaximumSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.35)));
        VisionLabel cannyFrame = new VisionLabel();
        cannyFrame.setMaximumSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.35)));
        VisionLabel resFrame = new VisionLabel();
        resFrame.setMaximumSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.35)));
        VisionLabel oriFrame = new VisionLabel();
        oriFrame.setMaximumSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.35)));
        
/*      hue.setMaximum(255);
        sat.setMaximum(255);
        val.setMaximum(255);*/
        hue.setMaximum(255);
        hue.setUpperValue(GoalTracker.kHueMax);
        hue.setValue(GoalTracker.kHueMin);
        hue.setPreferredSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.02)));

        sat.setMaximum(255);
        sat.setUpperValue(GoalTracker.kSatMax);
        sat.setValue(GoalTracker.kSatMin);
        sat.setPreferredSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.02)));

        val.setMaximum(255);
        val.setUpperValue(GoalTracker.kValMax);
        val.setValue(GoalTracker.kValMin);
        val.setPreferredSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.02)));

        can.setMaximum(255);
        can.setUpperValue(GoalTracker.kCanMax);
        can.setValue(GoalTracker.kCanMin);
        can.setPreferredSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.02)));

        shape.setMaximum(255);
        shape.setUpperValue(GoalTracker.kCanMax);
        shape.setValue(GoalTracker.kCanMin);
        shape.setPreferredSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.02)));


        JLabel hueLabel = new JLabel("Hue", SwingConstants.CENTER);
        hueLabel.setPreferredSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.02)));

        JLabel satLabel = new JLabel("Saturation", SwingConstants.CENTER);
        satLabel.setPreferredSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.02)));

        JLabel valLabel = new JLabel("Value", SwingConstants.CENTER);
        valLabel.setPreferredSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.02)));

        JLabel resultLabel = new JLabel("Result", SwingConstants.CENTER);
        resultLabel.setPreferredSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.02)));

        JLabel cannyLabel = new JLabel("After Canny", SwingConstants.CENTER);
        cannyLabel.setPreferredSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.02)));

        JLabel inputLabel = new JLabel("Input", SwingConstants.CENTER);
        inputLabel.setPreferredSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.02)));

        JLabel blankLabel = new JLabel(" ", SwingConstants.CENTER);
        blankLabel.setPreferredSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.02)));

        JLabel blankLabel2 = new JLabel(" ", SwingConstants.CENTER);
        blankLabel2.setPreferredSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.02)));

        JLabel blankLabel3 = new JLabel(" ", SwingConstants.CENTER);
        blankLabel3.setPreferredSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.02)));

        JLabel blankLabel4 = new JLabel(" ", SwingConstants.CENTER);
        blankLabel4.setPreferredSize(new Dimension((int)(windowWidth*0.3), (int)(windowHeight*0.02)));


        hsvCalibration.add(hueLabel, 0);
        hsvCalibration.add(satLabel, 1);
        hsvCalibration.add(valLabel, 2);

        hsvCalibration.add(hue, 3);
        hsvCalibration.add(sat, 4);
        hsvCalibration.add(val, 5);

        hsvCalibration.add(hueFrame, 6);
        hsvCalibration.add(satFrame, 7);
        hsvCalibration.add(valFrame, 8);

        hsvCalibration.add(resultLabel, 9);
        hsvCalibration.add(cannyLabel, 10);
        hsvCalibration.add(inputLabel, 11);


        hsvCalibration.add(blankLabel, 12);
        hsvCalibration.add(can, 13);
        hsvCalibration.add(blankLabel2, 14);

        hsvCalibration.add(resFrame, 15);
        hsvCalibration.add(cannyFrame, 16);
        hsvCalibration.add(oriFrame, 17);

        hsvCalibration.add(blankLabel3, 18);
        hsvCalibration.add(shape, 19);
        hsvCalibration.add(blankLabel4, 20);
        
/*      hmin.setVisible(true);
        smin.setVisible(true);
        vmin.setVisible(true);
        hmax.setVisible(true);          `````````````````````````````````````````````````````````````
        smax.setVisible(true);
        vmax.setVisible(true);*/
        hsvCalibration.setVisible(true);

/*        huemin.addChangeListener(this);
        satmin.addChangeListener(this);
        valmin.addChangeListener(this);
        huemax.addChangeListener(this);
        satmax.addChangeListener(this);
        valmax.addChangeListener(this);*/
        hue.addChangeListener(this);
        sat.addChangeListener(this);
        val.addChangeListener(this);
        can.addChangeListener(this);
        shape.addChangeListener(this);

		GoalTracker tracker = new GoalTracker();

		//VisionFrame frame = new VisionFrame("result");

		VideoCapture cam = new VideoCapture(0);

		int width = (int) cam.get(Highgui.CV_CAP_PROP_FRAME_WIDTH);
		int height = (int) cam.get(Highgui.CV_CAP_PROP_FRAME_HEIGHT);

		long lastTime = 0;
		Mat rawImage = new Mat(width, height, CvType.CV_32FC3);
		while (cam.grab()) {
			cam.retrieve(rawImage);
//			tracker.processImage(rawImage, frame);
            tracker.processImage(rawImage, hsvCalibration);
			if(System.currentTimeMillis() - lastTime > 1000){
/*				if(useMin){
                    System.out.println("hue:"+GoalTracker.kHueMin);
                    System.out.println("sat:"+GoalTracker.kSatMin);
                    System.out.println("val:"+GoalTracker.kValMin);
                }
                else{
                    System.out.println("hue:"+GoalTracker.kHueMax);
                    System.out.println("sat:"+GoalTracker.kSatMax);
                    System.out.println("val:"+GoalTracker.kValMax);
                }*/
                System.out.println("hue:"+GoalTracker.kHueMin);
                System.out.println("sat:"+GoalTracker.kSatMin);
                System.out.println("val:"+GoalTracker.kValMin);

                System.out.println("hue:"+GoalTracker.kHueMax);
                System.out.println("sat:"+GoalTracker.kSatMax);
                System.out.println("val:"+GoalTracker.kValMax);

				System.out.println("\n\n");


				lastTime = System.currentTimeMillis();
			}
		}
		cam.release();
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
		//GoalTracker tracker = new GoalTracker();
		
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
        /*if (e.getSource().equals(huemin)) {
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
        }*/
        if (e.getSource().equals(hue)) {
            GoalTracker.kHueMin = hue.getValue();
            GoalTracker.kHueMax = hue.getUpperValue();
        }
        if (e.getSource().equals(sat)) {
            GoalTracker.kSatMin = sat.getValue();
            GoalTracker.kSatMax = sat.getUpperValue();
        }
        if (e.getSource().equals(val)) {
            GoalTracker.kValMin = val.getValue();
            GoalTracker.kValMax = val.getUpperValue();
        }
        if (e.getSource().equals(can)) {
            GoalTracker.kCanMin = can.getValue();
            GoalTracker.kCanMax = can.getUpperValue();
        }
        if (e.getSource().equals(shape)) {
            GoalTracker.kShapeMin = shape.getValue();
            GoalTracker.kShapeMax = shape.getUpperValue();
        }
	}
}
