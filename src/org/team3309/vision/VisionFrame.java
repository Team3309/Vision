package org.team3309.vision;

import java.io.ByteArrayInputStream;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.highgui.Highgui;

public class VisionFrame {

	private JFrame frame = null;
	private JLabel label = null;

	public VisionFrame(String name) {
		frame = new JFrame(name);
		label = new JLabel();
		frame.setContentPane(label);
	}

	public void show(Mat img){
		MatOfByte memory = new MatOfByte();
		Highgui.imencode(".png", img, memory);
		try {
			label.setIcon(new ImageIcon(ImageIO.read(new ByteArrayInputStream(memory.toArray()))));
		} catch (IOException e) {
			e.printStackTrace();
		}
		if(!frame.isVisible()){
			frame.pack();
			frame.setVisible(true);
		}
	}
}
