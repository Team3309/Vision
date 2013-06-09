package org.team3309.vision;

import java.awt.BorderLayout;
import java.io.ByteArrayInputStream;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JSlider;
import javax.swing.event.ChangeListener;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.highgui.Highgui;

public class VisionFrame extends JFrame{

	private JLabel label = null;
	private JSlider slider = null;

	public VisionFrame(String name) {
		super(name);
		label = new JLabel();
		add(label);
	}

	public void show(Mat img){
		MatOfByte memory = new MatOfByte();
		Highgui.imencode(".png", img, memory);
		try {
			label.setIcon(new ImageIcon(ImageIO.read(new ByteArrayInputStream(memory.toArray()))));
		} catch (IOException e) {
			e.printStackTrace();
		}
		if(!isVisible()){
			pack();
			setVisible(true);
		}
	}
	
	public void addSlider(int min, int max, ChangeListener listener){
		if(slider == null){
			slider = new JSlider();
			this.add(slider, BorderLayout.SOUTH);
		}
		slider.addChangeListener(listener);
		slider.setMinimum(min);
		slider.setMaximum(max);
	}
}
