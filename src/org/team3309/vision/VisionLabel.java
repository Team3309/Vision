package org.team3309.vision;

import java.io.ByteArrayInputStream;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JLabel;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.highgui.Highgui;

import java.awt.Graphics;

public class VisionLabel extends JLabel{

    public void show(Mat img){
        MatOfByte memory = new MatOfByte();
        Highgui.imencode(".png", img, memory);
        try {
            setIcon(new ImageIcon(ImageIO.read(new ByteArrayInputStream(memory.toArray()))));
        } catch (IOException e) {
            e.printStackTrace();
        }
        if(!isVisible()){
            setVisible(true);
        }
    }
}
