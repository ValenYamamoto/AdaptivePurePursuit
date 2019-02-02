package main;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

import pathCreation.Point;
import pathCreation.Trajectory;
import pathCreation.Waypoint;

public class Main {
	
	public static void main(String[] args) {
		Point[] points = { new Point(0,0),
				new Point(3, 0), 
				new Point(10, 3)};
		Trajectory traj = new Trajectory(points, 15, 5, 5);
		
//		System.out.println(traj.injectPoints());
//		ArrayList<Point> inject = traj.injectPoints();
		
		System.out.println("Starting Path Generation");
		traj.generatePath();
		System.out.println("Finished Path Generation");
//		Waypoint way = new Waypoint(9, 9, 0, 0, 0);
//		System.out.println(way);
		Waypoint[] path = traj.getTraj();
		for (int i = 0; i < path.length; i ++) {
			System.out.println(path[i]);
		}
		
		String outFileLog = "C:/Users/Valen/Documents";
		PrintWriter writer;
		
		try {
			outFileLog = new SimpleDateFormat("'/Users/Valen/Documents/logfile'.MMddhhmm'.csv'").format(new Date());
			System.out.printf("outputfile is: %s\n",  outFileLog);
		
			File file = new File(outFileLog);
			writer = new PrintWriter(outFileLog, "UTF-8");
			
			for (int i = 0; i < path.length; i++) {
				writer.printf(path[i].toFile());
			}
			writer.close();
		} catch (FileNotFoundException | UnsupportedEncodingException e) {
			
		}
		
		
		
	}
}
