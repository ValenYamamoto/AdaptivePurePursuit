package pathCreation;

import java.util.ArrayList;

public class Trajectory {
	
	private Point[] waypoints;
	
	public Trajectory(Point[] waypoints) {
		this.waypoints = waypoints;
	}
	
	public ArrayList<Point> injectPoints() {
		double spacing = 6;
		ArrayList<Point> newPath = new ArrayList<Point>();
		
		for(int i = 0; i < waypoints.length - 1; i++) {
			Vector vector = new Vector(waypoints[i+1], waypoints[i]);
			double numPointsThatFit = Math.ceil(vector.magnitude()/spacing);
			
			for (int j = 0; i < numPointsThatFit; i++) {
				double x = waypoints[i].getX() + vector.getX() * j;
				double y = waypoints[i].getY() + vector.getY() * j;
				newPath.add(new Point(x, y));	
			}
		}
		newPath.add(waypoints[waypoints.length-1]);
		
		return newPath;
	}
	
	public ArrayList<Point> smoother(ArrayList<Point> path, double a, double b, double tolerance) {
		ArrayList<Point> newPath = path;
		
		double change = tolerance;
		
		while(change >= tolerance) {
			change = 0;
			for(int i = 1; i < path.length-1; i++) {
				for (int j)
			}
		}
	}
}
