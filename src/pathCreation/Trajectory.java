package pathCreation;

import java.util.ArrayList;

public class Trajectory {
	
	private Point[] waypoints;
	private double maxVel, maxAccel, k;
	private double samplingRate = 1.0/50;
	private double spacing = 0.5;
	private double lastOutput = 0;
	private Waypoint[] path;
	
	public Trajectory(Point[] waypoints, double maxVel, double maxAccel, double k) {
		this.waypoints = waypoints;
		this.maxVel = maxVel;
		this.maxAccel = maxAccel;
		this.k = k;
	}
	
	public void generatePath() {
		ArrayList<Point> path = injectPoints();
		path = smoother(path, 0.25, 0.75, 0.95);
		Waypoint[] finalPath = new Waypoint[path.size()];
		for(int i = 0; i < path.size(); i ++) {
			finalPath[i] = new Waypoint(path.get(i).getX(), path.get(i).getY(), 0, 0, 0);
		}
		
		finalPath[0].setCurvature(0);
		finalPath[0].setDistance(0);
		finalPath[0].setTargetVel(maxVel);
		
		finalPath[finalPath.length-1].setCurvature(0);

		for(int i = 1; i < finalPath.length - 1; i++) {
			finalPath[i].setCurvature(calculateCurvature(finalPath[i], finalPath[i-1],
					finalPath[i+1]));
			finalPath[i].setDistance(finalPath[i-1].getDistance() + 
					distanceFormula(finalPath[i], finalPath[i-1]));
			
			double v = Math.min(k/finalPath[i].getCurvature(), maxVel);
			finalPath[i].setTargetVel(v);
		}
		finalPath[finalPath.length-1].setDistance(finalPath[finalPath.length-2].getDistance() + 
					distanceFormula(finalPath[finalPath.length-1], finalPath[finalPath.length-2]));
		finalPath[finalPath.length - 1].setTargetVel(0);
		
		for (int i = finalPath.length - 2; i >= 0; i --) {
			double d = distanceFormula(finalPath[i+1], finalPath[i]);
			double v = Math.sqrt(Math.pow(finalPath[i+1].getTargetVel(), 2) + 2 * maxAccel * d);
			finalPath[i].setTargetVel(Math.min(v, finalPath[i].getTargetVel()));
		}
		this.path = finalPath;
	}
	
	public Waypoint[] getTraj() {
		return path;
	}
	
	public ArrayList<Point> injectPoints() {
		ArrayList<Point> newPath = new ArrayList<Point>();
		
		for(int i = 0; i < waypoints.length - 1; i++) {
			Vector vector = new Vector(waypoints[i], waypoints[i+1]);
			double numPointsThatFit = Math.ceil(vector.magnitude()/spacing);
			vector = vector.normalize();
			vector = vector.multiplyBy(spacing);
			System.out.println(numPointsThatFit);
			
			for (int j = 0; j < numPointsThatFit; j++) {
				double x = waypoints[i].getX() + vector.getX() * j;
				double y = waypoints[i].getY() + vector.getY() * j;
				newPath.add(new Point(x, y));	
			}
		}
		newPath.add(waypoints[waypoints.length-1]);
		
		return newPath;
	}
	/**
	 * 
	 * @param path
	 * @param a
	 * @param b
	 * @param tolerance
	 * @return arrayList of points
	 */
	public ArrayList<Point> smoother(ArrayList<Point> path, double a, double b, double tolerance) {
		ArrayList<Point> newPath = path;
		
		double change = tolerance;
		
		double origX, origY, newX, newY, lastY, lastX, nextX, nextY, 
		augX, augY, deltaX, deltaY;
		
		while(change >= tolerance) {
			change = 0;
			for(int i = 1; i < path.size() - 1; i++) {
				origX = path.get(i).getX();
				origY = path.get(i).getY();
				
				newX = newPath.get(i).getX();
				newY = newPath.get(i).getY();
				
				lastX = newPath.get(i-1).getX();
				lastY = newPath.get(i-1).getY();
				
				nextX = newPath.get(i+1).getX();
				nextY = newPath.get(i+1).getY();
				
				augX = newX + a*(origX-newX) + b*(lastX + nextX - (2*newX));
				augY = newY + a*(origY-newY) + b*(lastY + nextY - (2*newY));
//				System.out.printf("x: %f%ny: %f%n", a*(origX-newX) + b*(lastX + nextX - (2*newX)),(origY-newY) + b*(lastY + nextY - (2*newY)));
				newPath.set(i, new Point(augX, augY));
				
				deltaX = Math.abs(augX - newX);
				deltaY = Math.abs(augY - newY);
//				System.out.printf("i = %d%n  ^x: %f%n  ^y: %f%n", i, deltaX, deltaY);
//				System.out.printf("i = %d, origX: %.2f, origY: %.2f, newX: %.4f, newY: %.4f, lastX: %.2f, lastY: %.2f, nextX: %.2f, nextY: %.2f, augX: %.4f, augY: %.4f, Delta X: %.4f, Delta Y: %.4f%n", 
//						i, origX, origY, newX, newY, lastX, lastY, nextX, nextY, augX, augY, deltaX, deltaY);
				
				change += deltaX + deltaY;
//				System.out.printf("  change: %f%n", change);
			}
		}
		
		return newPath;
	}
	
	/**
	 * 
	 * @param p point you want curvature for
	 * @param q last point
	 * @param r next point
	 * @return curvature at point p
	 */
	public double calculateCurvature(Point p, Point q, Point r) {
		if (p.getX() == q.getX()) {
			p.setX(p.getX() + 0.001);
		}
		
		double k1 = (p.getX() * p.getX() + p.getY() * p.getY() -
				q.getX() * q.getX() - q.getY() * q.getY()) / 2 * (p.getX() - q.getX());
		double k2 = (p.getY() - q.getY())/(p.getX() - q.getX());
		double b = (q.getX() * q.getX() - 2 * q.getX() * k1 + q.getY() * q.getY() -
				r.getX() * r.getX() + 2 * r.getX() * k1 - r.getY() * r.getY()) /
				2 * (r.getX() * k2 - r.getY() + q.getY() - q.getX() * k2);
		double a = k1 - k2 * b;
		double rad = Math.sqrt(Math.pow(p.getX() - a, 2) + Math.pow(p.getY() - a, 2));
		
		double y = 1.0/rad;
		
		return y;
	}
	
	/**
	 * x2 - x1
	 * @param p1 x2 
	 * @param p2 x1
	 * @return distance between points
	 */
	public double distanceFormula(Point p1, Point p2) {
		double a = Math.pow(p1.getX() - p2.getX(), 2);
		double b = Math.pow(p1.getY() - p2.getY(), 2);
		return Math.sqrt(a + b);
	}
	
	public double rateLimiter(double input) {
		double maxChange = samplingRate * maxAccel;
		double output = lastOutput + constrain(input - lastOutput, - maxChange, maxChange);
		return output;
	}
	
	public double constrain(double x, double up, double down) {
		if (x > up) {
			x = up;
		} else if (x < down){
			x = down;
		}
		return x;
	}
}
