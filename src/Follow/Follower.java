package Follow;

import pathCreation.Point;
import pathCreation.Trajectory;
import pathCreation.Vector;
import pathCreation.Waypoint;

public class Follower {

	private Trajectory traj;
	private Waypoint[] path;
	private Point robotLocation;
	private double heading;
	private double rightVel;
	private double leftVel;
	private double lastVel;
	private double lookaheadDistance;
	private int lastIndex;
	private double trackWidth;
	private double kp;
	private double kv;
	private double ka;
	
	public Follower(Trajectory traj, double lookaheadDistance, double trackWidth) {
		this.traj = traj;
		this.path = this.traj.getTraj();
		this.lookaheadDistance = lookaheadDistance;
		this.lastIndex = -1;
		this.trackWidth = trackWidth;
		this.robotLocation = new Point(0,0);
		this.heading = 0;
		this.rightVel = 0;
		this.leftVel = 0;
		this.kp = 0;
		this.ka = 0;
		this.kv = 0;
	}
	
	public void getData() {
		// get robot Location
		// get heading
		// get current velocity
	}
	
	public void setConstants(double kp, double ka, double kv) {
		this.kp = kp;
		this.ka = ka;
		this.kv = kv;
	}
	
	public Waypoint findClosestPoint(double rx, double ry) {
		double minDistance = traj.distanceFormula(path[0], path[path.length-1]);
		Waypoint closest = path[0];
		Point robotLoc = new Point(rx, ry);
		for (int i = 0; i < path.length; i ++) {
			double d = Math.abs(traj.distanceFormula(robotLoc, path[i]));
			
			if(d < minDistance) {
				minDistance = d;
				closest = path[i];
			}
		}
		return closest;
	}
	
	public Point findLookaheadPoint(double rx, double ry, double heading) {
		for (int i = 0; i < path.length - 1; i ++) {
			Point start = path[i];
			Point end = path[i+1];
			Point robotLoc = new Point(rx, ry);
			Vector roboRay = new Vector(start, end);
			Vector f = new Vector(robotLoc, end);
			
			double a = Vector.dot(roboRay, roboRay);
			double b = 2 * Vector.dot(f, roboRay);
			double c = Vector.dot(f, f) - lookaheadDistance * lookaheadDistance;
			double discriminant = b*b - 4*a*c;
			
			if(discriminant >= 0) { // if intersection
				discriminant = Math.sqrt(discriminant);
				double t1 = (-b - discriminant)/(2*a);
				double t2 = (-b + discriminant)/(2*a);
				
				if ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)) { // TODO: check index
					double addx = t1 * roboRay.getX();
					double addy = t1 * roboRay.getY();
					
					return new Point(start.getX() + addx, start.getY() + addy);
				}
			}
		}
		return path[path.length-1];
	}
	
	public double getCurvatureToPoint(Point robotLoc, Point lookahead, double heading) {
		double a = -Math.tan(heading); // TODO: convert to radians
		double b = 1;
		double c = Math.tan(heading) * robotLoc.getX() - robotLoc.getY(); // TODO: convert to radians
		
		double d = Math.abs(a * lookahead.getX() + b * lookahead.getY() + c)/ (Math.sqrt(a*a + b*b));
		
		double L = traj.distanceFormula(lookahead, robotLoc);
		return 2*d/(L*L);
	}
	
	public double getSide(Point robotLoc, Point lookahead, double heading) {
		double bx = robotLoc.getX() + Math.cos(heading); // TODO: Radians
		double by = robotLoc.getY() + Math.sin(heading);
		
		double side = Math.signum(Math.sin(heading) * (lookahead.getX() - robotLoc.getX()) 
				- Math.cos(heading) * (lookahead.getY() - robotLoc.getY()));
		return side;
	}
	
	public double getRightWheelSpeed(double maxVel, double curv) {
		return (maxVel * (2 + curv * trackWidth)) / 2.0;
	}
	
	public double getLeftWheelSpeed(double maxVel, double curv) {
		return (maxVel * (2 - curv * trackWidth)) / 2.0;
	}
	
	public void drive() {
		getData();
		double x = robotLocation.getX();
		double y = robotLocation.getY();
		
		double targetVel = findClosestPoint(x, y).getTargetVel();
		targetVel = traj.rateLimiter(targetVel);
		double accel = targetVel - lastVel;
		
		Point lookahead = findLookaheadPoint(x, y, heading);
		double curv = getCurvatureToPoint(robotLocation, lookahead, heading);
		double side = getSide(robotLocation, lookahead, heading);
		
		double signedCurvature = curv * side;
		
		double rightTarget = getRightWheelSpeed(targetVel, signedCurvature);
		double leftTarget = getLeftWheelSpeed(targetVel, signedCurvature);
		
		double rightError = rightVel - rightTarget;
		double leftError = leftVel - leftTarget;
		
		double rightFeedback = rightError * kp;
		double leftFeedback = leftError * kp;
		
		double rightFeedForward = ka* accel + kv * rightTarget;
		double leftFeedForward = ka * accel + kv * leftTarget;
		
		double leftPower = leftFeedback + leftFeedForward;
		double rightPower = rightFeedback + rightFeedForward;
	}
}
