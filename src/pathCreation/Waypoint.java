package pathCreation;

public class Waypoint extends Point{

	private double x, y, distance, targetVel, curv;
	
	public Waypoint(double x, double y, double distance, double targetVel,
			double curv) {
		super(x, y);
		this.distance = distance;
		this.targetVel = targetVel;
		this.curv = curv;
	}
	
	public void setDistance(double d) {
		distance = d;
	}
	
	public double getDistance() {
		return distance;
	}
	
	public void setTargetVel(double v) {
		targetVel = v;
	}
	
	public double getTargetVel() {
		return targetVel;
	}
	
	public void setCurvature(double c) {
		curv = c;
	}
	
	public double getCurvature() {
		return curv;
	}
}
