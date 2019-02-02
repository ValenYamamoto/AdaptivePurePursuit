package pathCreation;

public class Waypoint extends Point{

	private double distance, targetVel, curv;
	
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
	
	public String toString() {
		return String.format("X: " + super.getX() + ",   Y: " + super.getY() + ",   Dist: " + distance + ",    v: " + 
				targetVel + ",    curv: " + curv);
	}
	
	public String toFile() {
		return String.format("%f, %f, %f, %f, %f%n", super.getX(), super.getY(), distance, targetVel, curv);
	}
}
