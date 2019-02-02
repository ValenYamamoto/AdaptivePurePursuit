package pathCreation;

public class Vector {
	
	private double x;
	private double y;
	
	/**
	 * Contructor for Vector with two points
	 * @param a starting point
	 * @param b second point
	 */
	public Vector(Point a, Point b) {
		this.x = b.getX() - a.getX();
		this.y = b.getY() - a.getY();
	}
	
	public Vector(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public double magnitude() {
		return Math.sqrt(x*x + y*y);
	}
	
	public Vector multiplyBy(double m) {
		return new Vector(x * m, y * m);
	}
	
	public Vector normalize() {
		double mag = magnitude();
		Vector normal = new Vector(x, y);
		normal.setX(x/mag);
		normal.setY(y/mag);
		return normal;
	}
	
	public double getX() {
		return x;
	}
	
	public double getY() {
		return y;
	}
	
	public void setX(double x) {
		this.x = x;
	}
	
	public void setY(double y) {
		this.y = y;
	}
	
	public static double dot(Vector a, Vector b) {
		return a.getX() * b.getX() + a.getY() * b.getY();
	}
}
