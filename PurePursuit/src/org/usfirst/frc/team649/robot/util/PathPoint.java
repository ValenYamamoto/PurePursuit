package org.usfirst.frc.team649.robot.util;

public class PathPoint {
	private double x;
	private double y;
	private double heading;
	private double curvature;
	private double error;
	
	public PathPoint() {
		
	}
	public PathPoint(double x, double y, double heading, double curvature, double error) {
		this.x = x;
		this.y = y;
		this.heading = heading;
		this.curvature = curvature;
		this.error = error;
	}
	
	public double getX() {
		return x;
	}
	
	public void setX(double x) {
		this.x = x;
	}
	
	public double getY() {
		return y;
	}
	
	public void sety(double y) {
		this.y = y;
	}
	
	public double getHeading() {
		return heading;
	}
	
	public void setHeading(double heading) {
		this.heading = heading;
	}
	
	public double getCurvature() {
		return curvature;
	}
	
	public void setCurvature(double curvature) {
		this.curvature = curvature;
	}
	
	public double getError() {
		return error;
	}
	
	public void setError(double error) {
		this.error = error;
	}
}
