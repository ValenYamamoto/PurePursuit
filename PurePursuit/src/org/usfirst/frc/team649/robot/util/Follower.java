package org.usfirst.frc.team649.robot.util;

public class Follower {
	
	private double kp;
	private double ki;
	private double kd;
	
	private int encoderOffset;
	private int ticksPerRevolution;
	private double wheelCircumference;
	
	public Follower() {

	}
	
	public void configurePID(double kp, double ki, double kd){
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
	}
	
	public void configureEncoders(int encoderOffset, int ticksPerRevolution, double wheelDiameter) {
		this.encoderOffset = encoderOffset;
		this.ticksPerRevolution = ticksPerRevolution;
		wheelCircumference = Math.PI * wheelDiameter;
	}
}
