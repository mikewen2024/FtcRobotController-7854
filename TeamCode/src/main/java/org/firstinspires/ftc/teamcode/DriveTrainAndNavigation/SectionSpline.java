package org.firstinspires.ftc.teamcode.DriveTrainAndNavigation;

import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.SplinePath;

import java.util.ArrayList;

public class SectionSpline{
    /* 
    Points represented by:
    1. x and y
    2. x and y of guide points, "weight" distance from roots
    
    Every 3 points is assumed to be a root, w/ 1st point and last point roots
    Exception: after first and before last roots, only 1 guide point 
    */
	
	// Meant to work with other Spline builder (guide spline path)
	// Note: derivatives adjustable
   public double [][] rootsAndGuides;
   public double [] weights; // For each root
   public double [][] tangents; // For each non-endpoint root
   public double [] coords; // For returning value
   public SplinePath[] sections; // For getting coordinates from each section
	public double [] tangent = {0.0, 0.0};
   
   public SectionSpline(double [][] roots, double weight) { // Weight raw-translated to distance
	   
	   this.rootsAndGuides = new double [roots.length * 3 - 4][];
	   // allocating memory for each point
	   for(int i = 0; i < this.rootsAndGuides.length; i++) {
		   this.rootsAndGuides [i] = new double [2];
	   }
	   this.weights = new double [roots.length];
	   this.tangents = new double [roots.length - 2][];
	   for(int i = 0; i < this.tangents.length; i++) {
		   this.tangents [i] = new double [2];
	   }
	   this.coords = new double [2];
	   this.sections = new SplinePath [roots.length - 1];
	   
	   // sections adjacent to first and last points are 1st order
	   // All other sections are 2nd order
	   
	   // Copy over roots
	   // First and last points
	   this.rootsAndGuides [0][0] = roots [0][0];
	   this.rootsAndGuides [0][1] = roots [0][1];
	   
	   this.rootsAndGuides [this.rootsAndGuides.length - 1][0] = roots [roots.length - 1][0];
	   this.rootsAndGuides [this.rootsAndGuides.length - 1][1] = roots [roots.length - 1][1];
	   
	   // Copy over other roots
	   for(int i = 1; i < roots.length - 1; i++) {
		   this.rootsAndGuides [i*3 - 1][0] = roots [i][0];
		   this.rootsAndGuides [i*3 - 1][1] = roots [i][1];
	   }
			   
	   
	   // For each non-endpoint root, calculate noramlized tangent vector at that point based on adjacent points' delta
	   for(int i = 1; i < roots.length - 1; i++) {
		   this.tangents [i-1] [0] = roots[i+1][0] - roots[i-1][0];
		   this.tangents [i-1] [1] = roots[i+1][1] - roots[i-1][1];
		   double magnitude = Math.sqrt(this.tangents [i-1][0]*this.tangents [i-1][0] + this.tangents [i-1][1]*this.tangents [i-1][1]);
		   this.tangents [i-1] [0] /= magnitude;
		   this.tangents [i-1] [1] /= magnitude;
		   
		   // Point before
		   this.rootsAndGuides [i*3 - 2][0] = -weight * this.tangents [i-1][0] + roots[i][0];
		   this.rootsAndGuides [i*3 - 2][1] = -weight * this.tangents [i-1][1] + roots[i][1];
		   
		   // Point after
		   this.rootsAndGuides [i*3][0] = weight * this.tangents [i-1][0] + roots[i][0];
		   this.rootsAndGuides [i*3][1] = weight * this.tangents [i-1][1] + roots[i][1];
	   }
	   
	   // Generate spline objects for each part, first and last generated seperately since first order
	   // First section
	   this.sections [0] = new SplinePath(new double [][] {this.rootsAndGuides[0], this.rootsAndGuides[1], this.rootsAndGuides[2]});
	   // Last section
	   this.sections [this.sections.length - 1] = new SplinePath(new double [][] {this.rootsAndGuides[this.rootsAndGuides.length - 3], this.rootsAndGuides[this.rootsAndGuides.length - 2], this.rootsAndGuides[this.rootsAndGuides.length - 1]});
	   // Other sections
	   for(int i = 1; i < this.sections.length - 1; i++) {
		   this.sections [i] = new SplinePath(new double [][] {this.rootsAndGuides[i*3 - 1], this.rootsAndGuides[i*3], this.rootsAndGuides[i*3 + 1], this.rootsAndGuides[i*3 + 2]});
	   }
   }
   
   public double [] getState(double t) { // Input double [0.0, 1.0], get point b/w start and end of full path
	   double interRootInterval = 1.0 / ((double) this.sections.length);
	   int sectionNumber = (int) Math.floor(t / interRootInterval);// Which spline we are getting the value from, index in sections array
	   double localTimeValue = t % interRootInterval;
	   localTimeValue = (double) (localTimeValue / interRootInterval);
	   
	   return this.sections[sectionNumber].getCoords(localTimeValue);
   }

   public double getTangentAngle(double t){ // Returns angle
	   this.tangent [0] = this.getState(t + 0.001)[0] - this.getState(t - 0.001)[0];
	   this.tangent [1] = this.getState(t + 0.001)[1] - this.getState(t - 0.001)[1];

	   if(this.tangent [0] > 0.0){
		   if(this.tangent[0] == 0){
			   this.tangent[0] += 0.01;
		   }
		   return Math.atan(this.tangent [1] / this.tangent [0]);
	   }else{
		   if(this.tangent[0] == 0){
			   this.tangent[0] += 0.01;
		   }
		   return Math.PI + Math.atan(this.tangent [1] / this.tangent [0]);
	   }
   }
}