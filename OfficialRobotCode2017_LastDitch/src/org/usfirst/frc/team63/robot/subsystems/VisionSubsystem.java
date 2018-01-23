
package org.usfirst.frc.team63.robot.subsystems;

import org.usfirst.frc.team63.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class VisionSubsystem extends Subsystem {
	private static final double kEpsilon = 1E-9;
	private static final double kTargetHWRatio = 2.5;
	
	private NetworkTable table;
	private double[] defaultValue = new double[0];
	
	private double[] last_areas = new double[0];
	private double[] last_centerXs = new double[0];
	private double[] last_centerYs = new double[0];
	private double[] last_heights = new double[0];
	private double[] last_widths = new double[0];
	
	private double[] curr_areas;
	private double[] curr_centerXs;
	private double[] curr_centerYs;
	private double[] curr_heights;
	private double[] curr_widths;
	
	private double[] hw_ratios;
	private int index1 = 0;
	private int index2 = 1;
	
	private boolean has_valid_data = false;
	private boolean first_run = true;

	public double target_dist = 0.0;
	public double target_shift = 0.0;
	public double target_skew = 0.0;
	
	public VisionSubsystem()
	{
		table = NetworkTable.getTable("GRIP/myContoursReport");
	}
	
	public synchronized boolean updateContourData()
	{
		boolean bUpdated = false;
		
		curr_areas = table.getNumberArray("area", defaultValue);
		curr_centerXs = table.getNumberArray("centerX", defaultValue);
		curr_centerYs = table.getNumberArray("centerY", defaultValue);
		curr_heights = table.getNumberArray("height", defaultValue);
		curr_widths = table.getNumberArray("width", defaultValue);
		
		if(first_run)
		{
			first_run = false;
			bUpdated = processTargetData();
		}
		else if(curr_areas.length != last_areas.length ||
		   curr_centerXs.length != last_centerXs.length ||
	       curr_centerYs.length != last_centerYs.length ||
		   curr_heights.length != last_heights.length ||
		   curr_widths.length != last_widths.length)
		{
			bUpdated = processTargetData();
		}
		else
		{
			for(int i = 0; i < curr_areas.length; i++)
			{
				if(Math.abs(curr_areas[i] - last_areas[i]) > kEpsilon ||
				   Math.abs(curr_centerXs[i] - last_centerXs[i]) > kEpsilon ||
				   Math.abs(curr_centerYs[i] - last_centerYs[i]) > kEpsilon ||
				   Math.abs(curr_heights[i] - last_heights[i]) > kEpsilon ||
				   Math.abs(curr_widths[i] - last_widths[i]) > kEpsilon)
				{
					bUpdated = processTargetData();
					break;
				}
			}
		}
		
		last_areas = curr_areas;
		last_centerXs = curr_centerXs;
		last_centerYs = curr_centerYs;
		last_heights = curr_heights;
		last_widths = curr_widths;
		
		return bUpdated;
	}
	
	public synchronized double getTargetCenter()
	{
		if(curr_centerXs.length < 2)
		{
			return -1;
		}
		return (curr_centerXs[index1] + curr_centerXs[index2]) / 2.0;
	}
	
	public synchronized double getTargetArea()
	{
		if(curr_areas.length < 2)
		{
			return -1;
		}
		return (curr_areas[index1] + curr_areas[index2]) / 2.0;
	}
	
	public synchronized void outputToSmartDashboard()
	{
		SmartDashboard.putNumber("target_area", getTargetArea());
		SmartDashboard.putNumber("target_center", getTargetCenter());
		SmartDashboard.putNumber("target_dist", target_dist);
		SmartDashboard.putNumber("target_shift", target_shift);
		SmartDashboard.putNumber("target_skew", target_skew);
		SmartDashboard.putNumber("vision_index1", index1);
		SmartDashboard.putNumber("vision_index2", index2);
	}
		
	private synchronized boolean processTargetData()
	{
		has_valid_data = false;
		int length_check = curr_areas.length; 
		if(length_check < 2)
		{
			return has_valid_data;
		}
		
		if(curr_areas.length != length_check || curr_centerXs.length != length_check || curr_centerYs.length != length_check || curr_heights.length != length_check || curr_widths.length != length_check)
		{
			return has_valid_data;
		}
		
		has_valid_data = true;

		hw_ratios = new double[curr_areas.length];
		for(int i = 0; i < curr_areas.length; i++)
		{
			hw_ratios[i] = ((curr_heights[i] / curr_widths[i]) / kTargetHWRatio);
		}
		
		if(hw_ratios.length == 2)		
		{
			index1 = 0;
			index2 = 1;			
		}
		else
		{
			//Find the best score
			double best_delta = 1000;
			int best_index = 0;
			for(int i = 0; i < hw_ratios.length; i++)
			{
				if(Math.abs(1.0 - hw_ratios[i]) < best_delta)
				{
					best_delta = Math.abs(1.0 - hw_ratios[i]);
					best_index = i;
				}
			}

			//Find the second best score
			double second_best_delta = 1000;
			int second_best_index = 0;
			for(int i = 0; i < hw_ratios.length; i++)
			{
				if(i != best_index)
				{
					if(Math.abs(1.0 - hw_ratios[i]) < second_best_delta)
					{
						second_best_delta = Math.abs(1.0 - hw_ratios[i]);
						second_best_index = i;
					}
				}
			}
			
			index1 = best_index;
			index2 = second_best_index;
		}
		
		target_dist = targetAreaToDistance(getTargetArea());
		target_shift = targetCenterToShift(getTargetCenter());
		
		if(curr_areas[index1] > curr_areas[index2])
		{
			target_skew = targetAreasToSkew();
		}
		else
		{
			target_skew = targetAreasToSkew();
		}
		
		outputToSmartDashboard();
		
		return has_valid_data;
	}
			
	private synchronized double targetAreaToDistance(double area)
	{
		return Math.min(Math.max(1883.1*Math.pow(area,-0.489), 0.5), 48);
	}
	
	private synchronized double targetCenterToShift(double center)
	{		
		double a = -(center - RobotMap.kCameraResWidth / 2.0) / (RobotMap.kCameraResWidth / 2.0);
		double w = a * target_dist * Math.tan(RobotMap.kCameraFieldOfViewTheta);
		System.out.println("center: " + center + ", a: " + a + ", w: " + w);
		return Math.min(Math.max(w, -36), 36);
	}
	
	private synchronized double targetAreasToSkew()
	{
//		double small_area = 0;
//		double small_area_x = 0;
//		double big_area = 0;
//		double big_area_x = 0;
//		
//		if(curr_areas[0] < curr_areas[1])
//		{	
//			small_area = curr_areas[0];
//			small_area_x = curr_centerXs[0];
//			big_area = curr_areas[1];
//			big_area_x = curr_centerXs[1];
//		}
//		else
//		{
//			small_area = curr_areas[1];
//			small_area_x = curr_centerXs[1];
//			big_area = curr_areas[0];
//			big_area_x = curr_centerXs[0];
//		}
//		
//		double ratio = small_area/big_area;
//		double y = -100.0*ratio + 100;
//		
//		if(small_area_x < big_area_x)
//		{
//			return Math.max(-y, -30);
//		}
//		else
//		{
//			return Math.min(y, 30);
//		}		
		
		return 0;
	}
	
    public void initDefaultCommand()
    {
    	
    }
}


