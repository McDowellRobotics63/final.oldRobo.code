package org.usfirst.frc.team63.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

public class XboxDPadButton extends Button {	
	
    public enum DPAD_BUTTON {
        DPAD_UP, DPAD_RIGHT, DPAD_DOWN, DPAD_LEFT;
    }
    
    private Joystick xbox_controller = null;
    private DPAD_BUTTON dpad_type = DPAD_BUTTON.DPAD_UP;

	public XboxDPadButton(Joystick controller, DPAD_BUTTON type)
	{
		xbox_controller = controller;
		dpad_type = type;
	}
	
	public boolean get() {
		if(dpad_type == DPAD_BUTTON.DPAD_UP)
		{
			return xbox_controller.getPOV() == 0;
		}
		else if(dpad_type == DPAD_BUTTON.DPAD_RIGHT)
		{
			return xbox_controller.getPOV() == 90;
		}
		else if(dpad_type == DPAD_BUTTON.DPAD_DOWN)
		{
			return xbox_controller.getPOV() == 180;
		}
		else if(dpad_type == DPAD_BUTTON.DPAD_LEFT)
		{
			return xbox_controller.getPOV() == 270;
		}
		else
		{
			return false;
		}
	}
}
