package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.JoystickConstants;

public class Joystick {
    private final int m_port;

    public Joystick(int port){
        m_port = port;
    }

    public double leftX(){
        return new XboxController(m_port).getRawAxis(JoystickConstants.kXStick1);
    }

    public double leftY(){
        return new XboxController(m_port).getRawAxis(JoystickConstants.kYStick1);
    }

    public double rightX(){
        return new XboxController(m_port).getRawAxis(JoystickConstants.kXStick2);
    }

    public double rightY(){
        return new XboxController(m_port).getRawAxis(JoystickConstants.kYStick2);
    }

    public double leftTrigger(){
        return new XboxController(m_port).getRawAxis(JoystickConstants.kLeftTrigger);
    }

    public double rightTrigger(){
        return new XboxController(m_port).getRawAxis(JoystickConstants.kRightTrigger);
    }

    public JoystickButton a(){
        return new JoystickButton(new XboxController(m_port), 1);
    }

    public JoystickButton x(){
        return new JoystickButton(new XboxController(m_port), 2);
    }

    public JoystickButton b(){
        return new JoystickButton(new XboxController(m_port), 3);
    }

    public JoystickButton y(){
        return new JoystickButton(new XboxController(m_port), 4);
    }

    public JoystickButton leftBumper(){
        return new JoystickButton(new XboxController(m_port), 5);
    }

    public JoystickButton rightBumper(){
        return new JoystickButton(new XboxController(m_port), 6);
    }

    public JoystickButton back(){
        return new JoystickButton(new XboxController(m_port), 7);
    }

    public JoystickButton start(){
        return new JoystickButton(new XboxController(m_port), 8);
    }

    public JoystickButton leftStick(){
        return new JoystickButton(new XboxController(m_port), 9);
    }

    public JoystickButton rightStick(){
        return new JoystickButton(new XboxController(m_port), 10);
    }

    public POVButton dpadUp(){
        return new POVButton(new XboxController(m_port), 0);
    }

    public POVButton dpadRight(){
        return new POVButton(new XboxController(m_port), 90);
    }

    public POVButton dpadLeft(){
        return new POVButton(new XboxController(m_port), 270);
    }

    public POVButton dpadDown(){
        return new POVButton(new XboxController(m_port), 180);
    }

}