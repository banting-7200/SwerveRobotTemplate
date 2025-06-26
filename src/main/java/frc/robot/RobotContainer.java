package frc.robot;

import edu.wpi.first.wpilibj.event.EventLoop;

public class RobotContainer {

    private static RobotContainer instance;
    private EventLoop loop = new EventLoop();
    
    public static RobotContainer getInstance() {
        if (instance == null) instance = new RobotContainer();
        return instance;
      }



    public void teleopPeriodic(){
        loop.poll();
    }






}
