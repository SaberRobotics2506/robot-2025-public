package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{
    //Creating LED Object
    private final AddressableLED m_led = new AddressableLED(0);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(14);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("kitbot");

    //Anything in this section is from last year and is yet to be implemented
    private int yellow_dot = 0;
    private int brown_dot = 0;
    private boolean warning = false;
    public boolean pathfinding = false;
    public boolean operatorPathfinding = false;
    public boolean boomPathfinding = false;
    public int pulseOffset = 0;
    public double matchTime;
    public boolean robotOrient;
    public double rainbow_dot;
    public boolean bottomLimitDot;
    private Extake m_extake;

    public DoublePublisher pathfindingCommand;
    public double isPathfinding;

    //Constructor
    public LED(Extake m_extake){
        this.m_extake = m_extake; //instantiate extake system
        m_led.setLength(m_ledBuffer.getLength()); //set how long the led is
        m_led.setData(m_ledBuffer); //sets the led output dat
        m_led.start(); //start leds
        set(0, 0, 0); //starts LEDS as off

        // Set the default command to turn the strip off, otherwise the last colors written by
        // the last command to run will continue to be displayed.
        // Note: Other default patterns could be used instead
        //setDefaultCommand(runPattern(LEDPattern.solid(Color.kRed)).withName("Off"));

        pathfindingCommand = table.getDoubleTopic("pathfindingCommand").publish();
    }


    /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(m_ledBuffer));
    }
  
    //POPULAR COLORS AND THEIR RGB VALUES
    // Orange
    // set(255,165,0);

    //Yellow
    // set(255,255,0);

    //Green
    // set(0,255,0);

    //Blue
    // set(0,0,255);

    //indigo
    // set(75,0,130);

    //violet
    // set(127,0,255);

    //this method is a work in progress
    //should change LED Strip to color red
    public void setRed(){
        LEDPattern red = LEDPattern.solid(Color.kRed);

        // Apply the LED pattern to the data buffer
        red.applyTo(m_ledBuffer);
    }


    //periodic runs every few ms and is a method that is always running
    //This is where the colors are changing.
    @Override
    public void periodic() {
        if(!DriverStation.isEnabled()){
            idle(); //if the robot is disabled, the colors will be based off the alliance
        }if(DriverStation.isEStopped()){
            setPulseOrange();    
        } else if (pathfinding) {
            setPulseBlue();
        } else {
            matchTime = Timer.getMatchTime();
            if(matchTime>20||matchTime<18){ //Checks if the time is between 18 and 20 seconds
                if(!m_extake.getIrSensor()){
                    set(0, 255, 0); //Turns Green if there is a game piece in the robot
                }else if(m_extake.getIrSensor()){
                    set(255, 0, 0); //Turns Red if there is no game piece in the robot
                }
            } else {
                if(warning){
                    set(255, 50, 50);
                } else {
                    set(0, 0, 0);
                }
                warning = !warning; //switches on and off repeatedly to flash
            }
        }

        m_led.setData(m_ledBuffer); //Have to do this for the color to actually apply to the LED

        // Telemetry
        if (pathfinding) {
            isPathfinding = 1.0;
        } else {
            isPathfinding = 0.0;
        }

        pathfindingCommand.set(isPathfinding);
    }

    //Sets each in strip to a color
    public void set(int red, int green, int blue){
        for (int i=0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, red, green, blue); //Sets each individual LED to the desired Color
        }
    }

    public void setPulseRed(){
        int intensity = 255 - pulseOffset;
            for (int i = 0; i < m_ledBuffer.getLength() / 2 + 1; i++) {
                m_ledBuffer.setRGB(i, (intensity + 25*i) % 255, 0, 0);
                m_ledBuffer.setRGB(m_ledBuffer.getLength()-1-i, (intensity + 25*i) % 255, 0, 0);
            }
            pulseOffset = (pulseOffset + 15) % 255;
    }

    public void setPulseBlue(){
        int intensity = 255 - pulseOffset;
            for (int i = 0; i < m_ledBuffer.getLength() / 2 + 1; i++) {
                m_ledBuffer.setRGB(i, 0, 0, (intensity + 25*i) % 255);
                m_ledBuffer.setRGB(m_ledBuffer.getLength()-1-i, 0, 0, (intensity + 25*i) % 255);
            }
            pulseOffset = (pulseOffset + 15) % 255;
    }

    //this is for when the robot is E stoped
    public void setPulseOrange(){
        int intensity = 255 - pulseOffset;
            for (int i = 0; i < m_ledBuffer.getLength() / 2 + 1; i++) {
                m_ledBuffer.setRGB(i, (intensity + 25*i) % 255, (intensity + 25*1) % 165, 0);
                m_ledBuffer.setRGB(m_ledBuffer.getLength()-1-i, (intensity + 25*i) % 255, 0, 0);
            }
            pulseOffset = (pulseOffset + 15) % 255;
    }

    public void yellow_dot(){
        yellow_dot++;
        for(int i=0;i<m_ledBuffer.getLength(); i++){
            if(yellow_dot%28==i){
                m_ledBuffer.setRGB(i, 255, 255, 0);
            } else {
                m_ledBuffer.setRGB(i, 0, 0, 255);
            }
        }
    }

      public void purple_dot(){
        yellow_dot++;
        for(int i=0;i<m_ledBuffer.getLength(); i++){
            if(yellow_dot%28==i){
                m_ledBuffer.setRGB(i, 255, 255, 0);
            } else {
                m_ledBuffer.setRGB(i, 160, 32, 240);
            }
        }
    }

    //Old Rainbow pattern code
    public void rainbow_dot(){
        rainbow_dot++;

        for(int i=0;i<m_ledBuffer.getLength(); i++){
            if(i%7==0){
                m_ledBuffer.setRGB(i, 255,0,0);
            }else if(i%7==1){
                m_ledBuffer.setRGB(i, 255, 165, 0);
            }else if(i%7==2){
                m_ledBuffer.setRGB(i, 255, 255, 0);
            }else if(i%7==3){
                m_ledBuffer.setRGB(i, 0, 255, 0);
            }else if(i%7==4){
                m_ledBuffer.setRGB(i,0,0,255);
            } else if(i%7==5) {
                m_ledBuffer.setRGB(i, 75, 0, 130);
            } else if(i%7==6) {
                m_ledBuffer.setRGB(i, 127, 0, 255);
            }
        }
    }
    
      public void realYellow_dot(){
        yellow_dot++;
        for(int i=0;i<m_ledBuffer.getLength(); i++){
            if(yellow_dot%28==i){
                m_ledBuffer.setRGB(i, 255, 255, 255);
            } else {
                m_ledBuffer.setRGB(i, 255, 255, 0);
            }
        }
    }
    public void bottomLimit(){
        for(int i = 0; i<m_ledBuffer.getLength(); i++){
            if(i%4 <2){
                m_ledBuffer.setRGB(i, 160, 32, 240);
            }else{
                m_ledBuffer.setRGB(i, 255,0,0);
            }
        }
    }


    public void idle() {
        var alliance = DriverStation.getAlliance();
        if(!alliance.isPresent()){
            return;
        }

        if (alliance.get()== DriverStation.Alliance.Blue) {
            setPulseBlue();
        }
        else if(alliance.get() == DriverStation.Alliance.Red) {
            setPulseRed();
        }
    }

}
