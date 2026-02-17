package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{
    private static LED instance = null;

    private AddressableLED control = new AddressableLED(0); //TODO
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(0); //TODO

    private final Color INIT_YELLOW = new Color(255, 165, 0); 
    private final Color BLUE = new Color(0, 0, 255); 
    private final Color GREEN = new Color(0, 255, 0); 
    private final Color RED = new Color(255, 0, 0); 
    private final Color WHITE = new Color(255, 255, 255);
    private final Color OFF = new Color(0, 0, 0);

    private int blinkCounter = 0;
    
    private LED(){
        super("LED");
        this.control.setLength(buffer.getLength());
        this.control.setColorOrder(AddressableLED.ColorOrder.kRGB);
        this.setLights(INIT_YELLOW);
        this.control.setData(buffer);
        this.control.start();
    }

    public void setLights(Color color){
        for(int i = 0; i < buffer.getLength(); i++){
            buffer.setLED(i, color);
        }
    }

    public void setLightsBlink(Color color, int freq){
        if(blinkCounter % (2 * freq) < freq) this.setLights(color);
        else this.setLights(OFF);
    }

    @Override
    public void periodic(){
        if(Intake.getInstance().isIntaking()) this.setLights(BLUE);
    }

    public static LED getInstance(){
        if(instance == null) instance = new LED();
        return instance;

    }
}
