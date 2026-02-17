package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{

    private static LED instance = null;
    private AddressableLED control = new AddressableLED(0);//TODO FILLER

    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(64);//TODO FILLER

    private final Color WHITE = new Color(255, 255, 255);
    private final Color GREEN = new Color(0, 255, 0);
    private final Color RED = new Color(255, 0, 0);
    private final Color ORANGE = new Color(255, 128, 0);
    private final LEDPattern ORANGE_BLINK = LEDPattern.solid(ORANGE).blink(Seconds.of(0.3));
    private LED(){
        super("LED");
        control.setLength(buffer.getLength());
        control.setColorOrder(AddressableLED.ColorOrder.kRGB);
    }

    public void setLights(Color color){
        for(int i = 0; i < buffer.getLength(); i++){
            buffer.setLED(i, color);
        }
    }

    public void setLights(Color col1, Color col2, Color col3){
        for(int i = 0; i < buffer.getLength()/2; i+=1){
            if(i % 3 == 0){
                buffer.setLED(2*i, col1);
                buffer.setLED(2*i+1, col1);
            }
            else if(i % 3 == 1){
                buffer.setLED(2*i, col2);
                buffer.setLED(2*i+1, col2);
            }
            else{
                buffer.setLED(2*i, col3);
                buffer.setLED(2*i+1, col3);
            };
        }
    }

    public void setLEDTankDisable(){
        ORANGE_BLINK.applyTo(this.buffer);
    }
    
    public void setLEDDisable(){
        this.setLights(RED);
    }

    public void setLEDShooting(){
        this.setLights(GREEN);
    }

    public void setLEDIntaking(){
        this.setLights(WHITE);
    }


    @Override
    public void periodic(){
        

        this.control.setData(buffer);
    }

    public static LED getInstance(){
        if(LED.instance == null) 
            LED.instance = new LED();
        return LED.instance;
    }
}
