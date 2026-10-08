package frc.robot;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

import edu.wpi.first.math.util.Units;
import lombok.Getter;
public class FieldLines {
    @Getter public static final Line2D[] field = convert(new Line2D[]{
        //field walls
        new Line2D.Double(0,0,650.12,0),
        new Line2D.Double(0,0,0,316.64),
        new Line2D.Double(0,316.64,650.12,316.64),
        new Line2D.Double(650.12,316.64,650.12,0),
        //blue hub
        new Line2D.Double(158.06,134.82,158.06,181.82),
        new Line2D.Double(158.06,134.82,205.06,134.82),
        new Line2D.Double(158.06,181.82,205.06,181.82),
        new Line2D.Double(205.06,181.82,205.06,134.82),
        //red hub
        new Line2D.Double(445.06,134.82,492.06,134.82),
        new Line2D.Double(445.06,134.82,445.06,181.82),
        new Line2D.Double(445.06,181.82,492.06,181.82),
        new Line2D.Double(492.06,181.82,492.06,134.82),
    });

    private static Line2D[] convert(Line2D[] data){
        for(int i = 0;i < data.length;i++){
            data[i] = new Line2D.Double(
                inchToMeter(data[i].getX1()),
                inchToMeter(data[i].getY1()),
                inchToMeter(data[i].getX2()),
                inchToMeter(data[i].getY2()));
        }
        return data;
    }

    private static double inchToMeter(double val){
        return Units.inchesToMeters(val);
    }
}
