package frc.robot;


import java.util.ArrayList;
import java.util.LinkedList;
import lombok.Setter;
import lombok.Getter;

public class RollingAverage {
    private ArrayList<Double[]> times = new ArrayList<>();
    @Setter @Getter private double rollingTime;
    @Getter private double average = 0;

    public RollingAverage(double rollingTime){
        this.rollingTime = rollingTime;
    }

    public void update(double[] data){
        times.add(new Double[]{data[0],data[1]});

        while(times.size()>0 && times.get(0)[0]>rollingTime){
            times.remove(0);
        }
        double temp = 0;

        for(int i =0; i<times.size();i++){
            temp += times.get(i)[1];
        }
        average = times.size() > 0 ? temp/times.size() : 0;
    }
}
