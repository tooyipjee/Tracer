

#include "bluetooth.h"

Bluetooth::Bluetooth()
{
  
}

void Bluetooth::init(bool dataCollector)
{
  PhyphoxBLE::start("Tracer-Tennis");
  PhyphoxBleExperiment plotAccelGyro;   //generate experiment on Arduino which plot random values

  plotAccelGyro.setTitle("Tracer Plots");
  plotAccelGyro.setCategory("Arduino Experiments");
  plotAccelGyro.setDescription("Tennis data visualised on phyphox");

  if (dataCollector)
    {
      //View
      PhyphoxBleExperiment::View firstView;
      firstView.setLabel("Plots"); //Create a "view"
      //First Graph
      PhyphoxBleExperiment::Graph timeMs;      //Create graph which will plot values over time
      timeMs.setLabel("ax");
      timeMs.setUnitX("s");
      timeMs.setUnitY("ms");
      timeMs.setLabelX("Time");
      timeMs.setLabelY("Time");
      timeMs.setXPrecision(1);                 //The amount of digits shown after the decimal point
      timeMs.setYPrecision(1);
      timeMs.setChannel(0, 1);
      
      //Second Graph
      PhyphoxBleExperiment::Graph accel_x;      //Create graph which will plot values over time
      accel_x.setLabel("ax");
      accel_x.setUnitX("s");
      accel_x.setUnitY("m/s^2");
      accel_x.setLabelX("Time");
      accel_x.setLabelY("Accel");
      accel_x.setXPrecision(1);                 //The amount of digits shown after the decimal point
      accel_x.setYPrecision(1);
      accel_x.setChannel(0, 2);

      //Third Graph
      PhyphoxBleExperiment::Graph accel_y;      //Create graph which will plot values over time
      accel_y.setLabel("ay");
      accel_y.setUnitX("s");
      accel_y.setUnitY("m/s^2");
      accel_y.setLabelX("Time");
      accel_y.setLabelY("Accel");
      accel_y.setXPrecision(1);                 //The amount of digits shown after the decimal point
      accel_y.setYPrecision(1);
      accel_y.setChannel(0, 3);

        //Fourth Graph
        PhyphoxBleExperiment::Graph accel_z;      //Create graph which will plot values over time
      accel_z.setLabel("az");
      accel_z.setUnitX("s");
      accel_z.setUnitY("m/s^2");
      accel_z.setLabelX("Time");
      accel_z.setLabelY("Accel");
      accel_z.setXPrecision(1);                 //The amount of digits shown after the decimal point
      accel_z.setYPrecision(1);
      accel_z.setChannel(0, 4);

      
      
      //attach to experiment        
      firstView.addElement(timeMs);    
      firstView.addElement(accel_x);       
      firstView.addElement(accel_y);  
      firstView.addElement(accel_z);       
            
      plotAccelGyro.addView(firstView);         //attach view to experiment

    //  plotAccelGyro.addExportSet(mySet);        //attach exportSet to experiment
      PhyphoxBLE::addExperiment(plotAccelGyro);      //attach experiment to server
    }
  else
    {
            //View
      PhyphoxBleExperiment::View firstView;
      firstView.setLabel("Plots"); //Create a "view"
      //First Graph
      PhyphoxBleExperiment::Graph hits;      //Create graph which will plot values over time
      hits.setLabel("Hits");
      hits.setUnitX("s");
      hits.setUnitY("");
      hits.setLabelX("Time");
      hits.setLabelY("No. of hits");
      hits.setXPrecision(1);                 //The amount of digits shown after the decimal point
      hits.setYPrecision(1);
      hits.setChannel(0, 1);
      
      //Second Graph
      PhyphoxBleExperiment::Graph TS_hits;      //Create graph which will plot values over time
      TS_hits.setLabel("Top Spin");
      TS_hits.setUnitX("s");
      TS_hits.setUnitY("");
      TS_hits.setLabelX("Time");
      TS_hits.setLabelY("No. of hits");
      TS_hits.setXPrecision(1);                 //The amount of digits shown after the decimal point
      TS_hits.setYPrecision(1);
      TS_hits.setChannel(0, 2);

      //Third Graph
      PhyphoxBleExperiment::Graph SL_hits;      //Create graph which will plot values over time
      SL_hits.setLabel("Slice");
      SL_hits.setUnitX("s");
      SL_hits.setUnitY("");
      SL_hits.setLabelX("Time");
      SL_hits.setLabelY("No. of hits");
      SL_hits.setXPrecision(1);                 //The amount of digits shown after the decimal point
      SL_hits.setYPrecision(1);
      SL_hits.setChannel(0, 3);

      //Third Graph
      PhyphoxBleExperiment::Graph Flat_hits;      //Create graph which will plot values over time
      Flat_hits.setLabel("Flat");
      Flat_hits.setUnitX("s");
      Flat_hits.setUnitY("");
      Flat_hits.setLabelX("Time");
      Flat_hits.setLabelY("No. of hits");
      Flat_hits.setXPrecision(1);                 //The amount of digits shown after the decimal point
      Flat_hits.setYPrecision(1);
      Flat_hits.setChannel(0, 4);
      
      //attach to experiment        
      firstView.addElement(hits);    
      firstView.addElement(TS_hits);       
      firstView.addElement(SL_hits); 
      firstView.addElement(Flat_hits);  
            
      plotAccelGyro.addView(firstView);         //attach view to experiment

    //  plotAccelGyro.addExportSet(mySet);        //attach exportSet to experiment
      PhyphoxBLE::addExperiment(plotAccelGyro);      //attach experiment to server
    }


}

void Bluetooth::update(float f0, float f1, float f2, float f3, float f4, float f5, float f6, float f7, float f8)
{
  PhyphoxBLE::write(f0, f1, f2, f3,  f4,  f5,  f6,  f7,  f8);
}

void Bluetooth::update(float f0, float f1, float f2, float f3, float f4, float f5)
{
  PhyphoxBLE::write(f0, f1, f2, f3,  f4,  f5);
}
void Bluetooth::update(float f0, float f1, float f2, float f3, float f4)
{
  PhyphoxBLE::write(f0, f1, f2, f3,  f4);
}
void Bluetooth::update(float f0, float f1, float f2, float f3)
{
  PhyphoxBLE::write(f0, f1, f2, f3);
}
void Bluetooth::update(float f0, float f1, float f2)
{
  PhyphoxBLE::write(f0, f1, f2);
}

Bluetooth::~Bluetooth()
{
  
}
