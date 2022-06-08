#include "bluetooth.h"

Bluetooth::Bluetooth()
{
  
}

void Bluetooth::init()
{
  PhyphoxBLE::start("Tracer");
  PhyphoxBleExperiment plotAccelGyro;   //generate experiment on Arduino which plot random values

  plotAccelGyro.setTitle("Tracer Plots");
  plotAccelGyro.setCategory("Arduino Experiments");
  plotAccelGyro.setDescription("Tennis data visualised on phyphox");

  //View
  PhyphoxBleExperiment::View firstView;
  firstView.setLabel("Plots"); //Create a "view"

//  PhyphoxBleExperiment::Graph heading;      //Create graph which will plot random numbers over time
//  heading.setLabel("Heading");
//  heading.setUnitX("s");
//  heading.setUnitY("o");
//  heading.setLabelX("Time");
//  heading.setLabelY("Angle");
//  heading.setXPrecision(1);                 //The amount of digits shown after the decimal point
//  heading.setYPrecision(1);
//  heading.setChannel(0, 1);

  //Second Graph
  PhyphoxBleExperiment::Graph pitch;      //Create graph which will plot values over time
  pitch.setLabel("Absolute Acc");
  pitch.setUnitX("s");
  pitch.setUnitY("g");
  pitch.setLabelX("Time");
  pitch.setLabelY("Angle");
  pitch.setXPrecision(1);                 //The amount of digits shown after the decimal point
  pitch.setYPrecision(1);
  pitch.setChannel(0, 1);

  //Third Graph
  PhyphoxBleExperiment::Graph roll;     
  roll.setLabel("Integrated Acc");
  roll.setUnitX("s");
  roll.setUnitY("gs");
  roll.setLabelX("Time");
  roll.setLabelY("Angle");
  roll.setXPrecision(1);                 
  roll.setYPrecision(1);
  roll.setChannel(0, 2);

    //Fourth Graph
  PhyphoxBleExperiment::Graph accel_y;      
  accel_y.setLabel("Accel (y)");
  accel_y.setUnitX("s");
  accel_y.setUnitY("g");
  accel_y.setLabelX("Time");
  accel_y.setLabelY("Angle");
  accel_y.setXPrecision(1);                 
  accel_y.setYPrecision(1);
  accel_y.setChannel(0, 3);
  
  //Fourth Graph
  PhyphoxBleExperiment::Graph accel_x;     
  accel_x.setLabel("Accel (x)");
  accel_x.setUnitX("s");
  accel_x.setUnitY("g");
  accel_x.setLabelX("Time");
  accel_x.setLabelY("Angle");
  accel_x.setXPrecision(1);                 
  accel_x.setYPrecision(1);
  accel_x.setChannel(0, 4);

  //Fifth Graph
  PhyphoxBleExperiment::Graph counts;      
  counts.setLabel("Counts");
  counts.setUnitX("s");
  counts.setUnitY("hits");
  counts.setLabelX("Time");
  counts.setLabelY("Hits");
  counts.setXPrecision(1);                 
  counts.setYPrecision(1);
  counts.setChannel(0, 5);

//  
//  //Info
//  PhyphoxBleExperiment::InfoField myInfo;      //Creates an info-box.
//  myInfo.setInfo("In this view you can set a value between 1 and 10. The squared random value will be multiplied by this value and can be seen here.");
//  myInfo.setXMLAttribute("size=\"1.2\"");
//
//  //Separator
//  PhyphoxBleExperiment::Separator mySeparator;      //Creates a line to separate elements.
//  mySeparator.setHeight(0.3);                       //Sets height of the separator.
//  mySeparator.setColor("404040");                   //Sets color of the separator. Uses a 6 digit hexadecimal value in "quotation marks".
//
//  //Value
//  PhyphoxBleExperiment::Value myValue;         //Creates a value-box.
//  myValue.setLabel("Number");                  //Sets the label
//  myValue.setPrecision(2);                     //The amount of digits shown after the decimal point.
//  myValue.setUnit("u");                        //The physical unit associated with the displayed value.
//  myValue.setColor("FFFFFF");                  //Sets font color. Uses a 6 digit hexadecimal value in "quotation marks".
//  myValue.setChannel(3);
//  myValue.setXMLAttribute("size=\"2\"");
//
//  //Edit
//  PhyphoxBleExperiment::Edit myEdit;
//  myEdit.setLabel("Editfield");
//  myEdit.setUnit("u");
//  myEdit.setSigned(false);
//  myEdit.setDecimal(false);
//  myEdit.setChannel(1);
//  myEdit.setXMLAttribute("max=\"10\"");
//
//  //Export
//  PhyphoxBleExperiment::ExportSet mySet;       //Provides exporting the data to excel etc.
//  mySet.setLabel("mySet");
//
//  PhyphoxBleExperiment::ExportData myData1;
//  myData1.setLabel("myData1");
//  myData1.setDatachannel(1);
//
//  PhyphoxBleExperiment::ExportData myData2;
//  myData2.setLabel("myData2");
//  myData2.setDatachannel(2);

  //attach to experiment

//  firstView.addElement(heading);           
  firstView.addElement(pitch);            
  firstView.addElement(roll);            
  firstView.addElement(accel_y);            
  firstView.addElement(accel_x);            
  firstView.addElement(counts);            

  plotAccelGyro.addView(firstView);         //attach view to experiment

//  plotAccelGyro.addExportSet(mySet);        //attach exportSet to experiment
  PhyphoxBLE::addExperiment(plotAccelGyro);      //attach experiment to server


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
Bluetooth::~Bluetooth()
{
  
}
