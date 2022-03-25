

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
  plotAccelGyro.setDescription("Accel data visualised on phyphox");

  //View
  PhyphoxBleExperiment::View firstView;
  firstView.setLabel("Plots"); //Create a "view"
//  PhyphoxBleExperiment::View secondView;
//  secondView.setLabel("Control"); //Create a "view"

  //Graph
  /* Assign Channels, so which data is plotted on x or y axis
     first parameter represents x-axis, second y-axis
     Channel 0 means a timestamp is created after the BLE package arrives in phyphox
     Channel 1 to N corresponding to the N-parameter which is written in server.write()
  */
  PhyphoxBleExperiment::Graph accel_x;      //Create graph which will plot random numbers over time
  accel_x.setLabel("Heading");
  accel_x.setUnitX("s");
  accel_x.setUnitY("o");
  accel_x.setLabelX("Time");
  accel_x.setLabelY("Angle");
  accel_x.setXPrecision(1);                 //The amount of digits shown after the decimal point
  accel_x.setYPrecision(1);
  accel_x.setChannel(0, 1);

  //Second Graph
  PhyphoxBleExperiment::Graph accel_y;      //Create graph which will plot random numbers over time
  accel_y.setLabel("Pitch");
  accel_y.setUnitX("s");
  accel_y.setUnitY("o");
  accel_y.setLabelX("Time");
  accel_y.setLabelY("Angle");
  accel_y.setXPrecision(1);                 //The amount of digits shown after the decimal point
  accel_y.setYPrecision(1);
  accel_y.setChannel(0, 2);

  //Third Graph
  PhyphoxBleExperiment::Graph accel_z;      //Create graph which will plot random numbers over time
  accel_z.setLabel("Roll");
  accel_z.setUnitX("s");
  accel_z.setUnitY("o");
  accel_z.setLabelX("Time");
  accel_z.setLabelY("Angle");
  accel_z.setXPrecision(1);                 //The amount of digits shown after the decimal point
  accel_z.setYPrecision(1);
  accel_z.setChannel(0, 3);
  
//  //Info
//  PhyphoxBleExperiment::InfoField myInfo;      //Creates an info-box.
//  myInfo.setInfo("In this view you can set a value between 1 and 10. The squared random value will be multiplied by this value and can be seen here.");
//  //myInfo.setColor("404040");                   //Sets font color. Uses a 6 digit hexadecimal value in "quotation marks".
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

  firstView.addElement(accel_x);            //attach graph to view
  firstView.addElement(accel_y);            //attach second graph to view
  firstView.addElement(accel_z);            //attach second graph to view
//  secondView.addElement(myInfo);                //attach info to view
//  secondView.addElement(mySeparator);          //attach separator to view
//  secondView.addElement(myValue);               //attach value to view
//  secondView.addElement(myEdit);               //attach editfield to view (Linked to value)
  plotAccelGyro.addView(firstView);         //attach view to experiment
//  plotAccelGyro.addView(secondView);
//  mySet.addElement(myData1);                   //attach data to exportSet
//  mySet.addElement(myData2);                   //attach data to exportSet
//  plotAccelGyro.addExportSet(mySet);        //attach exportSet to experiment
  PhyphoxBLE::addExperiment(plotAccelGyro);      //attach experiment to server


}

void Bluetooth::update(float x, float y, float z)
{
  PhyphoxBLE::write(x, y, z);
}

Bluetooth::~Bluetooth()
{
  
}
