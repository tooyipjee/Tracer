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

  //First Graph
  PhyphoxBleExperiment::Graph firstGraph;      //Create graph which will plot values over time
  firstGraph.setLabel("First Graph");
  firstGraph.setUnitX("x");
  firstGraph.setUnitY("y");
  firstGraph.setLabelX("xLabel");
  firstGraph.setLabelY("yLabel");
  firstGraph.setXPrecision(1);                 //The amount of digits shown after the decimal point
  firstGraph.setYPrecision(1);
  firstGraph.setChannel(0, 1);

  //Second Graph
  PhyphoxBleExperiment::Graph secondGraph;      //Create graph which will plot values over time
  secondGraph.setLabel("Second Graph");
  secondGraph.setUnitX("x");
  secondGraph.setUnitY("y");
  secondGraph.setLabelX("xLabel");
  secondGraph.setLabelY("yLabel");
  secondGraph.setXPrecision(1);                 //The amount of digits shown after the decimal point
  secondGraph.setYPrecision(1);
  secondGraph.setChannel(0, 2);

    //Third Graph
  PhyphoxBleExperiment::Graph thirdGraph;      //Create graph which will plot values over time
  thirdGraph.setLabel("Third Graph");
  thirdGraph.setUnitX("x");
  thirdGraph.setUnitY("y");
  thirdGraph.setLabelX("xLabel");
  thirdGraph.setLabelY("yLabel");
  thirdGraph.setXPrecision(1);                 //The amount of digits shown after the decimal point
  thirdGraph.setYPrecision(1);
  thirdGraph.setChannel(0, 3);

    //Fourth Graph
  PhyphoxBleExperiment::Graph fourthGraph;      //Create graph which will plot values over time
  fourthGraph.setLabel("Fourth Graph");
  fourthGraph.setUnitX("x");
  fourthGraph.setUnitY("y");
  fourthGraph.setLabelX("xLabel");
  fourthGraph.setLabelY("yLabel");
  fourthGraph.setXPrecision(1);                 //The amount of digits shown after the decimal point
  fourthGraph.setYPrecision(1);
  fourthGraph.setChannel(0, 4);

    //Fifth Graph
  PhyphoxBleExperiment::Graph fifthGraph;      //Create graph which will plot values over time
  fifthGraph.setLabel("Fifth Graph");
  fifthGraph.setUnitX("x");
  fifthGraph.setUnitY("y");
  fifthGraph.setLabelX("xLabel");
  fifthGraph.setLabelY("yLabel");
  fifthGraph.setXPrecision(1);                 //The amount of digits shown after the decimal point
  fifthGraph.setYPrecision(1);
  fifthGraph.setChannel(0, 5);

  //attach to experiment
  firstView.addElement(firstGraph);
  firstView.addElement(secondGraph);
  firstView.addElement(thirdGraph);
  firstView.addElement(fourthGraph);
  firstView.addElement(fifthGraph);


  plotAccelGyro.addView(firstView);         //attach view to experiment

  PhyphoxBLE::addExperiment(plotAccelGyro);      //attach experiment to server


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

void Bluetooth::update(float f0, float f1)
{
  PhyphoxBLE::write(f0, f1);
}

void Bluetooth::update(float f0)
{
  PhyphoxBLE::write(f0);
}
Bluetooth::~Bluetooth()
{

}
