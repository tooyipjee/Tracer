###########################################
#
# Example script to build a
# pinout diagram. Includes basic
# features and convenience classes.
#
###########################################

from pinout.core import Group, Image
from pinout.components.layout import Diagram_2Rows
from pinout.components.pinlabel import PinLabelGroup, PinLabel
from pinout.components.text import TextBlock
from pinout.components import leaderline as lline
from pinout.components.legend import Legend


# Import data for the diagram
import data

# Create a new diagram
# The Diagram_2Rows class provides 2 panels,
# 'panel_01' and 'panel_02', to insert components into.
diagram = Diagram_2Rows(1600, 1050, 905, "diagram")

# Add a stylesheet
diagram.add_stylesheet("styles.css", embed=True)

# Create a group to hold the pinout-diagram components.
graphic = diagram.panel_01.add(Group(400, 42))

# Add and embed an image
hardware = graphic.add(Image("hardware.png", embed=True))


# Create a single pin label
graphic.add(
    PinLabel(
        content="RESET",
        x=137,
        y=713,
        tag="pwr",
        body={"x": 200, "y": 30, "height":25},
        scale=(-1,-1)
    )
)

graphic.add(
    PinLabel(
        content="FLASH",
        x=336,
        y=669,
        tag="mcu",
        body={"x": 200, "y": 30, "height":25},
        scale=(-1,-1)
    )
)

graphic.add(
    PinLabel(
        content="ESP32-PICO-D4",
        x=227,
        y=515,
        tag="mcu",
        body={"x": 200, "y": 30, "width":150, "height":25},
        scale=(-1,-1)
    )
)

graphic.add(
    PinLabel(
        content="VL53L0X",
        x=257,
        y=344,
        tag="sns",
        body={"x": 200, "y": 30, "height":25},
        scale=(-1,-1)
    )
)

graphic.add(
    PinLabel(
        content="3.3V 700mA LDO",
        x=261,
        y=249,
        tag="pwr",
        body={"x": 200, "y": 30, "height":25, "width":150},
        scale=(-1,-1)
    )
)

graphic.add(
    PinLabel(
        content="TP4065",
        x=500,
        y=279,
        tag="pwr",
        body={"x": 200, "y": 30, "height":25},
        scale=(1,1)
    )
)

graphic.add(
    PinLabel(
        content="Micro USB",
        x=650,
        y=422,
        tag="con",
        body={"x": 200, "y": 30, "height":25, "width":150},
        scale=(1,1)
    )
)

graphic.add(
    PinLabel(
        content="LSM6DSL",
        x=372,
        y=456,
        tag="sns",
        body={"x": 350, "y": 30, "height":25, "width":150},
        scale=(1,1)
    )
)

graphic.add(
    PinLabel(
        content="CP2104",
        x=452,
        y=555,
        tag="mcu",
        body={"x": 200, "y": 30, "height":25},
        scale=(1,1)
    )
)

graphic.add(
    PinLabel(
        content="DEBUG",
        x=524,
        y=750,
        tag="con",
        body={"x": 200, "y": 30, "height":25},
        scale=(1,1)
    )
)

# Create a title and description text-blocks
title_block = diagram.panel_02.add(
    TextBlock(
        data.title,
        x=20,
        y=30,
        line_height=18,
        tag="panel title_block",
    )
)
diagram.panel_02.add(
    TextBlock(
        data.description,
        x=20,
        y=60,
        width=title_block.width,
        height=diagram.panel_02.height - title_block.height,
        line_height=18,
        tag="panel text_block",
    )
)

# Create a legend
legend = diagram.panel_02.add(
    Legend(
        data.legend,
        x=340,
        y=8,
        max_height=132,
    )
)

# Export the diagram via commandline:
# >>> py -m pinout.manager --export pinout_diagram.py diagram.svg
