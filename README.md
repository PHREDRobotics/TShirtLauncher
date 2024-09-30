# TShirtLauncher

# Instructions
## TODO

* Purpose

* To Run Instructions
 * attach Safety Switch and Drive Controller
 * Store Ram Rod

* Power-Up
* Pressurize 
  * Compressor
  * External
  
* Charging Battery

* To Operate
  * Hold safety switch - Drive
  * Load T-shirt
  * Store Ram-rod
  * Fire
  
* Light Meanings

Color       | Meaning 
----------- | ----------
Rainbow     | Powered On
Blue        | Safety Switch Held But Ram Rod Not Stored, Launcher can be driven
Red         | Launcher Is armed and Can fire
Green       | Launcher is firing 
Orange Base | Compressor is on and the pressure is Below the Low Pressure setting.  The height of the Orange bars represents how far to go to full pressure
Yellow Base | Compressor is on, and above the Low pressure value, but not yet to Full pressure.  The height of the yellow represents the proportion
 
 
## Leg Filling Logic

Leg LEDs go logically up or down.  The leg array stores the length of each leg, the number for the "top" node, and the direction to move "down" the leg.

We also store values for the minimum leg length and the number of nodes in the "top half" of the legs.

The fill color is determined by if the PSI is above the PSI_START value or not. Fill all the legs with the fill color.

Then for each leg, figure out what to "erase" based on the current PSI:

If current PSI > PSI_START, we are adjusting above the mid-way point

If not, then erase to midway and adjust further.
  
