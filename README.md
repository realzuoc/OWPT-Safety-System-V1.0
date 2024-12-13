# OWPT-Safety-System-V1.0
The most fundamental safety system developed for Optical Wireless Power Transmission System using camera.

The system detail could be view from the paper that has already published. 
The paper DOI is: 
Zuo, C.; Miyamoto, T. Camera-Based Safety System for Optical Wireless Power Transmission Using Dynamic Safety-Distance. Photonics 2024, 11, 500. https://doi.org/10.3390/photonics11060500

For the system, it should be ok to run with any computer that has the environment and necessary package, which could be installed from the header of the two python code files. 
The algorithm contains in the code was developped based on python, and only using OpenCV library and CPU to operate.

Hardware requirements: Any realsense camera, recommond D400 series and above; PC.
Software requirements: Necessary Python packages; Visual C++ Library; please use python version lower than 3.10.

The light control module should be run first, this is to simulate the light is activated first in any application of OWPT, the safety system is a system that surveillance the operation. If you do not have the serial connected stage light or something else, please just comment the "lc.controller()" in the main, as the system can also output the text in the command window for indication of the light status.

After running the light control, make sure camera is connected, and run the safety system code.

This work is part of the Optical Wireless Power Transmission System, the detail of the OWPT can be view from our laboratory website,
https://www.first.iir.titech.ac.jp/member/core3.html#miyamoto
