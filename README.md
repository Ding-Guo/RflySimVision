# RflySimVision
[![An example for RflySimVision](https://img.youtube.com/vi/E5NQAp64wsA/maxresdefault.jpg)](https://youtu.be/E5NQAp64wsA)
# #Installation
Requirements: Windows10/11
1. Download and configure the RFlysim platform according to the tutorial of https://rflysim.com/docs/#/en/2_Configuration/SoftwareInstallation
2. Download the demo code of RflySimVision at [https://github.com/Ding-Guo/RflySimVision.git](https://github.com/Ding-Guo/RflySimVision.git)
3. open wsl  
```
mv RflySimVision/ego-planner-swarm RflySimVision/sensor_pkg /home 
cd /home/ego-planner-swarm
catkin_make
``` 
## Autonomous exploration
### Windows
```
cd RflySimVision
SingleRun.bat
```
### WSL
```
cd /home/ego-planner-swarm/shfiles
bash single.sh
```
<p align = "center">
<img src="pictures/demo1.gif" width = "576" height = "324" border="5" />
</p>

## Swarm exploration
### Windows
```
cd RflySimVision
SwarmRun.bat
```
### WSL
```
cd /home/ego-planner-swarm/shfiles
bash swarm.sh
```
<p align = "center">
<img src="pictures/demo2.gif" width = "576" height = "324" border="5" />
</p>
