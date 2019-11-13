# Omni-bot
## Introduction
The Omni Bot is a differential drive robot with an omnidirectional camera, built for usage with various Visual Compass algorithms. The robot itself is modelled with only minimal sensors, to show that this can be performed using low-budget robots and sensors. The algorithm included in the initial release is based on work by Wystrach et al., (2013). In that it performs a 'learning walk' to record several snapshots of it's route before attempting to use those snapshots to return home. In the future, more variants will be added, such as the original Cartwright and Collett (1983) method and some other variations.

## Package content
This package currently contains the following features:

### A visual compass algorithm using the multi-snapshot method described by Wystrach et al(2013)

***This Algorithm is still under development. work is being done to reduce the number of mismatches when attempting to find its way back to the nest***

### A world with three cylinders arranged in a triangle around the robot. as per the class Cartwright and Collet (1983) paper.

![fig.1 camera view in Collett-based world](/figures_and_diagrams/Col_world.png)
*fig.1 camera view in Collett-based world*


### A world with a single cylinder (as nest). as per the Wystrach et al (2013) paper.

![fig.2 camera view in Wystrach-based world](/figures_and_diagrams/Wys_world.png)
*fig.2 camera view in Wystrach-based world*

### A URDF and xacro robot model built with a differential drive system and an omni-directional camera

![fig.3a a robot model shown in rviz](/figures_and_diagrams/omni_model.png)

*fig.3a robot model shown in rviz*

![fig.3b screenshot of camera feed within the Collet-based world](/figures_and_diagrams/Collett_cam_view.png)

*fig.3b screenshot of camera feed within the Collet-based world*

![fig.3c screenshot of camera feed within the Wystrach-based world](/figures_and_diagrams/Wystrach_cam_view.png)

*fig.3c screenshot of camera feed within the Wystrach-based world*

## References

[Cartwright, B.A. and Collett, T.S. (1983) Landmark learning in bees. *Journal of Comparative Physiology*, 151(4) 521–543.](https://link.springer.com/article/10.1007/BF00605469)
[Wystrach, A., Mangan, M., Philippides, A. and Graham, P. (2013) Snapshots in ants? New interpretations of paradigmatic experiments. *The Journal of Experimental Biology*, 216 1766–1770.](https://jeb.biologists.org/content/216/10/1766.short)


