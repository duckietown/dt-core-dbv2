# Duckietown car interface - Duckiebot V2

This repository is for the functionality added for Duckiebot V2. DBv2 overrides some of the packages used on
DBv1 (For example, kinematics), and adds some packages not present on DBv1 (For example, sensors). Therefore, this
container inherits from `dt-core` (For DBv1), and changes the launch script to launch the DBv2 versions
of packages where necessary.

This code was mostly written by Merlin Hosner in 
[his fork](https://github.com/hosnerm/Software/tree/devel-dbv2-PI-encoder-18), and copied to this repository by
Timothy Scott.