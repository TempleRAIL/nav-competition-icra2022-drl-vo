<p align="center">
  <img width = "100%" src='res/BARN_Challenge.png' />
  </p>

--------------------------------------------------------------------------------

# DRL-VO control policy for ICRA 2022 BARN Challenge

Our DRL-VO control policy ranked 1st in the simulated competition and 3rd in the final physical competition of the ICRA 2022 BARN Challenge.
Implementation details can be found at our paper ["DRL-VO: Learning to Navigate Through Crowded Dynamic Scenes Using Velocity Obstacles"](https://doi.org/10.1109/TRO.2023.3257549
)([arXiv](https://arxiv.org/pdf/2301.06512.pdf)) in IEEE Transactions on Robotics (T-RO) 2023. 
Video demos can be found at [multimedia demonstrations](https://www.youtube.com/watch?v=KneELRT8GzU&list=PLouWbAcP4zIvPgaARrV223lf2eiSR-eSS&index=2&ab_channel=PhilipDames).

The details of BARN Challenge can be found at our paper ["Autonomous Ground Navigation in Highly Constrained Spaces: Lessons Learned From the Benchmark Autonomous Robot Navigation Challenge at ICRA 2022 [Competitions]"](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9975161)

* navigation metric: 0.2339

## Requirements:
* Ubuntu 20.04/18.04
* ROS-Noetic/ROS Melodic
* Python 3.7
* Singularity 

## Usage:
Download pre-created ["nav_competition_image.sif"](https://doi.org/10.5281/zenodo.7851293) container to the home directory.
### Simulation:
```
# clone this project:
git clone -b hardware https://github.com/TempleRAIL/nav-competition-icra2022-drl-vo.git
cd nav-competition-icra2022-drl-vo

# move nav_competition_image.sif container to current directory:
mv ~/nav_competition_image.sif ./

# single world test:
./singularity_run.sh ./nav_competition_image.sif python run.py --out ~/drl_vo_out.txt

# 50 worlds test: 1 trial
./singularity_run.sh ./nav_competition_image.sif python run_drl_vo.py --out ~/drl_vo_out.txt --trials 1

# 50 worlds test: 10 trial
./singularity_run.sh ./nav_competition_image.sif python run_drl_vo.py --out ~/drl_vo_out.txt --trials 10

# report results:
./singularity_run.sh ./nav_competition_image.sif python report_test.py --out_path ~/drl_vo_out.txt
```
### Hardware:
```
# enter the directory of nav_competition_image.sif container and run the container:
cd ~
singularity shell --nv nav_competition_image.sif
source /etc/.bashrc

# create ros workspace and clone this project:
mkdir -p temple_ws/src
cd temple_ws/src
git clone -b hardware https://github.com/TempleRAIL/nav-competition-icra2022-drl-vo.git
catkin_make
source devel/setup.sh

# set the goal point and run the DRL-VO policy: Cartesian coordinate system (the positive direction of the x-axis is to the right, and the positive direction of the y-axis is forward)
roslaunch jackal_helper move_base_drl_vo.launch goal_x:="-10" goal_y:="20"
```

--------------------------------------------------------------------------------
## Citation
```
@article{xie2023drl,
  author={Xie, Zhanteng and Dames, Philip},
  journal={IEEE Transactions on Robotics}, 
  title={DRL-VO: Learning to Navigate Through Crowded Dynamic Scenes Using Velocity Obstacles}, 
  year={2023},
  volume={},
  number={},
  pages={1-20},
  doi={10.1109/TRO.2023.3257549}
}

@article{xiao2022autonomous,
  title={Autonomous Ground Navigation in Highly Constrained Spaces: Lessons Learned From the Benchmark Autonomous Robot Navigation Challenge at ICRA 2022 [Competitions]},
  author={Xiao, Xuesu and Xu, Zifan and Wang, Zizhao and Song, Yunlong and Warnell, Garrett and Stone, Peter and Zhang, Tingnan and Ravi, Shravan and Wang, Gary and Karnan, Haresh and others},
  journal={IEEE Robotics \& Automation Magazine},
  volume={29},
  number={4},
  pages={148--156},
  year={2022},
  publisher={IEEE}
}

```
