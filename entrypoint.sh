#!/bin/bash
source /etc/.bashrc
cd /jackal_ws/src/nav-competition-icra2022-drl-vo
exec ${@:1}
