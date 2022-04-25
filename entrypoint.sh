#!/bin/bash
source /jackal_ws/devel/setup.sh
cd /jackal_ws/src/nav-competition-icra2022-drl-vo
exec ${@:1}
