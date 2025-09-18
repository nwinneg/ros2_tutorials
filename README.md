--- 
Author: Nate Winneg
Date:   2024-07-12
---

# Summary: 
This repository contains source code and notes as I've been walking myself through the ros2 tutorials on docs.ros.org
These notes are specific to the ros2 humble hawksbill distro -- a future task will be to try to build these packages in jazzy jalisco or the latest at the time

That is all for now -- more to come (and hopefully a more interesting project in the next repo :)

For lightweight testing / understanding of these packages, docker desktop offers a simple solution.
- To create a disposable container running docker: docker run -it --rm osrf/ros:humble-desktop bash
- To connect a new container in a new terminal window to the same network: docker run -it --rm --network host osrf/ros:humble-desktop bash
  - -it: makes the terminal interactive
  - --rm: removes the container upon exiting to keep host system clean
- This repository can then be cloned from created containers temporarily
