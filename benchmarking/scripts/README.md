# Generating movies out of benchmarks

As shown in the video of the publication, we create several movies of the planners in static and dynamic environments.

After running either `study_dynamic_obstacles` or `study_static_obstacles`, several files will be generated.

In the current workspace, there will be a file called `info_about_bm.txt` which contains the information about the benchmark. Furthermore, several `*.path` files will be generated (each containing a path from the start to the goal for a given planner and benchmark). Using the script found in this folder, we can create the movies (either using the 2D or 3D script).