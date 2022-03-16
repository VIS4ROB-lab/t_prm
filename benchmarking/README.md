# T-PRM Benchmarking

See folder `study` for some examples.

Make sure that you compile against the OMPL version with which you are actually running the benchmarks.

## Run Instructions
1) Build the benchmarking library (`cmake -DBUILD_BENCHMARKING ..` and `make`)
2) Build the study executable (done together with the above command)
3) Run the study executable (`./benchmarking/study_dynamic_obstacles` or `./benchmarking/study_static_obstacles`)