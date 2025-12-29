^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lightning_rrt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Update README with future work section
* Move get_rrt_path into protected access for testing
* Refactor rrt into a separate function for testing
* Add launch and rviz config
* Bugfixes, naming conventions, and QoL cleanup
* Update README with video link
* Update README
  Expanded the description of the package and added future roadmap ideas.
* Fix link reference in README for example_map.cpp
* Update README with roadmap
  Add roadmap section for future enhancements.
* Merge branch 'main' of github.com:david-dorf/lightning_rrt
* Change example_client to example_map
* Create README.md with package details
  Added installation and usage instructions for the lightning_rrt package.
* Apply fix to bound and collision checking to account for map origin. Add L shaped obstacle to example.
* Fix collision checking. RRT works as expected now. Needs additional checks on start/goal to prevent initial collision/out-of-bounds
* Separate start and goal markers out of array
* Add visualizer. Collision checking is bugged.
* Working RRT with collision and straight line to goal checks
* Add path publisher, empty vector to store nodes
* Add random point generation and bounds checking
* Significant refactor using custom message
* Add start and goal marker publishing to example
* Add example map and separate publishers for start and goal
* Add marker visualization
* Add goal subscription
* Add path publisher and timer
* Add working map subscription
* Initial commit with basic sub and service
* Contributors: David Dorf
