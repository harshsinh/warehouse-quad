add_executable (linedetector_node linedetector_node.cxx)
target_link_libraries (linedetector_node
						${catkin_LIBRARIES}
						${OpenCV_LIBRARIES}
)
