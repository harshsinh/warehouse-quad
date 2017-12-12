add_executable (skulldetector_node skulldetector_node.cxx)
target_link_libraries (skulldetector_node
						${catkin_LIBRARIES}
						${OpenCV_LIBRARIES}
)

