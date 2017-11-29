add_executable (pcadetector_node
				pcadetector_node.cxx
				slic.cpp)
target_link_libraries (pcadetector_node
						${catkin_LIBRARIES}
						${OpenCV_LIBRARIES}
)
