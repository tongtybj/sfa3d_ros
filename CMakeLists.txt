cmake_minimum_required(VERSION 3.0.2)
project(sfa3d_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  catkin_virtualenv
)

# catkin_python_setup()


catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

if (${catkin_virtualenv_VERSION} VERSION_LESS "0.6.1")
  message(WARNING "Please install catkin_virtualenv>=0.6.1.")
  message(WARNING "Current catkin_virtualen version is ${catkin_virtualenv_VERSION}")
else()
  catkin_generate_virtualenv(
    PYTHON_INTERPRETER python3
    USE_SYSTEM_PACKAGES TRUE
    ISOLATE_REQUIREMENTS FALSE
    CHECK_VENV FALSE
    )
endif()


# http://docs.ros.org/en/jade/api/catkin/html/howto/format2/installing_python.html
# catkin_install_python is very important not only to declare the executable python files,
# but also record the dependency for thoses file in sys.path (which is more important for venv)
catkin_install_python(
  PROGRAMS scripts/sfa3d_ros_wrapper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

