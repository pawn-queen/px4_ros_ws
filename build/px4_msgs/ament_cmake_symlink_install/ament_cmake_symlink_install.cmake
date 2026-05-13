# generated from
# ament_cmake_core/cmake/symlink_install/ament_cmake_symlink_install.cmake.in

# create empty symlink install manifest before starting install step
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/symlink_install_manifest.txt")

#
# Reimplement CMake install(DIRECTORY) command to use symlinks instead of
# copying resources.
#
# :param cmake_current_source_dir: The CMAKE_CURRENT_SOURCE_DIR when install
#   was invoked
# :type cmake_current_source_dir: string
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(ament_cmake_symlink_install_directory cmake_current_source_dir)
  cmake_parse_arguments(ARG "OPTIONAL" "DESTINATION" "DIRECTORY;PATTERN;PATTERN_EXCLUDE" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_cmake_symlink_install_directory() called with "
      "unused/unsupported arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # make destination absolute path and ensure that it exists
  if(NOT IS_ABSOLUTE "${ARG_DESTINATION}")
    set(ARG_DESTINATION "/home/pawn/px4_ros_ws/install/px4_msgs/${ARG_DESTINATION}")
  endif()
  if(NOT EXISTS "${ARG_DESTINATION}")
    file(MAKE_DIRECTORY "${ARG_DESTINATION}")
  endif()

  # default pattern to include
  if(NOT ARG_PATTERN)
    set(ARG_PATTERN "*")
  endif()

  # iterate over directories
  foreach(dir ${ARG_DIRECTORY})
    # make dir an absolute path
    if(NOT IS_ABSOLUTE "${dir}")
      set(dir "${cmake_current_source_dir}/${dir}")
    endif()

    if(EXISTS "${dir}")
      # if directory has no trailing slash
      # append folder name to destination
      set(destination "${ARG_DESTINATION}")
      string(LENGTH "${dir}" length)
      math(EXPR offset "${length} - 1")
      string(SUBSTRING "${dir}" ${offset} 1 dir_last_char)
      if(NOT dir_last_char STREQUAL "/")
        get_filename_component(destination_name "${dir}" NAME)
        set(destination "${destination}/${destination_name}")
      else()
        # remove trailing slash
        string(SUBSTRING "${dir}" 0 ${offset} dir)
      endif()

      # glob recursive files
      set(relative_files "")
      foreach(pattern ${ARG_PATTERN})
        file(
          GLOB_RECURSE
          include_files
          RELATIVE "${dir}"
          "${dir}/${pattern}"
        )
        if(NOT include_files STREQUAL "")
          list(APPEND relative_files ${include_files})
        endif()
      endforeach()
      foreach(pattern ${ARG_PATTERN_EXCLUDE})
        file(
          GLOB_RECURSE
          exclude_files
          RELATIVE "${dir}"
          "${dir}/${pattern}"
        )
        if(NOT exclude_files STREQUAL "")
          list(REMOVE_ITEM relative_files ${exclude_files})
        endif()
      endforeach()
      list(SORT relative_files)

      foreach(relative_file ${relative_files})
        set(absolute_file "${dir}/${relative_file}")
        # determine link name for file including destination path
        set(symlink "${destination}/${relative_file}")

        # ensure that destination exists
        get_filename_component(symlink_dir "${symlink}" PATH)
        if(NOT EXISTS "${symlink_dir}")
          file(MAKE_DIRECTORY "${symlink_dir}")
        endif()

        _ament_cmake_symlink_install_create_symlink("${absolute_file}" "${symlink}")
      endforeach()
    else()
      if(NOT ARG_OPTIONAL)
        message(FATAL_ERROR
          "ament_cmake_symlink_install_directory() can't find '${dir}'")
      endif()
    endif()
  endforeach()
endfunction()

#
# Reimplement CMake install(FILES) command to use symlinks instead of copying
# resources.
#
# :param cmake_current_source_dir: The CMAKE_CURRENT_SOURCE_DIR when install
#   was invoked
# :type cmake_current_source_dir: string
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(ament_cmake_symlink_install_files cmake_current_source_dir)
  cmake_parse_arguments(ARG "OPTIONAL" "DESTINATION;RENAME" "FILES" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_cmake_symlink_install_files() called with "
      "unused/unsupported arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # make destination an absolute path and ensure that it exists
  if(NOT IS_ABSOLUTE "${ARG_DESTINATION}")
    set(ARG_DESTINATION "/home/pawn/px4_ros_ws/install/px4_msgs/${ARG_DESTINATION}")
  endif()
  if(NOT EXISTS "${ARG_DESTINATION}")
    file(MAKE_DIRECTORY "${ARG_DESTINATION}")
  endif()

  if(ARG_RENAME)
    list(LENGTH ARG_FILES file_count)
    if(NOT file_count EQUAL 1)
    message(FATAL_ERROR "ament_cmake_symlink_install_files() called with "
      "RENAME argument but not with a single file")
    endif()
  endif()

  # iterate over files
  foreach(file ${ARG_FILES})
    # make file an absolute path
    if(NOT IS_ABSOLUTE "${file}")
      set(file "${cmake_current_source_dir}/${file}")
    endif()

    if(EXISTS "${file}")
      # determine link name for file including destination path
      get_filename_component(filename "${file}" NAME)
      if(NOT ARG_RENAME)
        set(symlink "${ARG_DESTINATION}/${filename}")
      else()
        set(symlink "${ARG_DESTINATION}/${ARG_RENAME}")
      endif()
      _ament_cmake_symlink_install_create_symlink("${file}" "${symlink}")
    else()
      if(NOT ARG_OPTIONAL)
        message(FATAL_ERROR
          "ament_cmake_symlink_install_files() can't find '${file}'")
      endif()
    endif()
  endforeach()
endfunction()

#
# Reimplement CMake install(PROGRAMS) command to use symlinks instead of copying
# resources.
#
# :param cmake_current_source_dir: The CMAKE_CURRENT_SOURCE_DIR when install
#   was invoked
# :type cmake_current_source_dir: string
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(ament_cmake_symlink_install_programs cmake_current_source_dir)
  cmake_parse_arguments(ARG "OPTIONAL" "DESTINATION" "PROGRAMS" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_cmake_symlink_install_programs() called with "
      "unused/unsupported arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # make destination an absolute path and ensure that it exists
  if(NOT IS_ABSOLUTE "${ARG_DESTINATION}")
    set(ARG_DESTINATION "/home/pawn/px4_ros_ws/install/px4_msgs/${ARG_DESTINATION}")
  endif()
  if(NOT EXISTS "${ARG_DESTINATION}")
    file(MAKE_DIRECTORY "${ARG_DESTINATION}")
  endif()

  # iterate over programs
  foreach(file ${ARG_PROGRAMS})
    # make file an absolute path
    if(NOT IS_ABSOLUTE "${file}")
      set(file "${cmake_current_source_dir}/${file}")
    endif()

    if(EXISTS "${file}")
      # determine link name for file including destination path
      get_filename_component(filename "${file}" NAME)
      set(symlink "${ARG_DESTINATION}/${filename}")
      _ament_cmake_symlink_install_create_symlink("${file}" "${symlink}")
    else()
      if(NOT ARG_OPTIONAL)
        message(FATAL_ERROR
          "ament_cmake_symlink_install_programs() can't find '${file}'")
      endif()
    endif()
  endforeach()
endfunction()

#
# Reimplement CMake install(TARGETS) command to use symlinks instead of copying
# resources.
#
# :param TARGET_FILES: the absolute files, replacing the name of targets passed
#   in as TARGETS
# :type TARGET_FILES: list of files
# :param ARGN: the same arguments as the CMake install command except that
#   keywords identifying the kind of type and the DESTINATION keyword must be
#   joined with an underscore, e.g. ARCHIVE_DESTINATION.
# :type ARGN: various
#
function(ament_cmake_symlink_install_targets)
  cmake_parse_arguments(ARG "OPTIONAL" "ARCHIVE_DESTINATION;DESTINATION;LIBRARY_DESTINATION;RUNTIME_DESTINATION"
    "TARGETS;TARGET_FILES" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_cmake_symlink_install_targets() called with "
      "unused/unsupported arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # iterate over target files
  foreach(file ${ARG_TARGET_FILES})
    if(NOT IS_ABSOLUTE "${file}")
      message(FATAL_ERROR "ament_cmake_symlink_install_targets() target file "
        "'${file}' must be an absolute path")
    endif()

    # determine destination of file based on extension
    set(destination "")
    get_filename_component(fileext "${file}" EXT)
    if(fileext STREQUAL ".a" OR fileext STREQUAL ".lib")
      set(destination "${ARG_ARCHIVE_DESTINATION}")
    elseif(fileext STREQUAL ".dylib" OR fileext MATCHES "\\.so(\\.[0-9]+)?(\\.[0-9]+)?(\\.[0-9]+)?$")
      set(destination "${ARG_LIBRARY_DESTINATION}")
    elseif(fileext STREQUAL "" OR fileext STREQUAL ".dll" OR fileext STREQUAL ".exe")
      set(destination "${ARG_RUNTIME_DESTINATION}")
    endif()
    if(destination STREQUAL "")
      set(destination "${ARG_DESTINATION}")
    endif()

    # make destination an absolute path and ensure that it exists
    if(NOT IS_ABSOLUTE "${destination}")
      set(destination "/home/pawn/px4_ros_ws/install/px4_msgs/${destination}")
    endif()
    if(NOT EXISTS "${destination}")
      file(MAKE_DIRECTORY "${destination}")
    endif()

    if(EXISTS "${file}")
      # determine link name for file including destination path
      get_filename_component(filename "${file}" NAME)
      set(symlink "${destination}/${filename}")
      _ament_cmake_symlink_install_create_symlink("${file}" "${symlink}")
    else()
      if(NOT ARG_OPTIONAL)
        message(FATAL_ERROR
          "ament_cmake_symlink_install_targets() can't find '${file}'")
      endif()
    endif()
  endforeach()
endfunction()

function(_ament_cmake_symlink_install_create_symlink absolute_file symlink)
  # register symlink for being removed during install step
  file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/symlink_install_manifest.txt"
    "${symlink}\n")

  # avoid any work if correct symlink is already in place
  if(EXISTS "${symlink}" AND IS_SYMLINK "${symlink}")
    get_filename_component(destination "${symlink}" REALPATH)
    get_filename_component(real_absolute_file "${absolute_file}" REALPATH)
    if(destination STREQUAL real_absolute_file)
      message(STATUS "Up-to-date symlink: ${symlink}")
      return()
    endif()
  endif()

  message(STATUS "Symlinking: ${symlink}")
  if(EXISTS "${symlink}" OR IS_SYMLINK "${symlink}")
    file(REMOVE "${symlink}")
  endif()

  execute_process(
    COMMAND "/usr/bin/cmake" "-E" "create_symlink"
      "${absolute_file}"
      "${symlink}"
  )
  # the CMake command does not provide a return code so check manually
  if(NOT EXISTS "${symlink}" OR NOT IS_SYMLINK "${symlink}")
    get_filename_component(destination "${symlink}" REALPATH)
    message(FATAL_ERROR
      "Could not create symlink '${symlink}' pointing to '${absolute_file}'")
  endif()
endfunction()

# end of template

message(STATUS "Execute custom install script")

# begin of custom install code

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/px4_msgs" "DESTINATION" "share/ament_index/resource_index/rosidl_interfaces")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/px4_msgs" "DESTINATION" "share/ament_index/resource_index/rosidl_interfaces")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ActionRequest.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ActionRequest.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ActuatorArmed.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ActuatorArmed.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ActuatorControlsStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ActuatorControlsStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ActuatorMotors.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ActuatorMotors.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ActuatorOutputs.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ActuatorOutputs.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ActuatorServos.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ActuatorServos.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ActuatorServosTrim.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ActuatorServosTrim.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ActuatorTest.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ActuatorTest.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/AdcReport.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/AdcReport.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Airspeed.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Airspeed.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/AirspeedValidated.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/AirspeedValidated.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/AirspeedWind.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/AirspeedWind.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ArmingCheckReply.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ArmingCheckReply.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ArmingCheckRequest.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ArmingCheckRequest.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/AutotuneAttitudeControlStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/AutotuneAttitudeControlStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/AuxGlobalPosition.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/AuxGlobalPosition.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/BatteryInfo.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/BatteryInfo.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/BatteryStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/BatteryStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ButtonEvent.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ButtonEvent.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/CameraCapture.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/CameraCapture.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/CameraStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/CameraStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/CameraTrigger.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/CameraTrigger.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/CanInterfaceStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/CanInterfaceStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/CellularStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/CellularStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/CollisionConstraints.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/CollisionConstraints.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ConfigOverrides.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ConfigOverrides.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ControlAllocatorStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ControlAllocatorStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Cpuload.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Cpuload.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DatamanRequest.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DatamanRequest.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DatamanResponse.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DatamanResponse.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DebugArray.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DebugArray.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DebugKeyValue.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DebugKeyValue.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DebugValue.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DebugValue.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DebugVect.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DebugVect.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DeviceInformation.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DeviceInformation.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DifferentialPressure.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DifferentialPressure.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DistanceSensor.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DistanceSensor.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DistanceSensorModeChangeRequest.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DistanceSensorModeChangeRequest.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DronecanNodeStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/DronecanNodeStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Ekf2Timestamps.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Ekf2Timestamps.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EscEepromRead.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EscEepromRead.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EscEepromWrite.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EscEepromWrite.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EscReport.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EscReport.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EscStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EscStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorAidSource1d.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorAidSource1d.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorAidSource2d.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorAidSource2d.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorAidSource3d.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorAidSource3d.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorBias.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorBias.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorBias3d.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorBias3d.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorEventFlags.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorEventFlags.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorFusionControl.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorFusionControl.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorGpsStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorGpsStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorInnovations.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorInnovations.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorSelectorStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorSelectorStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorSensorBias.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorSensorBias.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorStates.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorStates.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorStatusFlags.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/EstimatorStatusFlags.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Event.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Event.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FailsafeFlags.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FailsafeFlags.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FailureDetectorStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FailureDetectorStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FigureEightStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FigureEightStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FixedWingLateralGuidanceStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FixedWingLateralGuidanceStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FixedWingLateralSetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FixedWingLateralSetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FixedWingLateralStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FixedWingLateralStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FixedWingLongitudinalSetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FixedWingLongitudinalSetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FixedWingRunwayControl.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FixedWingRunwayControl.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FlightPhaseEstimation.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FlightPhaseEstimation.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FollowTarget.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FollowTarget.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FollowTargetEstimator.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FollowTargetEstimator.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FollowTargetStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FollowTargetStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FuelTankStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/FuelTankStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GainCompression.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GainCompression.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GeneratorStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GeneratorStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GeofenceResult.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GeofenceResult.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GeofenceStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GeofenceStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GimbalControls.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GimbalControls.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GimbalDeviceAttitudeStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GimbalDeviceAttitudeStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GimbalDeviceInformation.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GimbalDeviceInformation.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GimbalDeviceSetAttitude.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GimbalDeviceSetAttitude.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GimbalManagerInformation.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GimbalManagerInformation.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GimbalManagerSetAttitude.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GimbalManagerSetAttitude.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GimbalManagerSetManualControl.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GimbalManagerSetManualControl.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GimbalManagerStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GimbalManagerStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GotoSetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GotoSetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GpioConfig.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GpioConfig.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GpioIn.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GpioIn.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GpioOut.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GpioOut.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GpioRequest.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GpioRequest.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GpsDump.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GpsDump.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GpsInjectData.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/GpsInjectData.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Gripper.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Gripper.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/HealthReport.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/HealthReport.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/HeaterStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/HeaterStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/HomePosition.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/HomePosition.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/HoverThrustEstimate.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/HoverThrustEstimate.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/InputRc.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/InputRc.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/InternalCombustionEngineControl.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/InternalCombustionEngineControl.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/InternalCombustionEngineStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/InternalCombustionEngineStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/IridiumsbdStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/IridiumsbdStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/IrlockReport.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/IrlockReport.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LandingGear.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LandingGear.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LandingGearWheel.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LandingGearWheel.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LandingTargetInnovations.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LandingTargetInnovations.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LandingTargetPose.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LandingTargetPose.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LateralControlConfiguration.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LateralControlConfiguration.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LaunchDetectionStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LaunchDetectionStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LedControl.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LedControl.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LogMessage.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LogMessage.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LoggerStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LoggerStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LongitudinalControlConfiguration.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/LongitudinalControlConfiguration.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/MagWorkerData.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/MagWorkerData.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/MagnetometerBiasEstimate.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/MagnetometerBiasEstimate.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ManualControlSetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ManualControlSetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ManualControlSwitches.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ManualControlSwitches.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/MavlinkLog.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/MavlinkLog.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/MavlinkTunnel.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/MavlinkTunnel.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/MessageFormatRequest.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/MessageFormatRequest.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/MessageFormatResponse.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/MessageFormatResponse.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Mission.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Mission.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/MissionResult.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/MissionResult.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ModeCompleted.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ModeCompleted.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/MountOrientation.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/MountOrientation.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/NavigatorMissionItem.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/NavigatorMissionItem.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/NavigatorStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/NavigatorStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/NeuralControl.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/NeuralControl.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/NormalizedUnsignedSetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/NormalizedUnsignedSetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ObstacleDistance.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ObstacleDistance.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OffboardControlMode.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OffboardControlMode.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OnboardComputerStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OnboardComputerStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OpenDroneIdArmStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OpenDroneIdArmStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OpenDroneIdOperatorId.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OpenDroneIdOperatorId.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OpenDroneIdSelfId.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OpenDroneIdSelfId.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OpenDroneIdSystem.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OpenDroneIdSystem.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OrbTest.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OrbTest.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OrbTestLarge.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OrbTestLarge.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OrbTestMedium.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OrbTestMedium.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OrbitStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/OrbitStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ParameterResetRequest.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ParameterResetRequest.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ParameterSetUsedRequest.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ParameterSetUsedRequest.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ParameterSetValueRequest.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ParameterSetValueRequest.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ParameterSetValueResponse.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ParameterSetValueResponse.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ParameterUpdate.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/ParameterUpdate.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Ping.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Ping.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/PositionControllerLandingStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/PositionControllerLandingStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/PositionControllerStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/PositionControllerStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/PositionSetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/PositionSetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/PositionSetpointTriplet.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/PositionSetpointTriplet.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/PowerButtonState.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/PowerButtonState.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/PowerMonitor.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/PowerMonitor.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/PpsCapture.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/PpsCapture.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/PurePursuitStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/PurePursuitStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/PwmInput.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/PwmInput.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Px4ioStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Px4ioStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/QshellReq.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/QshellReq.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/QshellRetval.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/QshellRetval.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RadioStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RadioStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RangingBeacon.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RangingBeacon.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RaptorInput.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RaptorInput.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RaptorStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RaptorStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RateCtrlStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RateCtrlStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RcChannels.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RcChannels.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RcParameterMap.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RcParameterMap.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RegisterExtComponentReply.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RegisterExtComponentReply.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RegisterExtComponentRequest.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RegisterExtComponentRequest.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RoverAttitudeSetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RoverAttitudeSetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RoverAttitudeStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RoverAttitudeStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RoverPositionSetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RoverPositionSetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RoverRateSetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RoverRateSetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RoverRateStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RoverRateStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RoverSpeedSetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RoverSpeedSetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RoverSpeedStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RoverSpeedStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RoverSteeringSetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RoverSteeringSetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RoverThrottleSetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RoverThrottleSetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Rpm.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Rpm.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RtlStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RtlStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RtlTimeEstimate.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/RtlTimeEstimate.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SatelliteInfo.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SatelliteInfo.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorAccel.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorAccel.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorAccelFifo.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorAccelFifo.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorAirflow.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorAirflow.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorBaro.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorBaro.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorCombined.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorCombined.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorCorrection.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorCorrection.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorGnssRelative.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorGnssRelative.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorGnssStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorGnssStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorGps.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorGps.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorGyro.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorGyro.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorGyroFft.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorGyroFft.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorGyroFifo.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorGyroFifo.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorHygrometer.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorHygrometer.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorMag.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorMag.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorOpticalFlow.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorOpticalFlow.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorPreflightMag.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorPreflightMag.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorSelection.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorSelection.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorTemp.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorTemp.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorUwb.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorUwb.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorsStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorsStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorsStatusImu.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SensorsStatusImu.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SystemPower.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/SystemPower.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TakeoffStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TakeoffStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TaskStackInfo.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TaskStackInfo.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TecsStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TecsStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TelemetryStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TelemetryStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TiltrotorExtraControls.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TiltrotorExtraControls.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TimesyncStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TimesyncStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TrajectorySetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TrajectorySetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TrajectorySetpoint6dof.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TrajectorySetpoint6dof.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TransponderReport.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TransponderReport.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TuneControl.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/TuneControl.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/UavcanParameterRequest.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/UavcanParameterRequest.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/UavcanParameterValue.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/UavcanParameterValue.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/UlogStream.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/UlogStream.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/UlogStreamAck.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/UlogStreamAck.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/UnregisterExtComponent.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/UnregisterExtComponent.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleAcceleration.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleAcceleration.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleAirData.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleAirData.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleAngularAccelerationSetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleAngularAccelerationSetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleAngularVelocity.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleAngularVelocity.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleAttitude.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleAttitude.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleAttitudeSetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleAttitudeSetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleCommand.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleCommand.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleCommandAck.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleCommandAck.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleConstraints.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleConstraints.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleControlMode.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleControlMode.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleGlobalPosition.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleGlobalPosition.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleImu.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleImu.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleImuStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleImuStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleLandDetected.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleLandDetected.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleLocalPosition.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleLocalPosition.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleLocalPositionSetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleLocalPositionSetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleMagnetometer.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleMagnetometer.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleOdometry.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleOdometry.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleOpticalFlow.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleOpticalFlow.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleOpticalFlowVel.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleOpticalFlowVel.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleRatesSetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleRatesSetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleRoi.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleRoi.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleThrustSetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleThrustSetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleTorqueSetpoint.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VehicleTorqueSetpoint.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VelocityLimits.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VelocityLimits.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VtolVehicleStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/VtolVehicleStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Vtx.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Vtx.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/WheelEncoders.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/WheelEncoders.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Wind.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/Wind.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/YawEstimatorStatus.json" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/msg/YawEstimatorStatus.json" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/srv/VehicleCommand.json" "DESTINATION" "share/px4_msgs/srv")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_type_description/px4_msgs/srv/VehicleCommand.json" "DESTINATION" "share/px4_msgs/srv")

# install(DIRECTORY "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_c/px4_msgs/" "DESTINATION" "include/px4_msgs/px4_msgs" "PATTERN" "*.h")
ament_cmake_symlink_install_directory("/home/pawn/px4_ros_ws/src/px4_msgs" DIRECTORY "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_c/px4_msgs/" "DESTINATION" "include/px4_msgs/px4_msgs" "PATTERN" "*.h")

# install(FILES "/opt/ros/jazzy/lib/python3.12/site-packages/ament_package/template/environment_hook/library_path.sh" "DESTINATION" "share/px4_msgs/environment")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/opt/ros/jazzy/lib/python3.12/site-packages/ament_package/template/environment_hook/library_path.sh" "DESTINATION" "share/px4_msgs/environment")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/library_path.dsv" "DESTINATION" "share/px4_msgs/environment")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/library_path.dsv" "DESTINATION" "share/px4_msgs/environment")

# install(DIRECTORY "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_typesupport_fastrtps_c/px4_msgs/" "DESTINATION" "include/px4_msgs/px4_msgs" "PATTERN_EXCLUDE" "*.cpp")
ament_cmake_symlink_install_directory("/home/pawn/px4_ros_ws/src/px4_msgs" DIRECTORY "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_typesupport_fastrtps_c/px4_msgs/" "DESTINATION" "include/px4_msgs/px4_msgs" "PATTERN_EXCLUDE" "*.cpp")

# install(DIRECTORY "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_cpp/px4_msgs/" "DESTINATION" "include/px4_msgs/px4_msgs" "PATTERN" "*.hpp")
ament_cmake_symlink_install_directory("/home/pawn/px4_ros_ws/src/px4_msgs" DIRECTORY "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_cpp/px4_msgs/" "DESTINATION" "include/px4_msgs/px4_msgs" "PATTERN" "*.hpp")

# install(DIRECTORY "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_typesupport_fastrtps_cpp/px4_msgs/" "DESTINATION" "include/px4_msgs/px4_msgs" "PATTERN_EXCLUDE" "*.cpp")
ament_cmake_symlink_install_directory("/home/pawn/px4_ros_ws/src/px4_msgs" DIRECTORY "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_typesupport_fastrtps_cpp/px4_msgs/" "DESTINATION" "include/px4_msgs/px4_msgs" "PATTERN_EXCLUDE" "*.cpp")

# install(DIRECTORY "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_typesupport_introspection_c/px4_msgs/" "DESTINATION" "include/px4_msgs/px4_msgs" "PATTERN" "*.h")
ament_cmake_symlink_install_directory("/home/pawn/px4_ros_ws/src/px4_msgs" DIRECTORY "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_typesupport_introspection_c/px4_msgs/" "DESTINATION" "include/px4_msgs/px4_msgs" "PATTERN" "*.h")

# install(DIRECTORY "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_typesupport_introspection_cpp/px4_msgs/" "DESTINATION" "include/px4_msgs/px4_msgs" "PATTERN" "*.hpp")
ament_cmake_symlink_install_directory("/home/pawn/px4_ros_ws/src/px4_msgs" DIRECTORY "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_typesupport_introspection_cpp/px4_msgs/" "DESTINATION" "include/px4_msgs/px4_msgs" "PATTERN" "*.hpp")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/pythonpath.sh" "DESTINATION" "share/px4_msgs/environment")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/pythonpath.sh" "DESTINATION" "share/px4_msgs/environment")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/pythonpath.dsv" "DESTINATION" "share/px4_msgs/environment")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/pythonpath.dsv" "DESTINATION" "share/px4_msgs/environment")

# install(DIRECTORY "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_python/px4_msgs/px4_msgs.egg-info/" "DESTINATION" "lib/python3.12/site-packages/px4_msgs-2.0.1-py3.12.egg-info")
ament_cmake_symlink_install_directory("/home/pawn/px4_ros_ws/src/px4_msgs" DIRECTORY "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_python/px4_msgs/px4_msgs.egg-info/" "DESTINATION" "lib/python3.12/site-packages/px4_msgs-2.0.1-py3.12.egg-info")

# install(DIRECTORY "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_py/px4_msgs/" "DESTINATION" "lib/python3.12/site-packages/px4_msgs" "PATTERN_EXCLUDE" "*.pyc" "PATTERN_EXCLUDE" "__pycache__")
ament_cmake_symlink_install_directory("/home/pawn/px4_ros_ws/src/px4_msgs" DIRECTORY "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_generator_py/px4_msgs/" "DESTINATION" "lib/python3.12/site-packages/px4_msgs" "PATTERN_EXCLUDE" "*.pyc" "PATTERN_EXCLUDE" "__pycache__")

# install("TARGETS" "px4_msgs_s__rosidl_typesupport_fastrtps_c" "DESTINATION" "lib/python3.12/site-packages/px4_msgs")
include("/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_symlink_install_targets_0_${CMAKE_INSTALL_CONFIG_NAME}.cmake")

# install("TARGETS" "px4_msgs_s__rosidl_typesupport_introspection_c" "DESTINATION" "lib/python3.12/site-packages/px4_msgs")
include("/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_symlink_install_targets_1_${CMAKE_INSTALL_CONFIG_NAME}.cmake")

# install("TARGETS" "px4_msgs_s__rosidl_typesupport_c" "DESTINATION" "lib/python3.12/site-packages/px4_msgs")
include("/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_symlink_install_targets_2_${CMAKE_INSTALL_CONFIG_NAME}.cmake")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActionRequest.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActionRequest.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorArmed.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorArmed.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorControlsStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorControlsStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorMotors.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorMotors.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorOutputs.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorOutputs.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorServos.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorServos.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorServosTrim.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorServosTrim.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorTest.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ActuatorTest.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/AdcReport.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/AdcReport.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Airspeed.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Airspeed.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/AirspeedValidated.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/AirspeedValidated.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/AirspeedWind.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/AirspeedWind.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ArmingCheckReply.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ArmingCheckReply.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ArmingCheckRequest.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ArmingCheckRequest.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/AutotuneAttitudeControlStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/AutotuneAttitudeControlStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/AuxGlobalPosition.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/AuxGlobalPosition.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/BatteryInfo.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/BatteryInfo.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/BatteryStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/BatteryStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ButtonEvent.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ButtonEvent.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CameraCapture.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CameraCapture.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CameraStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CameraStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CameraTrigger.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CameraTrigger.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CanInterfaceStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CanInterfaceStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CellularStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CellularStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CollisionConstraints.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/CollisionConstraints.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ConfigOverrides.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ConfigOverrides.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ControlAllocatorStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ControlAllocatorStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Cpuload.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Cpuload.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DatamanRequest.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DatamanRequest.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DatamanResponse.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DatamanResponse.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DebugArray.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DebugArray.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DebugKeyValue.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DebugKeyValue.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DebugValue.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DebugValue.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DebugVect.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DebugVect.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DeviceInformation.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DeviceInformation.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DifferentialPressure.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DifferentialPressure.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DistanceSensor.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DistanceSensor.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DistanceSensorModeChangeRequest.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DistanceSensorModeChangeRequest.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DronecanNodeStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/DronecanNodeStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Ekf2Timestamps.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Ekf2Timestamps.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EscEepromRead.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EscEepromRead.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EscEepromWrite.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EscEepromWrite.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EscReport.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EscReport.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EscStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EscStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorAidSource1d.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorAidSource1d.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorAidSource2d.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorAidSource2d.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorAidSource3d.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorAidSource3d.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorBias.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorBias.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorBias3d.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorBias3d.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorEventFlags.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorEventFlags.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorFusionControl.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorFusionControl.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorGpsStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorGpsStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorInnovations.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorInnovations.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorSelectorStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorSelectorStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorSensorBias.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorSensorBias.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorStates.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorStates.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorStatusFlags.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/EstimatorStatusFlags.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Event.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Event.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FailsafeFlags.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FailsafeFlags.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FailureDetectorStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FailureDetectorStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FigureEightStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FigureEightStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FixedWingLateralGuidanceStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FixedWingLateralGuidanceStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FixedWingLateralSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FixedWingLateralSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FixedWingLateralStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FixedWingLateralStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FixedWingLongitudinalSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FixedWingLongitudinalSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FixedWingRunwayControl.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FixedWingRunwayControl.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FlightPhaseEstimation.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FlightPhaseEstimation.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FollowTarget.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FollowTarget.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FollowTargetEstimator.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FollowTargetEstimator.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FollowTargetStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FollowTargetStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FuelTankStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/FuelTankStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GainCompression.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GainCompression.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GeneratorStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GeneratorStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GeofenceResult.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GeofenceResult.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GeofenceStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GeofenceStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalControls.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalControls.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalDeviceAttitudeStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalDeviceAttitudeStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalDeviceInformation.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalDeviceInformation.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalDeviceSetAttitude.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalDeviceSetAttitude.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalManagerInformation.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalManagerInformation.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalManagerSetAttitude.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalManagerSetAttitude.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalManagerSetManualControl.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalManagerSetManualControl.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalManagerStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GimbalManagerStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GotoSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GotoSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GpioConfig.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GpioConfig.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GpioIn.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GpioIn.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GpioOut.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GpioOut.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GpioRequest.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GpioRequest.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GpsDump.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GpsDump.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GpsInjectData.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/GpsInjectData.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Gripper.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Gripper.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/HealthReport.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/HealthReport.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/HeaterStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/HeaterStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/HomePosition.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/HomePosition.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/HoverThrustEstimate.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/HoverThrustEstimate.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/InputRc.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/InputRc.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/InternalCombustionEngineControl.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/InternalCombustionEngineControl.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/InternalCombustionEngineStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/InternalCombustionEngineStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/IridiumsbdStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/IridiumsbdStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/IrlockReport.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/IrlockReport.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LandingGear.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LandingGear.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LandingGearWheel.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LandingGearWheel.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LandingTargetInnovations.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LandingTargetInnovations.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LandingTargetPose.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LandingTargetPose.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LateralControlConfiguration.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LateralControlConfiguration.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LaunchDetectionStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LaunchDetectionStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LedControl.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LedControl.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LogMessage.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LogMessage.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LoggerStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LoggerStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LongitudinalControlConfiguration.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/LongitudinalControlConfiguration.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MagWorkerData.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MagWorkerData.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MagnetometerBiasEstimate.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MagnetometerBiasEstimate.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ManualControlSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ManualControlSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ManualControlSwitches.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ManualControlSwitches.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MavlinkLog.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MavlinkLog.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MavlinkTunnel.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MavlinkTunnel.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MessageFormatRequest.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MessageFormatRequest.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MessageFormatResponse.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MessageFormatResponse.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Mission.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Mission.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MissionResult.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MissionResult.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ModeCompleted.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ModeCompleted.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MountOrientation.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/MountOrientation.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/NavigatorMissionItem.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/NavigatorMissionItem.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/NavigatorStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/NavigatorStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/NeuralControl.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/NeuralControl.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/NormalizedUnsignedSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/NormalizedUnsignedSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ObstacleDistance.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ObstacleDistance.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OffboardControlMode.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OffboardControlMode.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OnboardComputerStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OnboardComputerStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OpenDroneIdArmStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OpenDroneIdArmStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OpenDroneIdOperatorId.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OpenDroneIdOperatorId.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OpenDroneIdSelfId.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OpenDroneIdSelfId.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OpenDroneIdSystem.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OpenDroneIdSystem.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OrbTest.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OrbTest.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OrbTestLarge.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OrbTestLarge.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OrbTestMedium.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OrbTestMedium.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OrbitStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/OrbitStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ParameterResetRequest.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ParameterResetRequest.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ParameterSetUsedRequest.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ParameterSetUsedRequest.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ParameterSetValueRequest.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ParameterSetValueRequest.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ParameterSetValueResponse.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ParameterSetValueResponse.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ParameterUpdate.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/ParameterUpdate.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Ping.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Ping.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PositionControllerLandingStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PositionControllerLandingStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PositionControllerStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PositionControllerStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PositionSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PositionSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PositionSetpointTriplet.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PositionSetpointTriplet.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PowerButtonState.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PowerButtonState.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PowerMonitor.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PowerMonitor.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PpsCapture.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PpsCapture.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PurePursuitStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PurePursuitStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PwmInput.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/PwmInput.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Px4ioStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Px4ioStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/QshellReq.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/QshellReq.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/QshellRetval.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/QshellRetval.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RadioStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RadioStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RangingBeacon.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RangingBeacon.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RaptorInput.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RaptorInput.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RaptorStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RaptorStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RateCtrlStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RateCtrlStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RcChannels.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RcChannels.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RcParameterMap.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RcParameterMap.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RegisterExtComponentReply.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RegisterExtComponentReply.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RegisterExtComponentRequest.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RegisterExtComponentRequest.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverAttitudeSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverAttitudeSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverAttitudeStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverAttitudeStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverPositionSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverPositionSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverRateSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverRateSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverRateStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverRateStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverSpeedSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverSpeedSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverSpeedStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverSpeedStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverSteeringSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverSteeringSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverThrottleSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RoverThrottleSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Rpm.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Rpm.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RtlStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RtlStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RtlTimeEstimate.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/RtlTimeEstimate.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SatelliteInfo.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SatelliteInfo.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorAccel.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorAccel.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorAccelFifo.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorAccelFifo.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorAirflow.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorAirflow.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorBaro.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorBaro.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorCombined.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorCombined.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorCorrection.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorCorrection.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorGnssRelative.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorGnssRelative.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorGnssStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorGnssStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorGps.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorGps.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorGyro.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorGyro.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorGyroFft.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorGyroFft.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorGyroFifo.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorGyroFifo.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorHygrometer.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorHygrometer.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorMag.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorMag.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorOpticalFlow.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorOpticalFlow.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorPreflightMag.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorPreflightMag.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorSelection.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorSelection.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorTemp.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorTemp.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorUwb.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorUwb.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorsStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorsStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorsStatusImu.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SensorsStatusImu.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SystemPower.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/SystemPower.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TakeoffStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TakeoffStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TaskStackInfo.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TaskStackInfo.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TecsStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TecsStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TelemetryStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TelemetryStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TiltrotorExtraControls.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TiltrotorExtraControls.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TimesyncStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TimesyncStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TrajectorySetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TrajectorySetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TrajectorySetpoint6dof.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TrajectorySetpoint6dof.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TransponderReport.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TransponderReport.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TuneControl.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/TuneControl.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/UavcanParameterRequest.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/UavcanParameterRequest.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/UavcanParameterValue.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/UavcanParameterValue.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/UlogStream.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/UlogStream.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/UlogStreamAck.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/UlogStreamAck.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/UnregisterExtComponent.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/UnregisterExtComponent.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleAcceleration.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleAcceleration.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleAirData.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleAirData.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleAngularAccelerationSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleAngularAccelerationSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleAngularVelocity.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleAngularVelocity.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleAttitude.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleAttitude.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleAttitudeSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleAttitudeSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleCommand.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleCommand.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleCommandAck.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleCommandAck.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleConstraints.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleConstraints.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleControlMode.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleControlMode.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleGlobalPosition.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleGlobalPosition.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleImu.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleImu.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleImuStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleImuStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleLandDetected.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleLandDetected.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleLocalPosition.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleLocalPosition.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleLocalPositionSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleLocalPositionSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleMagnetometer.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleMagnetometer.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleOdometry.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleOdometry.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleOpticalFlow.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleOpticalFlow.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleOpticalFlowVel.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleOpticalFlowVel.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleRatesSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleRatesSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleRoi.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleRoi.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleThrustSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleThrustSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleTorqueSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VehicleTorqueSetpoint.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VelocityLimits.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VelocityLimits.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VtolVehicleStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/VtolVehicleStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Vtx.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Vtx.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/WheelEncoders.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/WheelEncoders.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Wind.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/Wind.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/YawEstimatorStatus.idl" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/msg/YawEstimatorStatus.idl" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/srv/VehicleCommand.idl" "DESTINATION" "share/px4_msgs/srv")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_adapter/px4_msgs/srv/VehicleCommand.idl" "DESTINATION" "share/px4_msgs/srv")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ActionRequest.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ActionRequest.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ActuatorArmed.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ActuatorArmed.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ActuatorControlsStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ActuatorControlsStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ActuatorMotors.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ActuatorMotors.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ActuatorOutputs.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ActuatorOutputs.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ActuatorServos.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ActuatorServos.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ActuatorServosTrim.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ActuatorServosTrim.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ActuatorTest.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ActuatorTest.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/AdcReport.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/AdcReport.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Airspeed.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Airspeed.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/AirspeedValidated.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/AirspeedValidated.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/AirspeedWind.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/AirspeedWind.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ArmingCheckReply.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ArmingCheckReply.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ArmingCheckRequest.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ArmingCheckRequest.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/AutotuneAttitudeControlStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/AutotuneAttitudeControlStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/AuxGlobalPosition.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/AuxGlobalPosition.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/BatteryInfo.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/BatteryInfo.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/BatteryStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/BatteryStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ButtonEvent.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ButtonEvent.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/CameraCapture.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/CameraCapture.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/CameraStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/CameraStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/CameraTrigger.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/CameraTrigger.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/CanInterfaceStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/CanInterfaceStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/CellularStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/CellularStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/CollisionConstraints.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/CollisionConstraints.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ConfigOverrides.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ConfigOverrides.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ControlAllocatorStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ControlAllocatorStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Cpuload.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Cpuload.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DatamanRequest.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DatamanRequest.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DatamanResponse.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DatamanResponse.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DebugArray.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DebugArray.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DebugKeyValue.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DebugKeyValue.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DebugValue.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DebugValue.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DebugVect.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DebugVect.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DeviceInformation.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DeviceInformation.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DifferentialPressure.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DifferentialPressure.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DistanceSensor.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DistanceSensor.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DistanceSensorModeChangeRequest.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DistanceSensorModeChangeRequest.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DronecanNodeStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/DronecanNodeStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Ekf2Timestamps.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Ekf2Timestamps.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EscEepromRead.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EscEepromRead.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EscEepromWrite.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EscEepromWrite.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EscReport.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EscReport.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EscStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EscStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorAidSource1d.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorAidSource1d.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorAidSource2d.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorAidSource2d.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorAidSource3d.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorAidSource3d.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorBias.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorBias.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorBias3d.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorBias3d.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorEventFlags.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorEventFlags.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorFusionControl.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorFusionControl.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorGpsStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorGpsStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorInnovations.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorInnovations.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorSelectorStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorSelectorStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorSensorBias.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorSensorBias.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorStates.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorStates.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorStatusFlags.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/EstimatorStatusFlags.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Event.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Event.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FailsafeFlags.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FailsafeFlags.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FailureDetectorStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FailureDetectorStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FigureEightStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FigureEightStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FixedWingLateralGuidanceStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FixedWingLateralGuidanceStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FixedWingLateralSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FixedWingLateralSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FixedWingLateralStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FixedWingLateralStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FixedWingLongitudinalSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FixedWingLongitudinalSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FixedWingRunwayControl.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FixedWingRunwayControl.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FlightPhaseEstimation.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FlightPhaseEstimation.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FollowTarget.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FollowTarget.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FollowTargetEstimator.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FollowTargetEstimator.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FollowTargetStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FollowTargetStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FuelTankStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/FuelTankStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GainCompression.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GainCompression.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GeneratorStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GeneratorStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GeofenceResult.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GeofenceResult.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GeofenceStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GeofenceStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GimbalControls.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GimbalControls.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GimbalDeviceAttitudeStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GimbalDeviceAttitudeStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GimbalDeviceInformation.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GimbalDeviceInformation.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GimbalDeviceSetAttitude.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GimbalDeviceSetAttitude.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GimbalManagerInformation.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GimbalManagerInformation.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GimbalManagerSetAttitude.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GimbalManagerSetAttitude.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GimbalManagerSetManualControl.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GimbalManagerSetManualControl.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GimbalManagerStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GimbalManagerStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GotoSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GotoSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GpioConfig.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GpioConfig.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GpioIn.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GpioIn.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GpioOut.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GpioOut.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GpioRequest.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GpioRequest.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GpsDump.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GpsDump.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GpsInjectData.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/GpsInjectData.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Gripper.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Gripper.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/HealthReport.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/HealthReport.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/HeaterStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/HeaterStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/HomePosition.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/HomePosition.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/HoverThrustEstimate.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/HoverThrustEstimate.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/InputRc.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/InputRc.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/InternalCombustionEngineControl.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/InternalCombustionEngineControl.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/InternalCombustionEngineStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/InternalCombustionEngineStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/IridiumsbdStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/IridiumsbdStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/IrlockReport.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/IrlockReport.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LandingGear.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LandingGear.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LandingGearWheel.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LandingGearWheel.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LandingTargetInnovations.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LandingTargetInnovations.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LandingTargetPose.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LandingTargetPose.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LateralControlConfiguration.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LateralControlConfiguration.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LaunchDetectionStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LaunchDetectionStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LedControl.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LedControl.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LogMessage.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LogMessage.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LoggerStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LoggerStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LongitudinalControlConfiguration.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/LongitudinalControlConfiguration.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/MagWorkerData.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/MagWorkerData.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/MagnetometerBiasEstimate.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/MagnetometerBiasEstimate.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ManualControlSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ManualControlSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ManualControlSwitches.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ManualControlSwitches.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/MavlinkLog.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/MavlinkLog.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/MavlinkTunnel.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/MavlinkTunnel.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/MessageFormatRequest.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/MessageFormatRequest.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/MessageFormatResponse.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/MessageFormatResponse.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Mission.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Mission.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/MissionResult.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/MissionResult.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ModeCompleted.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ModeCompleted.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/MountOrientation.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/MountOrientation.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/NavigatorMissionItem.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/NavigatorMissionItem.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/NavigatorStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/NavigatorStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/NeuralControl.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/NeuralControl.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/NormalizedUnsignedSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/NormalizedUnsignedSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ObstacleDistance.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ObstacleDistance.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OffboardControlMode.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OffboardControlMode.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OnboardComputerStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OnboardComputerStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OpenDroneIdArmStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OpenDroneIdArmStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OpenDroneIdOperatorId.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OpenDroneIdOperatorId.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OpenDroneIdSelfId.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OpenDroneIdSelfId.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OpenDroneIdSystem.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OpenDroneIdSystem.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OrbTest.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OrbTest.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OrbTestLarge.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OrbTestLarge.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OrbTestMedium.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OrbTestMedium.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OrbitStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/OrbitStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ParameterResetRequest.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ParameterResetRequest.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ParameterSetUsedRequest.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ParameterSetUsedRequest.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ParameterSetValueRequest.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ParameterSetValueRequest.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ParameterSetValueResponse.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ParameterSetValueResponse.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ParameterUpdate.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/ParameterUpdate.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Ping.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Ping.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/PositionControllerLandingStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/PositionControllerLandingStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/PositionControllerStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/PositionControllerStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/PositionSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/PositionSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/PositionSetpointTriplet.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/PositionSetpointTriplet.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/PowerButtonState.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/PowerButtonState.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/PowerMonitor.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/PowerMonitor.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/PpsCapture.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/PpsCapture.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/PurePursuitStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/PurePursuitStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/PwmInput.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/PwmInput.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Px4ioStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Px4ioStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/QshellReq.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/QshellReq.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/QshellRetval.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/QshellRetval.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RadioStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RadioStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RangingBeacon.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RangingBeacon.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RaptorInput.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RaptorInput.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RaptorStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RaptorStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RateCtrlStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RateCtrlStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RcChannels.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RcChannels.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RcParameterMap.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RcParameterMap.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RegisterExtComponentReply.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RegisterExtComponentReply.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RegisterExtComponentRequest.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RegisterExtComponentRequest.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RoverAttitudeSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RoverAttitudeSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RoverAttitudeStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RoverAttitudeStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RoverPositionSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RoverPositionSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RoverRateSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RoverRateSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RoverRateStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RoverRateStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RoverSpeedSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RoverSpeedSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RoverSpeedStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RoverSpeedStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RoverSteeringSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RoverSteeringSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RoverThrottleSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RoverThrottleSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Rpm.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Rpm.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RtlStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RtlStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RtlTimeEstimate.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/RtlTimeEstimate.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SatelliteInfo.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SatelliteInfo.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorAccel.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorAccel.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorAccelFifo.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorAccelFifo.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorAirflow.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorAirflow.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorBaro.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorBaro.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorCombined.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorCombined.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorCorrection.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorCorrection.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorGnssRelative.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorGnssRelative.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorGnssStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorGnssStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorGps.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorGps.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorGyro.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorGyro.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorGyroFft.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorGyroFft.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorGyroFifo.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorGyroFifo.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorHygrometer.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorHygrometer.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorMag.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorMag.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorOpticalFlow.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorOpticalFlow.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorPreflightMag.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorPreflightMag.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorSelection.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorSelection.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorTemp.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorTemp.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorUwb.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorUwb.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorsStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorsStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorsStatusImu.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SensorsStatusImu.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SystemPower.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/SystemPower.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TakeoffStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TakeoffStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TaskStackInfo.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TaskStackInfo.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TecsStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TecsStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TelemetryStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TelemetryStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TiltrotorExtraControls.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TiltrotorExtraControls.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TimesyncStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TimesyncStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TrajectorySetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TrajectorySetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TrajectorySetpoint6dof.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TrajectorySetpoint6dof.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TransponderReport.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TransponderReport.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TuneControl.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/TuneControl.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/UavcanParameterRequest.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/UavcanParameterRequest.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/UavcanParameterValue.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/UavcanParameterValue.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/UlogStream.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/UlogStream.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/UlogStreamAck.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/UlogStreamAck.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/UnregisterExtComponent.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/UnregisterExtComponent.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleAcceleration.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleAcceleration.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleAirData.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleAirData.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleAngularAccelerationSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleAngularAccelerationSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleAngularVelocity.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleAngularVelocity.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleAttitude.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleAttitude.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleAttitudeSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleAttitudeSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleCommand.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleCommand.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleCommandAck.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleCommandAck.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleConstraints.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleConstraints.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleControlMode.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleControlMode.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleGlobalPosition.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleGlobalPosition.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleImu.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleImu.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleImuStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleImuStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleLandDetected.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleLandDetected.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleLocalPosition.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleLocalPosition.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleLocalPositionSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleLocalPositionSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleMagnetometer.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleMagnetometer.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleOdometry.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleOdometry.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleOpticalFlow.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleOpticalFlow.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleOpticalFlowVel.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleOpticalFlowVel.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleRatesSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleRatesSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleRoi.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleRoi.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleThrustSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleThrustSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleTorqueSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VehicleTorqueSetpoint.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VelocityLimits.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VelocityLimits.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VtolVehicleStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/VtolVehicleStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Vtx.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Vtx.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/WheelEncoders.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/WheelEncoders.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Wind.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/Wind.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/YawEstimatorStatus.msg" "DESTINATION" "share/px4_msgs/msg")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/msg/YawEstimatorStatus.msg" "DESTINATION" "share/px4_msgs/msg")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/srv/VehicleCommand.srv" "DESTINATION" "share/px4_msgs/srv")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/srv/VehicleCommand.srv" "DESTINATION" "share/px4_msgs/srv")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/px4_msgs" "DESTINATION" "share/ament_index/resource_index/package_run_dependencies")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/px4_msgs" "DESTINATION" "share/ament_index/resource_index/package_run_dependencies")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/px4_msgs" "DESTINATION" "share/ament_index/resource_index/parent_prefix_path")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/px4_msgs" "DESTINATION" "share/ament_index/resource_index/parent_prefix_path")

# install(FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh" "DESTINATION" "share/px4_msgs/environment")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh" "DESTINATION" "share/px4_msgs/environment")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/ament_prefix_path.dsv" "DESTINATION" "share/px4_msgs/environment")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/ament_prefix_path.dsv" "DESTINATION" "share/px4_msgs/environment")

# install(FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh" "DESTINATION" "share/px4_msgs/environment")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh" "DESTINATION" "share/px4_msgs/environment")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/path.dsv" "DESTINATION" "share/px4_msgs/environment")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/path.dsv" "DESTINATION" "share/px4_msgs/environment")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/local_setup.bash" "DESTINATION" "share/px4_msgs")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/local_setup.bash" "DESTINATION" "share/px4_msgs")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/local_setup.sh" "DESTINATION" "share/px4_msgs")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/local_setup.sh" "DESTINATION" "share/px4_msgs")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/local_setup.zsh" "DESTINATION" "share/px4_msgs")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/local_setup.zsh" "DESTINATION" "share/px4_msgs")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/local_setup.dsv" "DESTINATION" "share/px4_msgs")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/local_setup.dsv" "DESTINATION" "share/px4_msgs")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/package.dsv" "DESTINATION" "share/px4_msgs")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_environment_hooks/package.dsv" "DESTINATION" "share/px4_msgs")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_index/share/ament_index/resource_index/packages/px4_msgs" "DESTINATION" "share/ament_index/resource_index/packages")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_index/share/ament_index/resource_index/packages/px4_msgs" "DESTINATION" "share/ament_index/resource_index/packages")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_cmake/rosidl_cmake-extras.cmake" "DESTINATION" "share/px4_msgs/cmake")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_cmake/rosidl_cmake-extras.cmake" "DESTINATION" "share/px4_msgs/cmake")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake" "DESTINATION" "share/px4_msgs/cmake")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake" "DESTINATION" "share/px4_msgs/cmake")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake" "DESTINATION" "share/px4_msgs/cmake")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake" "DESTINATION" "share/px4_msgs/cmake")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake" "DESTINATION" "share/px4_msgs/cmake")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake" "DESTINATION" "share/px4_msgs/cmake")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake" "DESTINATION" "share/px4_msgs/cmake")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake" "DESTINATION" "share/px4_msgs/cmake")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake" "DESTINATION" "share/px4_msgs/cmake")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake" "DESTINATION" "share/px4_msgs/cmake")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake" "DESTINATION" "share/px4_msgs/cmake")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake" "DESTINATION" "share/px4_msgs/cmake")

# install(FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_core/px4_msgsConfig.cmake" "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_core/px4_msgsConfig-version.cmake" "DESTINATION" "share/px4_msgs/cmake")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_core/px4_msgsConfig.cmake" "/home/pawn/px4_ros_ws/build/px4_msgs/ament_cmake_core/px4_msgsConfig-version.cmake" "DESTINATION" "share/px4_msgs/cmake")

# install(FILES "/home/pawn/px4_ros_ws/src/px4_msgs/package.xml" "DESTINATION" "share/px4_msgs")
ament_cmake_symlink_install_files("/home/pawn/px4_ros_ws/src/px4_msgs" FILES "/home/pawn/px4_ros_ws/src/px4_msgs/package.xml" "DESTINATION" "share/px4_msgs")
