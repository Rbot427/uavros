FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/uav_msgs/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/ControllerCommand.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_ControllerCommand.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
