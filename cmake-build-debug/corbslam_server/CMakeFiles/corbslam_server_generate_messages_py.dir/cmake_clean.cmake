FILE(REMOVE_RECURSE
  "CMakeFiles/corbslam_server_generate_messages_py"
  "../devel/lib/python2.7/dist-packages/corbslam_server/msg/_corbslam_message.py"
  "../devel/lib/python2.7/dist-packages/corbslam_server/srv/_corbslam_insert.py"
  "../devel/lib/python2.7/dist-packages/corbslam_server/srv/_corbslam_update.py"
  "../devel/lib/python2.7/dist-packages/corbslam_server/msg/__init__.py"
  "../devel/lib/python2.7/dist-packages/corbslam_server/srv/__init__.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/corbslam_server_generate_messages_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
