FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/re_2dmap_extractor/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/RequestLocMap.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_RequestLocMap.lisp"
  "../srv_gen/lisp/RequestNavMap.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_RequestNavMap.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
