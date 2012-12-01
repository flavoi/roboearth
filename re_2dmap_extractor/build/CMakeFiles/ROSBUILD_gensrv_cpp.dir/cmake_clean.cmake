FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/re_2dmap_extractor/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/re_2dmap_extractor/RequestLocMap.h"
  "../srv_gen/cpp/include/re_2dmap_extractor/RequestNavMap.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
