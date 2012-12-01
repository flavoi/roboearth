FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/re_2dmap_extractor/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/re_2dmap_extractor/srv/__init__.py"
  "../src/re_2dmap_extractor/srv/_RequestLocMap.py"
  "../src/re_2dmap_extractor/srv/_RequestNavMap.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
