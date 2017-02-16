FILE(REMOVE_RECURSE
  "CMakeFiles/clearAll"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/clearAll.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
