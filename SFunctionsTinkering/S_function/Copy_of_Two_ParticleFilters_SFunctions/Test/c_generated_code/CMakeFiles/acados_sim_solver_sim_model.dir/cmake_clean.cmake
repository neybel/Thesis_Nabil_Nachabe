file(REMOVE_RECURSE
  "acados_sim_solver_sim_model.dll"
  "acados_sim_solver_sim_model.dll.manifest"
  "acados_sim_solver_sim_model.lib"
  "acados_sim_solver_sim_model.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang C)
  include(CMakeFiles/acados_sim_solver_sim_model.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
