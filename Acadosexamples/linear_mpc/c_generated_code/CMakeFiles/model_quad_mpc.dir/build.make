# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.29

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\CMake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\user\acados\examples\acados_matlab_octave\linear_mpc\c_generated_code

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\user\acados\examples\acados_matlab_octave\linear_mpc\c_generated_code

# Include any dependencies generated for this target.
include CMakeFiles/model_quad_mpc.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/model_quad_mpc.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/model_quad_mpc.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/model_quad_mpc.dir/flags.make

CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun.c.obj: CMakeFiles/model_quad_mpc.dir/flags.make
CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun.c.obj: CMakeFiles/model_quad_mpc.dir/includes_C.rsp
CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun.c.obj: quad_mpc_model/quad_mpc_dyn_disc_phi_fun.c
CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun.c.obj: CMakeFiles/model_quad_mpc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=C:\Users\user\acados\examples\acados_matlab_octave\linear_mpc\c_generated_code\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun.c.obj"
	C:\ProgramData\MATLAB\SupportPackages\R2022b\3P.instrset\mingw_w64.instrset\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun.c.obj -MF CMakeFiles\model_quad_mpc.dir\quad_mpc_model\quad_mpc_dyn_disc_phi_fun.c.obj.d -o CMakeFiles\model_quad_mpc.dir\quad_mpc_model\quad_mpc_dyn_disc_phi_fun.c.obj -c C:\Users\user\acados\examples\acados_matlab_octave\linear_mpc\c_generated_code\quad_mpc_model\quad_mpc_dyn_disc_phi_fun.c

CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun.c.i"
	C:\ProgramData\MATLAB\SupportPackages\R2022b\3P.instrset\mingw_w64.instrset\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\user\acados\examples\acados_matlab_octave\linear_mpc\c_generated_code\quad_mpc_model\quad_mpc_dyn_disc_phi_fun.c > CMakeFiles\model_quad_mpc.dir\quad_mpc_model\quad_mpc_dyn_disc_phi_fun.c.i

CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun.c.s"
	C:\ProgramData\MATLAB\SupportPackages\R2022b\3P.instrset\mingw_w64.instrset\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\user\acados\examples\acados_matlab_octave\linear_mpc\c_generated_code\quad_mpc_model\quad_mpc_dyn_disc_phi_fun.c -o CMakeFiles\model_quad_mpc.dir\quad_mpc_model\quad_mpc_dyn_disc_phi_fun.c.s

CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun_jac.c.obj: CMakeFiles/model_quad_mpc.dir/flags.make
CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun_jac.c.obj: CMakeFiles/model_quad_mpc.dir/includes_C.rsp
CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun_jac.c.obj: quad_mpc_model/quad_mpc_dyn_disc_phi_fun_jac.c
CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun_jac.c.obj: CMakeFiles/model_quad_mpc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=C:\Users\user\acados\examples\acados_matlab_octave\linear_mpc\c_generated_code\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun_jac.c.obj"
	C:\ProgramData\MATLAB\SupportPackages\R2022b\3P.instrset\mingw_w64.instrset\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun_jac.c.obj -MF CMakeFiles\model_quad_mpc.dir\quad_mpc_model\quad_mpc_dyn_disc_phi_fun_jac.c.obj.d -o CMakeFiles\model_quad_mpc.dir\quad_mpc_model\quad_mpc_dyn_disc_phi_fun_jac.c.obj -c C:\Users\user\acados\examples\acados_matlab_octave\linear_mpc\c_generated_code\quad_mpc_model\quad_mpc_dyn_disc_phi_fun_jac.c

CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun_jac.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun_jac.c.i"
	C:\ProgramData\MATLAB\SupportPackages\R2022b\3P.instrset\mingw_w64.instrset\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E C:\Users\user\acados\examples\acados_matlab_octave\linear_mpc\c_generated_code\quad_mpc_model\quad_mpc_dyn_disc_phi_fun_jac.c > CMakeFiles\model_quad_mpc.dir\quad_mpc_model\quad_mpc_dyn_disc_phi_fun_jac.c.i

CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun_jac.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun_jac.c.s"
	C:\ProgramData\MATLAB\SupportPackages\R2022b\3P.instrset\mingw_w64.instrset\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S C:\Users\user\acados\examples\acados_matlab_octave\linear_mpc\c_generated_code\quad_mpc_model\quad_mpc_dyn_disc_phi_fun_jac.c -o CMakeFiles\model_quad_mpc.dir\quad_mpc_model\quad_mpc_dyn_disc_phi_fun_jac.c.s

model_quad_mpc: CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun.c.obj
model_quad_mpc: CMakeFiles/model_quad_mpc.dir/quad_mpc_model/quad_mpc_dyn_disc_phi_fun_jac.c.obj
model_quad_mpc: CMakeFiles/model_quad_mpc.dir/build.make
.PHONY : model_quad_mpc

# Rule to build all files generated by this target.
CMakeFiles/model_quad_mpc.dir/build: model_quad_mpc
.PHONY : CMakeFiles/model_quad_mpc.dir/build

CMakeFiles/model_quad_mpc.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\model_quad_mpc.dir\cmake_clean.cmake
.PHONY : CMakeFiles/model_quad_mpc.dir/clean

CMakeFiles/model_quad_mpc.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\user\acados\examples\acados_matlab_octave\linear_mpc\c_generated_code C:\Users\user\acados\examples\acados_matlab_octave\linear_mpc\c_generated_code C:\Users\user\acados\examples\acados_matlab_octave\linear_mpc\c_generated_code C:\Users\user\acados\examples\acados_matlab_octave\linear_mpc\c_generated_code C:\Users\user\acados\examples\acados_matlab_octave\linear_mpc\c_generated_code\CMakeFiles\model_quad_mpc.dir\DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/model_quad_mpc.dir/depend
