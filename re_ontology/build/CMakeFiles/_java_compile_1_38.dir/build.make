# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/build

# Utility rule file for _java_compile_1_38.

# Include the progress variables for this target.
include CMakeFiles/_java_compile_1_38.dir/progress.make

CMakeFiles/_java_compile_1_38: ../bin/org/roboearth/re_ontology/ReVisionROSClient.class
CMakeFiles/_java_compile_1_38: ../bin/org/roboearth/re_ontology/ReVisionDummyPublisher.class

../bin/org/roboearth/re_ontology/ReVisionROSClient.class: ../src/org/roboearth/re_ontology/ReVisionROSClient.java
../bin/org/roboearth/re_ontology/ReVisionROSClient.class: ../src/org/roboearth/re_ontology/ReVisionDummyPublisher.java
	$(CMAKE_COMMAND) -E cmake_progress_report /home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../bin/org/roboearth/re_ontology/ReVisionROSClient.class, ../bin/org/roboearth/re_ontology/ReVisionDummyPublisher.class"
	cd /home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/src && /usr/lib/jvm/java-6-openjdk/bin/javac -source 1.5 -classpath /home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/msg_gen/java:/home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/srv_gen/java:/opt/ros/fuerte/stacks/client_rosjava_jni/rosjava_jni/bin:/home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/bin:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/3rdparty/jpl/jpl/jpl.jar:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/ias_prolog_addons/lib/classifier_interface.jar:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/ias_prolog_addons/lib/mallet-deps.jar:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/ias_prolog_addons/lib/mallet-dist.jar:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/ias_prolog_addons/lib/weka_fipm.jar:/opt/ros/fuerte/stacks/client_rosjava_jni/rosjava_jni/bin:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/knowrob_actions/bin:/usr/share/java/json-lib.jar:/usr/share/java/commons-collections3.jar:/usr/share/java/ezmorph.jar:/usr/share/java/commons-beanutils.jar:/usr/share/java/commons-lang.jar:/usr/share/java/commons-logging.jar:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/knowrob_common/bin:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/knowrob_common/lib/owlapi-bin.jar:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/knowrob_common/lib/vecmath.jar:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/knowrob_common/lib/guava-r09.jar:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/knowrob_common/lib/commons-lang-2.6.jar:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/knowrob_common/lib/HermiT.jar:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/tf_prolog/bin:/opt/ros/fuerte/stacks/client_rosjava_jni/tfjava/bin:/opt/ros/fuerte/stacks/client_rosjava_jni/tfjava/lib/vecmath.jar:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/comp_cop/bin:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/comp_cop/lib/apache-mime4j-0.6.jar:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/comp_cop/lib/commons-codec-1.3.jar:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/comp_cop/lib/commons-logging-1.1.1.jar:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/comp_cop/lib/google-api-translate-java-0.92.jar:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/comp_cop/lib/httpclient-4.0.1.jar:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/comp_cop/lib/httpcore-4.0.1.jar:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/comp_cop/lib/httpmime-4.0.1.jar:/home/flavio/Works/Roboearth_iaslab/stacks/knowrob/comp_cop/lib/xmlrpc-1.1.1.jar:/opt/ros/fuerte/stacks/client_rosjava_jni/rosjava_jni/bin: -d /home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/bin /home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/src/org/roboearth/re_ontology/ReVisionROSClient.java /home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/src/org/roboearth/re_ontology/ReVisionDummyPublisher.java

../bin/org/roboearth/re_ontology/ReVisionDummyPublisher.class: ../bin/org/roboearth/re_ontology/ReVisionROSClient.class

_java_compile_1_38: CMakeFiles/_java_compile_1_38
_java_compile_1_38: ../bin/org/roboearth/re_ontology/ReVisionROSClient.class
_java_compile_1_38: ../bin/org/roboearth/re_ontology/ReVisionDummyPublisher.class
_java_compile_1_38: CMakeFiles/_java_compile_1_38.dir/build.make
.PHONY : _java_compile_1_38

# Rule to build all files generated by this target.
CMakeFiles/_java_compile_1_38.dir/build: _java_compile_1_38
.PHONY : CMakeFiles/_java_compile_1_38.dir/build

CMakeFiles/_java_compile_1_38.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/_java_compile_1_38.dir/cmake_clean.cmake
.PHONY : CMakeFiles/_java_compile_1_38.dir/clean

CMakeFiles/_java_compile_1_38.dir/depend:
	cd /home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology /home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology /home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/build /home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/build /home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_ontology/build/CMakeFiles/_java_compile_1_38.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/_java_compile_1_38.dir/depend

