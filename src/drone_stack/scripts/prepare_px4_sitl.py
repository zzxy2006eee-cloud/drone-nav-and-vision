#!/usr/bin/env python3
"""Install this package's model and airframe into the pinned PX4 checkout."""
from pathlib import Path
import shutil
import subprocess


PACKAGE = Path(__file__).resolve().parents[1]
PX4 = PACKAGE.parents[1] / 'external' / 'PX4-Autopilot'
GAZEBO = PX4 / 'Tools/simulation/gazebo-classic/sitl_gazebo-classic'


def replace_once(path, old, new, marker):
    content = path.read_text()
    if marker in content:
        return
    if old not in content:
        raise SystemExit('PX4 v1.15.4 source layout changed: ' + str(path))
    path.write_text(content.replace(old, new, 1))


def main():
    if not (PX4 / 'CMakeLists.txt').exists() or not (GAZEBO / 'CMakeLists.txt').exists():
        raise SystemExit('PX4 v1.15.4 and Gazebo Classic sources are required under external/PX4-Autopilot')
    subprocess.run(['python3', str(PACKAGE / 'scripts/generate_model.py')], check=True)
    model_dest = GAZEBO / 'models/inspection_quad'
    model_dest.mkdir(parents=True, exist_ok=True)
    for name in ('inspection_quad.sdf.jinja', 'model.config'):
        shutil.copy2(PACKAGE / 'models/inspection_quad' / name, model_dest / name)
    shutil.copy2(PACKAGE / 'config/10052_gazebo-classic_inspection_quad',
                 PX4 / 'ROMFS/px4fmu_common/init.d-posix/airframes/'
                 '10052_gazebo-classic_inspection_quad')
    shutil.copy2(PACKAGE / 'config/inspection.px4board',
                 PX4 / 'boards/px4/sitl/inspection.px4board')
    airframe_list = PX4 / 'ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt'
    replace_once(airframe_list, '\t10030_gazebo-classic_px4vision',
                 '\t10030_gazebo-classic_px4vision\n'
                 '\t10052_gazebo-classic_inspection_quad',
                 '\t10052_gazebo-classic_inspection_quad')
    target = PX4 / 'src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake'
    content = target.read_text()
    if '\n\t\tinspection_quad\n' not in content:
        marker = '\n\t\tiris\n'
        if marker not in content:
            raise SystemExit('Cannot find Iris model entry in PX4 Gazebo target list')
        target.write_text(content.replace(marker, marker + '\t\tinspection_quad\n', 1))
    # Source archives have no .git file. PX4's standard skip flag should also
    # suppress the build dependency on that absent file.
    git_cmake = PX4 / 'cmake/px4_git.cmake'
    cmake_text = git_cmake.read_text()
    if 'SUBMODULE_GIT_DEPENDS' not in cmake_text:
        cmake_text = cmake_text.replace(
            '\tadd_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/git_init_${NAME}.stamp',
            '\tset(SUBMODULE_GIT_DEPENDS ${PX4_SOURCE_DIR}/.gitmodules)\n'
            '\tif(NOT DEFINED ENV{GIT_SUBMODULES_ARE_EVIL})\n'
            '\t\tlist(APPEND SUBMODULE_GIT_DEPENDS ${PATH}/.git)\n'
            '\tendif()\n\n'
            '\tadd_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/git_init_${NAME}.stamp', 1)
        cmake_text = cmake_text.replace(
            'DEPENDS ${PX4_SOURCE_DIR}/.gitmodules ${PATH}/.git',
            'DEPENDS ${SUBMODULE_GIT_DEPENDS}', 1)
        git_cmake.write_text(cmake_text)
    version_py = PX4 / 'src/lib/version/px_update_git_header.py'
    version_text = version_py.read_text()
    if '# A source archive has no .git' not in version_text:
        start = version_text.index('# Mavlink\n')
        end = version_text.index('# NuttX\n', start)
        version_text = version_text[:start] + '''# Mavlink
if os.path.exists('src/modules/mavlink/mavlink/.git'):
    mavlink_git_version = subprocess.check_output(
        ['git', 'rev-parse', '--verify', 'HEAD'], cwd='src/modules/mavlink/mavlink',
        stderr=subprocess.STDOUT).decode('utf-8').strip()
else:
    # A source archive has no .git; PX4 records the exact revision as a gitlink.
    mavlink_git_version = subprocess.check_output(
        ['git', 'ls-tree', 'HEAD', 'src/modules/mavlink/mavlink'],
        stderr=subprocess.STDOUT).decode('utf-8').split()[2]
mavlink_git_version_short = mavlink_git_version[0:16]
header += f"""
#define MAVLINK_LIB_GIT_VERSION_STR  "{mavlink_git_version}"
#define MAVLINK_LIB_GIT_VERSION_BINARY 0x{mavlink_git_version_short}
"""


''' + version_text[end:]
        version_py.write_text(version_text)
    target_text = target.read_text()
    if 'PX4_GAZEBO_JOBS' not in target_text:
        old = '\tif(parallel_jobs LESS 1)\n\t\tset(parallel_jobs 1)\n\tendif()'
        new = old + '\n\tif(DEFINED ENV{PX4_GAZEBO_JOBS})\n\t\tset(parallel_jobs $ENV{PX4_GAZEBO_JOBS})\n\tendif()'
        replace_once(target, old, new, 'PX4_GAZEBO_JOBS')
    replace_once(target, '\t\t\t-DGENERATE_ROS_MODELS=ON',
                 '\t\t\t-DGENERATE_ROS_MODELS=ON\n'
                 '\t\t\t-DBUILD_GSTREAMER_PLUGIN=OFF\n'
                 '\t\t\t-DBUILD_OPTICALFLOW_PLUGIN=OFF', 'BUILD_OPTICALFLOW_PLUGIN=OFF')
    gazebo_cmake = GAZEBO / 'CMakeLists.txt'
    # Pinned Gazebo Classic IMU correction: physical interval-averaged
    # specific force; preserve the upstream copyright and noise model.
    for directory, name in [('src', 'gazebo_imu_plugin.cpp'), ('include', 'gazebo_imu_plugin.h')]:
        patch_source = PACKAGE / 'patches/gazebo_imu' / name
        target_source = GAZEBO / directory / name
        if target_source.read_bytes() != patch_source.read_bytes():
            shutil.copy2(patch_source, target_source)
    sitl_run = PX4 / 'Tools/simulation/gazebo-classic/sitl_run.sh'
    simulator_init = PX4 / 'ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator'
    replace_once(simulator_init,
                 '\tparam set-default EKF2_MULTI_IMU 3\n'
                 '\tparam set-default SENS_IMU_MODE 0',
                 '\tif [ "$PX4_SIM_MODEL" != "gazebo-classic_inspection_quad" ]; then\n'
                 '\t\tparam set-default EKF2_MULTI_IMU 3\n'
                 '\t\tparam set-default SENS_IMU_MODE 0\n'
                 '\tfi',
                 'gazebo-classic_inspection_quad')
    replace_once(sitl_run,
                 'if [[ -n "$ROS_VERSION" ]] && [ "$ROS_VERSION" == "2" ]; then\n'
                 '\tros_args="-s libgazebo_ros_init.so -s libgazebo_ros_factory.so"',
                 'if [[ "$ROS_VERSION" == "1" ]]; then\n'
                 '\tros_args="-s libgazebo_ros_paths_plugin.so -s libgazebo_ros_api_plugin.so"\n'
                 'elif [[ "$ROS_VERSION" == "2" ]]; then\n'
                 '\tros_args="-s libgazebo_ros_init.so -s libgazebo_ros_factory.so"',
                 'libgazebo_ros_paths_plugin.so')
    replace_once(sitl_run,
                 'while gz model --verbose --spawn-file="${modelpath}/${model}/${model_name}.sdf" --model-name=${model} -x 1.01 -y 0.98 -z 0.83 2>&1',
                 'spawn_z=0.83\n'
                 '\tif [ "$model" == "inspection_quad" ]; then spawn_z=0.17; fi\n'
                 '\twhile gz model --verbose --spawn-file="${modelpath}/${model}/${model_name}.sdf" --model-name=${model} -x 1.01 -y 0.98 -z "$spawn_z" 2>&1',
                 'spawn_z=0.83')
    replace_once(sitl_run,
                 '\twhile gz model --verbose --spawn-file="${modelpath}/${model}/${model_name}.sdf" --model-name=${model} -x 1.01 -y 0.98 -z "$spawn_z" 2>&1 | grep -q "An instance of Gazebo is not running."; do\n'
                 '\t\techo "gzserver not ready yet, trying again!"\n'
                 '\t\tsleep 1\n'
                 '\tdone',
                 '\t# inspection_ros_spawn: use the owned ROS service, not Gazebo discovery.\n'
                 '\tif [[ "$model" == "inspection_quad" && "$ROS_VERSION" == "1" ]]; then\n'
                 '\t\tif ! timeout --signal=TERM --kill-after=5s 60s rosrun gazebo_ros spawn_model -sdf '
                 '-file "${modelpath}/${model}/${model_name}.sdf" -model "$model" '
                 '-x 1.01 -y 0.98 -z "$spawn_z"; then\n'
                 '\t\t\techo "Inspection model spawn failed or timed out" >&2\n'
                 '\t\t\tkill -TERM "$SIM_PID" 2>/dev/null || true\n'
                 '\t\t\twait "$SIM_PID" 2>/dev/null || true\n'
                 '\t\t\texit 1\n'
                 '\t\tfi\n'
                 '\telse\n'
                 '\t\twhile gz model --verbose --spawn-file="${modelpath}/${model}/${model_name}.sdf" --model-name=${model} -x 1.01 -y 0.98 -z "$spawn_z" 2>&1 | grep -q "An instance of Gazebo is not running."; do\n'
                 '\t\t\techo "gzserver not ready yet, trying again!"\n'
                 '\t\t\tsleep 1\n'
                 '\t\tdone\n'
                 '\tfi',
                 'inspection_ros_spawn')
    replace_once(sitl_run,
                 '-x 1.01 -y 0.98 -z "$spawn_z"; then',
                 '-x "${DRONE_SPAWN_X:-1.01}" -y "${DRONE_SPAWN_Y:-0.98}" '
                 '-z "${DRONE_SPAWN_Z:-$spawn_z}" -Y "${DRONE_SPAWN_YAW:-0}"; then',
                 'DRONE_SPAWN_YAW')
    replace_once(sitl_run,
                 '\t\t\twait "$SIM_PID" 2>/dev/null || true\n',
                 '\t\t\t# inspection_spawn_cleanup_bounded\n'
                 '\t\t\tsleep 2\n'
                 '\t\t\tkill -KILL "$SIM_PID" 2>/dev/null || true\n'
                 '\t\t\twait "$SIM_PID" 2>/dev/null || true\n',
                 'inspection_spawn_cleanup_bounded')
    replace_once(gazebo_cmake,
                 'option(BUILD_GSTREAMER_PLUGIN "enable gstreamer plugin" ON)',
                 'option(BUILD_GSTREAMER_PLUGIN "enable gstreamer plugin" ON)\n'
                 'option(BUILD_OPTICALFLOW_PLUGIN "enable optical flow plugin" ON)',
                 'option(BUILD_OPTICALFLOW_PLUGIN')
    replace_once(gazebo_cmake, 'find_package(gazebo REQUIRED)',
                 'find_package(gazebo REQUIRED)\nfind_package(Qt5 COMPONENTS Core Widgets Test REQUIRED)',
                 'find_package(Qt5 COMPONENTS Core Widgets Test REQUIRED)')
    replace_once(gazebo_cmake,
                 'add_subdirectory( external/OpticalFlow OpticalFlow )\nset( OpticalFlow_LIBS "OpticalFlow" )',
                 'if (BUILD_OPTICALFLOW_PLUGIN)\n'
                 '  add_subdirectory( external/OpticalFlow OpticalFlow )\n'
                 '  set( OpticalFlow_LIBS "OpticalFlow" )\nendif()',
                 'if (BUILD_OPTICALFLOW_PLUGIN)\n  add_subdirectory')
    replace_once(gazebo_cmake,
                 'add_library(gazebo_opticalflow_plugin SHARED src/gazebo_opticalflow_plugin.cpp)',
                 'if (BUILD_OPTICALFLOW_PLUGIN)\n'
                 '  add_library(gazebo_opticalflow_plugin SHARED src/gazebo_opticalflow_plugin.cpp)\nendif()',
                 'if (BUILD_OPTICALFLOW_PLUGIN)\n  add_library(gazebo_opticalflow_plugin')
    replace_once(gazebo_cmake,
                 '  gazebo_opticalflow_plugin\n  gazebo_aruco_plugin',
                 '  gazebo_aruco_plugin', 'list(APPEND plugins gazebo_opticalflow_plugin)')
    replace_once(gazebo_cmake,
                 'target_link_libraries(gazebo_opticalflow_plugin ${OpticalFlow_LIBS})',
                 'if (BUILD_OPTICALFLOW_PLUGIN)\n'
                 '  target_link_libraries(gazebo_opticalflow_plugin ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${TinyXML_LIBRARIES} ${OpticalFlow_LIBS})\n'
                 '  list(APPEND plugins gazebo_opticalflow_plugin)\nendif()',
                 'list(APPEND plugins gazebo_opticalflow_plugin)')
    print('PX4 inspection_quad model, airframe and target are ready')


if __name__ == '__main__':
    main()
