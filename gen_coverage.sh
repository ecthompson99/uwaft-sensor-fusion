#!/bin/bash

# Helper function
progress_wheel () {
    pid=$! # Process Id of the previous running command
    spin='-\|/'
    i=0
    while kill -0 $pid 2>/dev/null
    do
        i=$(( (i+1) %4 ))
        printf "\r${spin:$i:1}"
        sleep .1
    done
    printf "\r\n"
}

# ECMC="ecmc"
MASTER_TASK="master_task"
# SENSOR_DIAGNOSTIC_CHECK="sensor_diagnostic_check"
packages=(
    # $ECMC
    $MASTER_TASK
    # $SENSOR_DIAGNOSTIC_CHECK
)

for pkg in "${packages[@]}"; do 
    TEST_EXECUTABLE="${pkg}-test"
    TEST_SRC_DIR="src/${pkg}/test"
    SRC_SRC_DIR="src/${pkg}/src"
    TEST_BUILD_DIR="build/${pkg}/CMakeFiles/${TEST_EXECUTABLE}.dir/test"
    SRC_BUILD_DIR="build/${pkg}/CMakeFiles/${TEST_EXECUTABLE}.dir/src"
    COVERAGE_DIR="build/${pkg}/coverage"
    COVERAGE_OUTPUT="${COVERAGE_DIR}/main_coverage.info"
    COVERAGE_HTML_FOLDER="${COVERAGE_DIR}/out"
    COVERAGE_HTML_FILE="${COVERAGE_HTML_FOLDER}/index.html"

    yes | catkin clean >/dev/null
    catkin build >/dev/null &
    progress_wheel

    catkin run_tests >/dev/null &
    progress_wheel

    output="$(rosrun ${pkg} $TEST_EXECUTABLE)"
    if [[ $? -eq 1 ]] ; then
        :
        # Use to prevent commits if unit tests fail
    fi
    echo "$output"

    [ ! -d $COVERAGE_DIR ] && mkdir $COVERAGE_DIR

    lcov --no-external -c -d $TEST_SRC_DIR -d $SRC_SRC_DIR -d $TEST_BUILD_DIR -d $SRC_BUILD_DIR -o $COVERAGE_OUTPUT >/dev/null
    lcov --remove $COVERAGE_OUTPUT "/usr*" "*/devel*" -o $COVERAGE_OUTPUT >/dev/null
    genhtml $COVERAGE_OUTPUT -o $COVERAGE_HTML_FOLDER >/dev/null
    echo "To view coverage: ${COVERAGE_HTML_FILE}"
done
