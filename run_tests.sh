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

ECMC="ecmc"
MASTER_TASK="master_task"
SENSOR_DIAGNOSTIC_CHECK="sensor_diagnostic_check"
packages=(
    # $ECMC
    $MASTER_TASK
    # $SENSOR_DIAGNOSTIC_CHECK
)

TOTAL_COVERAGE_OUTPUT="build/main_coverage.info"
TOTAL_COVERAGE_HTML_FOLDER="build/out"
TOTAL_COVERAGE_HTML_FILE="${TOTAL_COVERAGE_HTML_FOLDER}/index.html"

# Needed for tests that include testing of nodes, pubs, subs
echo "starting roscore ..."
roscore >/dev/null &
ROSCORE_PID=$!

echo "building packages ..."
yes | catkin clean >/dev/null
catkin build >/dev/null &
progress_wheel

echo "building tests ..."
catkin run_tests >/dev/null &
progress_wheel

for pkg in "${packages[@]}"; do
    TEST_EXECUTABLE="${pkg}-test"
    SRC_DIR="src/${pkg}/src"
    BUILD_DIR="build/${pkg}/CMakeFiles/${TEST_EXECUTABLE}.dir/src"
    COVERAGE_DIR="build/${pkg}/coverage"
    COVERAGE_OUTPUT="${COVERAGE_DIR}/main_coverage.info"

    printf "\r\n"
    echo "running ${pkg} tests ..."

    output="$(rosrun ${pkg} $TEST_EXECUTABLE)"
    if [[ $? -eq 1 ]] ; then
        :
        # Use to prevent commits if tests fail
    fi
    echo "$output"

    [ ! -d $COVERAGE_DIR ] && mkdir $COVERAGE_DIR

    echo "generating coverage ..."
    lcov --no-external -c -d $SRC_DIR -d $BUILD_DIR -o $COVERAGE_OUTPUT &>/dev/null
    if [[ ! -f $TOTAL_COVERAGE_OUTPUT ]] ; then
        lcov -a $COVERAGE_OUTPUT -o $TOTAL_COVERAGE_OUTPUT >/dev/null
    else
        lcov -a $TOTAL_COVERAGE_OUTPUT -a $COVERAGE_OUTPUT -o $TOTAL_COVERAGE_OUTPUT >/dev/null
    fi
done

printf "\r\n"
echo "killing roscore ..."
kill $ROSCORE_PID

printf "\r\n"
genhtml $TOTAL_COVERAGE_OUTPUT -o $TOTAL_COVERAGE_HTML_FOLDER >/dev/null
echo "To view coverage: ${TOTAL_COVERAGE_HTML_FILE}"
