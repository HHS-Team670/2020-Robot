#!/usr/bin/env bash
function changejavaver {
unset JAVA_HOME
export JAVA_VERSION=$@
}

# Command to Change Java Version to Get it to Compile:
# . changejavaversion.sh
# changejavaver 11

# Command to Change it Back:
# . changejavaversion.sh
# changejavaver 16