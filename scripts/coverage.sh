#!/bin/bash
 
# Get the path to the current folder
SCRIPT_DIR=$(dirname $0)

# SRC_DIR is the directory containing the .gcno files (%{buildDir} in Qt Creator)
SRC_DIR="$SCRIPT_DIR/../build-tests-Desktop_Qt_5_13_1_GCC_64bit-Debug/"

# COV_DIR is the directory where the coverage results will be stored
COV_DIR="$SCRIPT_DIR/../doc/coverage/"

############################################################################################################

# Path where the HTML files should be saved
HTML_RESULTS="${COV_DIR}""/html/"

# Create the html folder if it does not exists
mkdir -p ${HTML_RESULTS}

# Generate our initial info
lcov -d "${SRC_DIR}" -c -o "${COV_DIR}/coverage.info"
 
# Generate the HTML files
genhtml -o "${HTML_RESULTS}" "${COV_DIR}/coverage-filtered.info"
 
# Reset our counts
lcov -d "${COV_DIR}" -z
 
# Open the index.html
#open "${HTML_RESULTS}/index.html"
