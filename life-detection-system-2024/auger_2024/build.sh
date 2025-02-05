#!/bin/bash

# Get directory path of this script and export as enviroment
# will this become an issue when building all scripts at once?

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )


./../../tools/shared-build.sh $SCRIPT_DIR