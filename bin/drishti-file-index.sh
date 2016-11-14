#!/bin/bash

find src -name "*.cpp" -or -name "*.h" -or -name "*.hpp" | xargs wc -l | sort -n -k1
