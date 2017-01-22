#!/bin/bash

find src -name "*.cpp" -or -name "*.h" -or -name "*.hpp" | xargs etags --append
