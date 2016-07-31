#!/bin/bash

find . -name "*.cpp" -or -name "*.h" -or -name "*.hpp" | xargs etags --append
