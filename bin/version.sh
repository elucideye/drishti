DRISHTI_VERSION=$(grep -o -E "version \"v[0-9]+\.[0-9]+\.[0-9]+\"" ${DRISHTISDK}/CMakeLists.txt | awk '{print $2}' | tr -d \" | tr -d v)
