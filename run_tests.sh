# Call this script with an optional build directory name (defaults to ./build)
echo Running all tests found in ${1-build}/bin...
find ${1-build}/bin -name "test_*" -perm /u+x | while read test; do echo "Now running $test..." && ./$test && echo ""; done
