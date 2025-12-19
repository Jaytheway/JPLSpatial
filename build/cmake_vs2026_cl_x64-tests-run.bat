@echo off
pushd ..
cmake --workflow --preset=vs-tests
popd
pause