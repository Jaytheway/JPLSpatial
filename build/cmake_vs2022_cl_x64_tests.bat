@echo off
pushd ..
python Build/_build_util.py "Visual Studio 17 2022" "Build/VS2022_CL_x64_Tests" x64 -DBUILD_TESTING=ON -DJPL_FETCH_DEPS=ON %*
popd
pause