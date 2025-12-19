@echo off
pushd ..
python Build/_build_util.py "Visual Studio 18 2026" "Build/VS2026_CL_x64_Tests" x64 -DBUILD_TESTING=ON -DJPL_FETCH_DEPS=ON %*
popd
pause