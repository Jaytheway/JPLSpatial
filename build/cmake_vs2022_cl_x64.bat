@echo off
pushd ..
python Build/_build_util.py "Visual Studio 17 2022" "Build/VS2022_CL_x64" x64 -DBUILD_TESTING=OFF -DJPL_FETCH_DEPS=OFF %*
popd
pause