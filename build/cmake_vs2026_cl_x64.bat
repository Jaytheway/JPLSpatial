@echo off
pushd ..
python Build/_build_util.py "Visual Studio 18 2026" "Build/VS2026_CL_x64" x64 -DBUILD_TESTING=OFF -DJPL_FETCH_DEPS=OFF %*
popd
pause