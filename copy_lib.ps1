$origind_dir = "E://SimpleClothSimulator"
$target_dir = "E:\KinectCap\lib\"



# debug lib
Copy-Item -Path $origind_dir/build_debug/src/geometries/Debug/geo_libd.lib -Destination $target_dir/geo_libd.lib -Recurse -Force -ErrorAction Ignore
Copy-Item -Path $origind_dir/build_debug/src/cameras/Debug/cam_libd.lib -Destination $target_dir/cam_libd.lib -Recurse -Force -ErrorAction Ignore
Copy-Item -Path $origind_dir/build_debug/src/sim/Debug/sim_libd.lib -Destination $target_dir/sim_libd.lib -Recurse -Force -ErrorAction Ignore

# release lib
Copy-Item -Path $origind_dir/build_release/src/geometries/Release/geo_lib.lib -Destination $target_dir/geo_lib.lib -Recurse -Force -ErrorAction Ignore
Copy-Item -Path $origind_dir/build_release/src/cameras/Release/cam_lib.lib -Destination $target_dir/cam_lib.lib -Recurse -Force -ErrorAction Ignore
Copy-Item -Path $origind_dir/build_release/src/sim/Release/sim_lib.lib -Destination $target_dir/sim_lib.lib -Recurse -Force -ErrorAction Ignore
