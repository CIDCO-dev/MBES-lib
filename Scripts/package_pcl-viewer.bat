@echo off

copy C:\Users\cidco\Library\pclDLL\pcl_common_release.dll build\\bin\\pcl-viewer
copy C:\Users\cidco\Library\pclDLL\pcl_visualization_release.dll build\\bin\\pcl-viewer
copy C:\Users\cidco\Library\pclDLL\pcl_kdtree_release.dll build\\bin\\pcl-viewer
copy C:\Users\cidco\Library\pclDLL\pcl_io_release.dll build\\bin\\pcl-viewer
copy C:\Users\cidco\Library\pclDLL\pcl_io_ply_release.dll build\\bin\\pcl-viewer

copy C:\Users\cidco\Library\runtimeDLL\* build\\bin\\pcl-viewer

"C:\Program Files\7-Zip\7z.exe" a -r pcl-viewer.zip -w build\bin\pcl-viewer\.
move pcl-viewer.zip build\bin
.168
