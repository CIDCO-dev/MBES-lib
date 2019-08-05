@echo off

copy C:\Users\cidco\Library\pclDLL\pcl_surface_release.dll build\\bin\\overlap
copy C:\Users\cidco\Library\pclDLL\pcl_segmentation_release.dll build\\bin\\overlap
copy C:\Users\cidco\Library\pclDLL\pcl_filters_release.dll build\\bin\\overlap
copy C:\Users\cidco\Library\pclDLL\pcl_visualization_release.dll build\\bin\\overlap
copy C:\Users\cidco\Library\pclDLL\pcl_common_release.dll build\\bin\\overlap
copy C:\Users\cidco\Library\pclDLL\pcl_kdtree_release.dll build\\bin\\overlap
copy C:\Users\cidco\Library\pclDLL\pcl_search_release.dll build\\bin\\overlap
copy C:\Users\cidco\Library\pclDLL\pcl_features_release.dll build\\bin\\overlap
copy C:\Users\cidco\Library\pclDLL\pcl_io_release.dll build\\bin\\overlap
copy C:\Users\cidco\Library\pclDLL\pcl_io_ply_release.dll build\\bin\\overlap
copy C:\Users\cidco\Library\pclDLL\pcl_ml_release.dll build\\bin\\overlap
copy C:\Users\cidco\Library\pclDLL\pcl_octree_release.dll build\\bin\\overlap
copy C:\Users\cidco\Library\pclDLL\pcl_sample_consensus_release.dll build\\bin\\overlap

copy C:\Users\cidco\Library\runtimeDLL\* build\\bin\\overlap

"C:\Program Files\7-Zip\7z.exe" a -r overlap.zip -w build\bin\overlap\.
move overlap.zip build\bin
