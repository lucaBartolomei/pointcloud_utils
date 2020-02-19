# Pointcloud Utils
Set of ROS nodes to do some processing with pointclouds.

## PLY Reader
There two ways of using this node:
1. Read a `ply` file and publish the vertices of the mesh as a PointCloud2 topic in ROS.
2. Read a Collada `dae` file and publish it as Marker in RViz.

### Read a ply file
First thing, create a mesh. To import a mesh, in meshlab import the `ply` or `obj` file. If you need to add a mesh: `Filters -> Set Texture` and select the texture (generally is a image file). Once you have a texture, project it to the vertices: `Filters -> Transfer: Texture to vertex color`. Export the mesh and start the node:
```
$ roslaunch pointcloud_utils ply_reader_node.launch ply_path:="path/to/mesh"
```  

### Read a Collada file
Even simpler. Just run:  
```
$ roslaunch pointcloud_utils ply_reader_node.launch collada_path:="path/to/collada/file"
```  
and then call the service `rosservice call /ply_reader_node/publish_collada`.

## Repeat a PointCloud2 message
Adjust the topics name in the launch file and then run:  
```
$ roslaunch pointcloud_utils pointcloud_repeater_node.launch
```  

## Create Mesh from PointCloud2 message
Adjust the topics name in the launch file and then run:  
```
$ roslaunch pointcloud_utils mesh_reconstructor.launch
```  

## Pointcloud filter node
Adjust the topics name and the parameters in the launch file and then run:  
```
$ roslaunch pointcloud_utils pointcloud_grid_filter.launch
```  
