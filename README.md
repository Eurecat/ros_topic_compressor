# ros_topic_compressor

Automatically compress and decompress ROS messages. Usefull to subscribe to large messages over Wifi.

Do you have problem with large messages, such as PointCloud2 or OccupancyGrid when
you connect to a remote robot using Wifi? This package is for you.

## Usage

Let's suppose that you want to compress a message of type PointCloud2 and named __/velodyne_points__.

* Run topic_compress_server on the robot, or the PC where the large messages are being published.

	  rosrun topic_compressor topic_compress_server

* On the other PC (the subscriber side) run:
     
      rosrun topic_compressor topic_compress_client _topics:=/velodyne_points
      
* Note that you can ask pass to the client multiple, comma separated, names of topics.
        
For each of the specified topics:

1. The server will compress the messages and publish a compressed version of it, with
the suffix "/compressed". In this case __/velodyne_points/compressed__

2. The client will publish __/velodyne_points/decompressed__

3. _Only_ if another ROS Node is subscribed to __/velodyne_points/decompressed__,
the communication between the server and the client take place. in other words, if no one 
use the decompressed message, no CPU not network bandwidth is wasted.
