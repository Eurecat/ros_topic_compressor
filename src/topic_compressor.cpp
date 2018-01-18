#include <topic_compressor/TopicCompressed.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <lz4.h>

ros::Subscriber subscriber;
ros::Publisher  publisher;

void topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg)
{
  const int src_size = msg->size();
  topic_compressor::TopicCompressed out_msg;
  out_msg.md5sum = msg->getMD5Sum();
  out_msg.datatype = msg->getDataType();
  out_msg.definition = msg->getMessageDefinition();
  out_msg.data.resize(src_size);
  out_msg.uncompressed_size = src_size;

  std::vector<uint8_t> src_buffer;

  // copy raw memory into the buffer
  src_buffer.resize( msg->size() );
  ros::serialization::OStream stream(src_buffer.data(), src_size);
  msg->write(stream);

  const int max_dst_size = LZ4_compressBound(src_size);
  out_msg.data.resize(max_dst_size);

  const int compressed_data_size = LZ4_compress_default(reinterpret_cast<const char*>(src_buffer.data()),
                                                        reinterpret_cast<char*>(out_msg.data.data()),
                                                        src_size,
                                                        max_dst_size);
  out_msg.data.resize(compressed_data_size);
  publisher.publish(out_msg);
  ROS_INFO("compressing from %d to %d\n",src_size, compressed_data_size );
}


// usage: pass the name of the file as command line argument
int main(int argc, char** argv)
{
  ros::init(argc, argv, "universal_compresser");
  ros::NodeHandle nh;

  std::string topic_name;
  if( !nh.getParam("in", topic_name) )
  {
      ROS_FATAL("missing parameter [in]. Specify the name of the topic to compress with the argument _in:=topic_name");
      return 1;
  }

  const std::string published_topic = topic_name + std::string("/compressed");
  publisher  = nh.advertise<topic_compressor::TopicCompressed>(published_topic,2);
  subscriber = nh.subscribe(topic_name, 1, topicCallback );

  ros::spin();

  return 0;
}
