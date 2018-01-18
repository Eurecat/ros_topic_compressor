#include <topic_compressor/TopicCompressed.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <lz4.h>

class Decompressor
{
public:
  Decompressor(ros::NodeHandle& nh, const std::string& source_topic,   const std::string& destination_topic):
    nh_(nh),
    source_topic_(source_topic),
    destination_topic_(destination_topic),
    publisher_created_(false)
  {
    subscriber_ = nh.subscribe<topic_compressor::TopicCompressed>(source_topic, 1,  &Decompressor::callback, this);
  }

  void callback(const topic_compressor::TopicCompressedConstPtr& msg)
  {  
    topic_tools::ShapeShifter shapeshifted_msg;
    std::vector<char> regen_buffer( msg->uncompressed_size );

    const int decompressed_size = LZ4_decompress_safe(reinterpret_cast<const char*>(msg->data.data()),
                                                      regen_buffer.data(),
                                                      msg->data.size(),
                                                      msg->uncompressed_size);

    ROS_INFO("decompressing from %d to %d\n", (int)msg->data.size(), decompressed_size );

    if( decompressed_size != msg->uncompressed_size )
    {
      ROS_ERROR("decompressing mismatch %d to %d\n", msg->uncompressed_size , decompressed_size );
      return;
    }

    ros::serialization::IStream stream( reinterpret_cast<uint8_t*>(regen_buffer.data()), regen_buffer.size() );
    shapeshifted_msg.read( stream );

    shapeshifted_msg.morph(msg->md5sum, msg->datatype, msg->definition, "");
    if( !publisher_created_ )
    {
      publisher_created_ = true;
      publisher_ = shapeshifted_msg.advertise( nh_, destination_topic_, 1, true,
                                               boost::bind( &Decompressor::onSubscriptionCallback, this, _1));
    }
    publisher_.publish(shapeshifted_msg);

    if( publisher_created_ )
    {
      if(publisher_.getNumSubscribers() == 0)
      {
        subscriber_.shutdown();
      }
    }
  }

  void onSubscriptionCallback(const ros::SingleSubscriberPublisher&)
  {
    subscriber_ = nh_.subscribe<topic_compressor::TopicCompressed>(source_topic_, 1,  &Decompressor::callback, this);
  }

private:

  ros::NodeHandle& nh_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  std::string destination_topic_, source_topic_;
  bool publisher_created_;

};

//-----------------------------------------------------------------------

// usage: pass the name of the file as command line argument
int main(int argc, char** argv)
{
  if( argc < 3 ){
    ROS_ERROR("Usage: need 2 argument with the name of the source and destination topic\n");
    return 1;
  }

  ros::init(argc, argv, "universal_decompressor");

  ros::NodeHandle nh;

  std::string topic_in;
  if( !nh.getParam("in", topic_in) )
  {
      ROS_FATAL("missing parameter [in]. Use the arguments _in:=compressed_topic _out:=decompressed_topic");
      return 1;
  }

  std::string topic_out;
  if( !nh.getParam("out", topic_out) )
  {
      ROS_FATAL("missing parameter [out]. Use the arguments _in:=compressed_topic _out:=decompressed_topic");
      return 1;
  }

  Decompressor decomp(nh, topic_in, topic_out);

  ros::spin();

  return 0;
}
