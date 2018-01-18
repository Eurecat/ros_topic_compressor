#include <topic_compressor/TopicCompressed.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <lz4.h>
#include <boost/algorithm/string.hpp>

class Decompressor
{
public:
  Decompressor(ros::NodeHandle& nh, const std::string& source_topics):
    nh_(nh)
  {
      //Notify the compression server that you want a certain set of topics to be compressed
      ros::Publisher pub_to_server = nh_.advertise<std_msgs::String>("topics_to_compress", 1);

      std_msgs::String request_for_compression;
      request_for_compression.data = source_topics;
      pub_to_server.publish(request_for_compression);

      std::vector<std::string> split_vector;
      boost::split( split_vector, source_topics, boost::is_any_of(",;"), boost::token_compress_on );

      // first callabck is mandatory to generate the shape shifter
      for (const auto & topic_name: split_vector)
      {
        if( source_topics_.count(topic_name) != 0) continue;

        source_topics_.insert(topic_name);
        boost::function<void(const topic_compressor::TopicCompressedConstPtr& msg)> callback;
        callback  = boost::bind( &Decompressor::callback, this, _1, topic_name);
        ros::Subscriber subscriber = nh_.subscribe(topic_name + "/compressed", 1, callback);
        subscribers_.insert( std::make_pair( topic_name, subscriber ) );
      }
  }

  void callback(const topic_compressor::TopicCompressedConstPtr& msg, const std::string& topic_name)
  {  
    topic_tools::ShapeShifter shapeshifted_msg;
    std::vector<char> regen_buffer( msg->uncompressed_size );

    const int decompressed_size = LZ4_decompress_safe(reinterpret_cast<const char*>(msg->data.data()),
                                                      regen_buffer.data(),
                                                      msg->data.size(),
                                                      msg->uncompressed_size);

    ROS_DEBUG("decompressing %s from %d to %d\n", topic_name.c_str(), (int)msg->data.size(), decompressed_size );

    if( decompressed_size != msg->uncompressed_size )
    {
      ROS_ERROR("decompressing mismatch %d to %d\n", msg->uncompressed_size , decompressed_size );
      return;
    }

    ros::serialization::IStream stream( reinterpret_cast<uint8_t*>(regen_buffer.data()), regen_buffer.size() );
    shapeshifted_msg.read( stream );

    shapeshifted_msg.morph(msg->md5sum, msg->datatype, msg->definition, "");

    // This condition is true if it is the first time we
    auto pub_it = publishers_.find(topic_name);

    if( pub_it == publishers_.end() )
    {
      auto callback = boost::bind( &Decompressor::onSubscriptionCallback, this, _1, topic_name);
      ros::Publisher publisher = shapeshifted_msg.advertise( nh_, topic_name + "/decompressed", 1, true, callback);
      auto res = publishers_.insert( std::make_pair(topic_name, publisher) );
      pub_it = res.first;
    }

    pub_it->second.publish(shapeshifted_msg);

    // if no one is listening anymore, unsubscribe
    if(pub_it->second.getNumSubscribers() == 0)
    {
      auto sub_it = subscribers_.find(topic_name);
      if( sub_it != subscribers_.end())
      {
        sub_it->second.shutdown();
      }
    }
  }

  void onSubscriptionCallback(const ros::SingleSubscriberPublisher&, const std::string& topic_name)
  {
    // if someone subscribe to the uncompressed message, the subscribe this client to the
    boost::function<void(const topic_compressor::TopicCompressedConstPtr& msg)> callback;
    callback  = boost::bind( &Decompressor::callback, this, _1, topic_name);
    auto subscriber = nh_.subscribe(topic_name + "/compressed", 1, callback);
    subscribers_.insert( std::make_pair( topic_name, subscriber ) );
  }

private:

  ros::NodeHandle& nh_;
  std::map<std::string,ros::Publisher>  publishers_;
  std::map<std::string,ros::Subscriber> subscribers_;
  std::set<std::string> source_topics_;

};

//-----------------------------------------------------------------------

// usage: pass the name of the file as command line argument
int main(int argc, char** argv)
{
  ros::init(argc, argv, "topic_decompressor_client");

  ros::NodeHandle nh;

  std::string topic_in;
  if( !nh.getParam("topics", topic_in) )
  {
      ROS_FATAL("missing parameter [topics]. Use the arguments _topics:=compressed_topic (comma separated if more than one)");
      return 1;
  }

  Decompressor decomp(nh, topic_in);

  ros::spin();

  return 0;
}
