#include <topic_compressor/TopicCompressed.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <lz4.h>
#include <boost/algorithm/string.hpp>

class Compressor
{

public:
  Compressor(): nh_("~")
  {
    compression_request_ = nh_.subscribe("/topics_to_compress", 10,
                                         &Compressor::requestForCompressionCallback,
                                         this);
  }

private:
  ros::NodeHandle nh_;
  std::map<std::string,ros::Subscriber> subscribers_;
  std::map<std::string,ros::Publisher>  publishers_;
  ros::Subscriber compression_request_;

  void requestForCompressionCallback(const std_msgs::StringConstPtr& topics)
  {
    std::vector<std::string> split_vector;
    boost::split( split_vector, topics->data,
                  boost::is_any_of(",;"), boost::token_compress_on );

    for (const auto & topic_name: split_vector)
    {
      boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;
      callback = boost::bind(&Compressor::topicCallback, this, _1, topic_name );

      ROS_INFO("Subscribing to: %s", topic_name.c_str());

      auto subscriber = nh_.subscribe(topic_name, 1, callback);
      auto publisher = nh_.advertise<topic_compressor::TopicCompressed>(
            topic_name + "/compressed",  1);

      subscribers_.insert( std::make_pair( topic_name, subscriber) );
      publishers_.insert(  std::make_pair( topic_name, publisher) );
    }
  }

  void topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg,
                     const std::string& topic_name)
  {
    ros::Publisher* publisher = nullptr;
    auto pub_it = publishers_.find(topic_name);
    if(pub_it == publishers_.end())
    {
      return;
    }
    else{
      publisher = &(pub_it->second);
    }

    if( publisher->getNumSubscribers() == 0)
    {
    //  ROS_DEBUG("no subscriber: skip");
      return;
    }

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
    publisher->publish(out_msg);

    ROS_DEBUG("compressing %s from %d to %d\n",
              topic_name.c_str(),
              src_size,
              compressed_data_size );
  }
};

// usage: pass the name of the file as command line argument
int main(int argc, char** argv)
{
  ros::init(argc, argv, "topic_compression_server");

  Compressor compressor;

  ros::spin();

  return 0;
}
