#include <stdio.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <boost/thread.hpp>

#include <ros/ros.h>

#include "audio_common_msgs/AudioData.h"

namespace audio_transport // Create a namespace called audio_transport
{
  class RosGstCapture // Create a class called RosGstCapture
  {
    public: 
      RosGstCapture() // Constructor
      {
        _bitrate = 192; // Default value for _bitrate (underscore indicates private member variable)

        std::string dst_type;

        // The bitrate at which to encode the audio
        ros::param::param<int>("~bitrate", _bitrate, 192); // Parameter name; Parameter variable; Default value

        // The destination of the audio
        ros::param::param<std::string>("~dst", dst_type, "appsink");

        // The source of the audio
        //ros::param::param<std::string>("~src", source_type, "alsasrc");

        _pub = _nh.advertise<audio_common_msgs::AudioData>("audio_user", 10, true); // Topic name: /audio_user

        _loop = g_main_loop_new(NULL, false);
        _pipeline = gst_pipeline_new("ros_pipeline"); // Create a new pipeline which consists of elements

        // We create the sink first, just for convenience (Sink is the kind of pad through which elements accept incoming data)
        if (dst_type == "appsink")
        {
        	/* gst_element_factory_make() create a new element of the type defined by the given element factory.
        	 *  If name is NULL, then the element will receive a guaranteed unique name,
        	 *  consisting of the element factory name and a number. If name is given,
        	 *  it will be given the name supplied. */

        	/* Allow the application to get access to raw buffer. Appsink is a sink plugin that
        	 * supports many different methods for making the application get a handle on the
        	 * GStreamer data in a pipeline. */
          _sink = gst_element_factory_make("appsink", "sink");
          // Set attributes of _sink element
          g_object_set(G_OBJECT(_sink), "emit-signals", true, NULL);
          g_object_set(G_OBJECT(_sink), "max-buffers", 100, NULL);
          g_signal_connect( G_OBJECT(_sink), "new-buffer", 
                            G_CALLBACK(onNewBuffer), this);
        }
        else
        {
          printf("file sink\n");
          _sink = gst_element_factory_make("filesink", "sink");
          g_object_set( G_OBJECT(_sink), "location", dst_type.c_str(), NULL);
        }

        _source = gst_element_factory_make("alsasrc", "source"); // Read from a sound card via ALSA
        _convert = gst_element_factory_make("audioconvert", "convert");

        _encode = gst_element_factory_make("lame", "encoder");
        g_object_set( G_OBJECT(_encode), "preset", 1001, NULL);
        g_object_set( G_OBJECT(_encode), "bitrate", _bitrate, NULL);

        gst_bin_add_many( GST_BIN(_pipeline), _source, _convert, _encode, _sink, NULL); // Add elements into pipeline
        gst_element_link_many(_source, _convert, _encode, _sink, NULL); // Link the elements in sequence
        /*}
        else
        {
          _sleep_time = 10000;
          _source = gst_element_factory_make("filesrc", "source");
          g_object_set(G_OBJECT(_source), "location", source_type.c_str(), NULL);

          gst_bin_add_many( GST_BIN(_pipeline), _source, _sink, NULL);
          gst_element_link_many(_source, _sink, NULL);
        }
        */

        gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING); // Set the state of the pipeline to PLAYING

        _gst_thread = boost::thread( boost::bind(g_main_loop_run, _loop) );
      } // End of constuctor

      void publish( const audio_common_msgs::AudioData &msg )
      {
        _pub.publish(msg);
      }

      static GstFlowReturn onNewBuffer (GstAppSink *appsink, gpointer userData)
      {
        RosGstCapture *server = reinterpret_cast<RosGstCapture*>(userData);

        GstBuffer *buffer;
        g_signal_emit_by_name(appsink, "pull-buffer", &buffer);

        audio_common_msgs::AudioData msg;
        msg.data.resize( buffer->size );
        memcpy( &msg.data[0], buffer->data, buffer->size);

        server->publish(msg);

        return GST_FLOW_OK;
      }

    private:
      ros::NodeHandle _nh;
      ros::Publisher _pub;

      boost::thread _gst_thread;

      GstElement *_pipeline, *_source, *_sink, *_convert, *_encode;
      GMainLoop *_loop; // Pointer of the main loop
      int _bitrate;
  };
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "audio_capture_user"); // Node name: audio_capture_user
  gst_init(&argc, &argv); // Initialize GStreamer

  audio_transport::RosGstCapture server;
  ros::spin();
}
