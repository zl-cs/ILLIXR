#include <iostream>
#include <fstream> 
#include "common/plugin.hpp"
#include "common/threadloop.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/phonebook.hpp"

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>

#include "gldemo_output.pb.h"
#include "zlib.h"

using namespace ILLIXR;

class server_writer : public plugin {
public:
    server_writer(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_clock{pb->lookup_impl<RelativeClock>()}
		, _m_frame{sb->get_reader<frame_from_gldemo>("frame")}
		, uncompressed_size{ILLIXR::FB_HEIGHT*ILLIXR::FB_WIDTH*3}
    { 
		// Initialize eCAL and create a protobuf publisher
		eCAL::Initialize(0, NULL, "GLdemo Offloading Server-side Writer");
		publisher = eCAL::protobuf::CPublisher<gldemo_output_proto::Rendered_frame>("gldemo_output");

		sb->schedule<frame_from_gldemo>(id, "frame", [this](switchboard::ptr<const frame_from_gldemo> datum, std::size_t) {
			this->send_gldemo_output(datum);
		});
	}
    virtual void start() override {
		// compression_file.clear();
		// send_frame_file.clear(); 
		compression_file.open(compression_name);
		send_frame_file.open(send_frame_name); 
		if (!compression_file.is_open() || !send_frame_file.is_open()) {
			ILLIXR::abort("File open failed!!!");
		}

		plugin::start();
	}

private:
    const std::shared_ptr<switchboard> sb;
	const std::shared_ptr<RelativeClock>_m_clock; 
	switchboard::reader<frame_from_gldemo> _m_frame;

	eCAL::protobuf::CPublisher<gldemo_output_proto::Rendered_frame> publisher;

	uLong uncompressed_size;
	uLong compressed_size;
	unsigned char* outbuff[2] = {nullptr};  
	uLong sizes[2] = {0}; 

	std::string compression_name = "metrics/compression_time.txt";
	std::ofstream compression_file;
	std::string send_frame_name = "metrics/send_frame_time.txt";
	std::ofstream send_frame_file;
	time_point startCompressionTime, endCompressionTime; 
	time_point startSendFrame, endSendFrame; 

	void send_gldemo_output(switchboard::ptr<const frame_from_gldemo> datum) {
		// switchboard::ptr<const frame_from_gldemo> datum = _m_frame.get_ro_nullable(); 
		if (datum == nullptr) return; 
		std::cout << "*****Received a frame\n"; 
		// std::cout << "*****Compressed_size before: " << compressed_size << std::endl; 
		
		startCompressionTime = _m_clock->now(); 
		for (int eye_idx = 0; eye_idx < 2; eye_idx++) {
			compressed_size = compressBound(uncompressed_size); 
			if (outbuff[eye_idx] != nullptr) { free(outbuff[eye_idx]); outbuff[eye_idx] = nullptr; };
			outbuff[eye_idx] = (unsigned char*)malloc(compressed_size);
			int is_success = compress2(outbuff[eye_idx], &compressed_size, eye_idx == 0 ? datum->left : datum->right, uncompressed_size, Z_BEST_COMPRESSION); 
			std::cout << "*****Compressed_size after: "<< compressed_size << "\t" << eye_idx << std::endl; 
			if (is_success != Z_OK) {
				std::cout << "*****Compressed_size after: "<< compressed_size << "\t" << eye_idx << std::endl; 
				ILLIXR::abort("Image compression failed !!! ");
			}
			sizes[eye_idx] = compressed_size;
		}
		endCompressionTime = _m_clock->now(); 
		std::cout << "!!!FRAME COMPRESSION TIME: " << (endCompressionTime-startCompressionTime).count() << "\n"; 

		startSendFrame = _m_clock->now();
		gldemo_output_proto::image* left_img = new gldemo_output_proto::image();
		std::string left_to_transfer(reinterpret_cast<char*>(outbuff[0]), sizes[0]); 
		left_img->set_pixels(left_to_transfer);
		assert(left_to_transfer.size() == sizes[0] && "left image corrupted");
		left_img->set_size(sizes[0]); 

		gldemo_output_proto::image* right_img = new gldemo_output_proto::image();
		std::string right_to_transfer(reinterpret_cast<char*>(outbuff[1]), sizes[1]); // FIXME unsigned char vs char
		right_img->set_pixels(right_to_transfer); // FIXME should I dereference it or now? 
		assert(right_to_transfer.size() == sizes[1] && "right image corrupted");
		right_img->set_size(sizes[1]); 

		gldemo_output_proto::swap_indices* si = new gldemo_output_proto::swap_indices();
		si->set_front(datum->swap_indices[0]);
		si->set_back(datum->swap_indices[1]);

		gldemo_output_proto::Rendered_frame* rf = new gldemo_output_proto::Rendered_frame();
		rf->set_allocated_left(left_img);
		rf->set_allocated_right(right_img);
		rf->set_allocated_si(si); 

		publisher.Send(*rf);
		delete rf; 
		endSendFrame = _m_clock->now();
		std::cout << "!!!FRAME SENDING TIME: " << (endSendFrame-startSendFrame).count() << "\n"; 
	};

};

PLUGIN_MAIN(server_writer)
